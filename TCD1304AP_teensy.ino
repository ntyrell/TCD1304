#include <Ethernet3.h>
#include <EthernetUdp3.h> 

/* Original sketch available at:
 * https://hackaday.io/project/18126-dav5-v301-raman-spectrometer/log/53099-using-an-arduino-r3-to-power-the-tcd1304ap-ccd-chip
 * 
 * TCD1304      - Teensy 3.6
 * ------------------------
 * pin 3 (ICG)  - pin 6 (D4)
 * pin 4 (MCLK) - pin 7 (D2)
 * pin 5 (SH)   - pin 8 (D3)
 */
#include <ADC.h>
#include <ADC_util.h>
#include <SPI.h>       

#define PIXELS 3648 // total number of data samples including dummy outputs
#define N 100 // number of subdivisions of framerate to expose for (electronic shutter)
#define CLOCK GPIOB_PDOR  // use output of GPIOB on Teensy 3.6

#define UDP_TX_PACKET_MAX_SIZE PIXELS*2 // redefine max packet size

#define ADCS (1<<0) // B0, teensy pin 16, ADC start signal pin
#define ADCF (1<<1) // B1, teensy pin 17, ADC finish signal pin

#define ICG (1<<18)  // B18 (TCD1304 pin 3, teensy pin 29)
#define MCLK (1<<19) // B19 (TCD1304 pin 4, teensy pin 30)
#define SH (1<<10)   // B10 (TCD1304 pin 5, teensy pin 31)  

#define F 1 // clock rate in MHz, 0.5 1 2 or 4 should work..

IntervalTimer frameSampler;
ADC *adc = new ADC(); // adc object

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(169, 254, 1, 1);
IPAddress subnet(255, 255, 0, 0);
EthernetUDP Udp;
unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

void setup(){
    // Enable the serial port.
  Serial.begin(921600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  pinMode(9, OUTPUT); // ethernet board reset pin
  digitalWrite(9, HIGH);
  delayMicroseconds(1000);
  digitalWrite(9, LOW);
  delayMicroseconds(1000);
  digitalWrite(9, HIGH);

  pinMode(10, OUTPUT);
  Ethernet.setCsPin(10); 
  Ethernet.init(1); // only initialize with 1 socket with 16k memory
  Ethernet.begin(mac, ip, subnet);
  //Ethernet.setSubnetMask(subnet); // teensy and rpi should have same subnet mask
  Udp.begin(localPort);

/*
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5100) {
    Serial.println("W5100 Ethernet controller detected.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5200) {
    Serial.println("W5200 Ethernet controller detected.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5500) {
    Serial.println("W5500 Ethernet controller detected.");
  }
*/
  Serial.print("max packet size [bytes]: ");
  Serial.println(UDP_TX_PACKET_MAX_SIZE);

  int packetSize = 0;
  while (packetSize == 0)
  {
    //Serial.println("checking for packet!");
    packetSize = Udp.parsePacket(); // wait
  }
  if (packetSize) 
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    for (int i =0; i < 4; i++)
    {
      Serial.print(Udp.remoteIP()[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  }

  //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  //Udp.write("hello");
  //Udp.endPacket();
  
  // set clock pins to output
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  CLOCK |= ICG; // Set the integration clear gate high.

  // generate clock frequency of 1MHz on teensy pin 7
  analogWriteFrequency(30, F*1000000);
  analogWrite(30, 124);

  // make ADC very fast
  adc->adc0->setAveraging(1);  
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->startContinuous(A14);   

  // to get longer exposure times, set N = 1 and add some extra time 
  frameSampler.begin(triggerCCD, (float) (6 + (32+PIXELS+14)* 4.0/((float) F)) / (float) N + 0.0);
  Serial.println("exiting setup");
}

IntervalTimer CCDsampler;
int c = 0;

void triggerCCD(){
  // this routine sends the message to the CCD to start sending data. CCD will begin when ICG goes high
  bool t = (c == 0);
  if (t) CLOCK &= ~ICG; // set ICG low
  delayMicroseconds(1); // timing requirement (1000 ns max)
  CLOCK |= SH;   // set SH high 
  delayMicroseconds(1); // timing requirement (1000 ns max)
  CLOCK &= ~SH;  // set SH low
  if (t) {
    CCDsampler.begin(sampleCCD, 4 / F); // it appears to take about 5us before the first ADC sample happens, but this appears to be related to the 5us delay..
    delayMicroseconds(4); // timing requirement (min 1000ns, typ 5000 ns); making this the same as (or a multiple of?) the ADC sample period seems to best align the first ADC sample with the rising edge of ICD; adding the delay seems to block the timer from executing
    CLOCK |= ICG;  // set ICG high
  }
  c++;
  if (c == N) c = 0; 
}

byte vals[2][2*PIXELS];
//uint16_t vals[PIXELS][2];
int i = 0;
int j = 0;
int f = 0;

void sampleCCD(){
  CLOCK ^= ADCS; // toggle ADC start read indicator pin
  // wait for conversion and sample ccd
  while (!adc->adc0->isComplete()); // wait for conversion
  uint16_t sample = (uint16_t)adc->adc0->analogReadContinuous();
  if (i >= 32) {
    int ii = i - 32;
    vals[j][2*ii] = lowByte(sample);
    vals[j][2*ii+1] = highByte(sample);
    //vals[i][j] = (uint16_t)adc->adc0->analogReadContinuous();
    CLOCK ^= ADCF; // toggle ADC finish read indicator pin; scope measurements show that this reading takes about 0.5us! which means that I could get close to the max clock rate of 4MHz (update: I tried 1MHz sampling of a 10kHz sine wave and it has a few glitches.. might be too much
    //i += 1;
    if (ii == (PIXELS-1)){ // stop sampling before dummy outputs
      CCDsampler.end();
      i = 0;
      // probably trigger a switch to the other array to prevent overwriting, and trigger a send or something
      j = !j;  
      f = 1; 
    }
  }
  i += 1;
}

byte (*packet)[PIXELS*2];


void loop(){
  if (f == 1){
    Serial.println("sending frame!");
    f = 0;
    int ts = micros();
    packet = &vals[!j];
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(*packet, PIXELS*2);
    Udp.endPacket();
    int tf = micros();
    Serial.print("elapsed time: ");
    Serial.println(tf - ts);
    
  }
}
