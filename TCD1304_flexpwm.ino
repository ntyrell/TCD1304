 /*
 * TCD1304      - Teensy 4.0
 * ------------------------
 * pin 3 (ICG)  - pin 5
 * pin 4 (MCLK) - pin 6
 * pin 5 (SH)   - pin 4
 */

#include <Ethernet3.h>
#include <EthernetUdp3.h> 
#include <ADC.h>
#include <ADC_util.h>
#include <SPI.h> 

#define PIXELS 3648
#define MCLK 4 // target TCD1304 clock rate in MHz; we will get as close as possible
#define UDP_TX_PACKET_MAX_SIZE 3*PIXELS/2 // redefine max packet size

// calculate counter values such that an integer number of both 128x divided fbus cycles and MCLK cycles happen during each frame
int CLKPMCLK = ceil((float)F_BUS_ACTUAL/(MCLK*1000000)); // f bus clock cycles per ccd clock cycles
int MCLKPF = ceil((float)(32+PIXELS+14)*4/128); // number of ccd clock cycles per frame
int CNTPF = MCLKPF * CLKPMCLK; // number of pwm counts per frame
int CLKPF = 128 * CNTPF; // bus clock ticks per frame
int USPF = CLKPF / (F_BUS_ACTUAL / 1000000); // microseconds per frame
// make sure that 6 + 1 + 0.5 > 128*116 - (32+PIXELS+14)*4 i.e. there's enough extra time between pixels to set ICG low then high
int CNT_ICG = (6.0 * (F_BUS_ACTUAL / 1000000))/128; 
int ES = 128; // how many times to divide framerate for electronic shutter [1 2 4 8 16 32 64 128]
int CNT_SH = (1.0 * (F_BUS_ACTUAL / 1000000))/ (128 / ES); // divide by whatever prescaler to set shorter integration time 
int off = (0.5 * (F_BUS_ACTUAL / 1000000))/(128 / ES); // delay time between ICG low and SH high
int adc_off = (0.0 * (F_BUS_ACTUAL / 1000000)); // delay time between ICG high and ADC sample start 
// for now the MCLK starts totally synchronized with ICG high edge but we could add a delay there too (datasheet says 0 delay is ok). I am not sure if I should change the polarity of the clock (do we want a rising or falling edge first?)

#include "setup_flexpwm.h"

ADC *adc; // adc object

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(169, 254, 1, 1);
IPAddress subnet(255, 255, 0, 0);
EthernetUDP Udp;
unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

int i = 0;
int ii = 0;

int s = 1;
byte vals[2][3*PIXELS/2];
int j = 0;
int f = 0;

uint16_t sample = 0;
uint16_t sample_prev = 0;

void flexpwm2_1_isr() {
  // resets the pixel count before ICG goes high to signal new frame
  FLEXPWM2_SM1STS = 1;  // clear interrupt flag for sm 0
  while (FLEXPWM2_SM1STS == 1); // wait for clear
  i = 0;
  ii = 0;
}

void flexpwm2_3_isr() {
  FLEXPWM2_SM3STS |= 12;  // clear all set bits

  if ((i >=32) && (i < (32 + PIXELS))) {
    // do a fast read of adc here!
    // wait for conversion and sample ccd
    sample_prev = sample;
    while (!adc->adc0->isComplete()); // wait for conversion
    sample = (uint16_t)adc->adc0->analogReadContinuous();
    if (i%2 == 1) { // only odd indices
      // bit shift and stuff to fit 2 samples into 3 bytes, MSB first
      vals[j][ii] = highByte(sample_prev) << 4 | lowByte(sample_prev) >> 4;
      vals[j][ii+1] = lowByte(sample_prev) << 4 | highByte(sample);
      vals[j][ii+2] = lowByte(sample);
      ii += 3;
      if (ii >= (3*PIXELS/2)){ // stop sampling before dummy outputs
        // probably trigger a switch to the other array to prevent overwriting, and trigger a send or something
        //ii = 0;
        j = !j;  
        f = 1; 
      }
    }
  }
  i += 1;
  
  //digitalWriteFast(7,s); // toggle pin for scoping
  //s = !s;
  //count += 1;
  
  //while (FLEXPWM2_SM3STS == 12); // wait for clear
}

void setup() {
  // set up adc here? for some reason ADC setup and interrupt attaching need to be last! otherwise the adc reading was super noisy...
  adc = new ADC();
  adc->adc0->setAveraging(1);  
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->startContinuous(A9); 
  
  pinMode(7, OUTPUT);
  
  Serial.begin(115200);

  Serial.println(USPF);
  Serial.println(CLKPF);
  Serial.println(CNTPF);
  Serial.println(CNT_ICG);
  Serial.println(CNT_SH);
  Serial.println(off);

  // set up clocks  
  CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
  
  //FLEXPWM2_MCTRL = 0; // make sure RUN flags are off
  FLEXPWM2_FCTRL0 = FLEXPWM_FCTRL0_FLVL(15); // logic high = fault
  FLEXPWM2_FSTS0 = 0x000F; // clear fault status
  FLEXPWM2_FFILT0 = 0;
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(15);

  setup_ICG();
  setup_SH();
  setup_MCLK();
  setup_ADCCLK();
  
  //Serial.print("FLEXPWM2_MCTRL: ");
  //Serial.println(FLEXPWM2_MCTRL);
  
  //FLEXPWM2_SM2CTRL2 |= 64; // force initialization ICG
  //FLEXPWM2_SM0CTRL2 |= 64; // force initialization SH
  // not actually initializing as expected..

  
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(15); // do counters initialize upon loading?
  //FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(15); // why is this not necessary?

  //attachInterrupt(3, beginCCDsampler, RISING);
  
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

  // attach interrupts
  // not sure why this works better. do i need to attach interrupts after ethernet starts??? apparently yes
  attachInterruptVector(IRQ_FLEXPWM2_1, flexpwm2_1_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_1);

  attachInterruptVector(IRQ_FLEXPWM2_3, flexpwm2_3_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_3);
  NVIC_SET_PRIORITY(IRQ_FLEXPWM2_3, 128);

}

/*
 * teensy 4 seems to have ADC noise issues while a packet is being
 * sent to the Ethernet board. I do not think these issues showed
 * up on the 3.6. It seems like noise can be minimized by keeping
 * the analog read pin far from the SPI clock pin (and presumably
 * other SPI pins) as well as keeping the SPI clock slow (setting
 * in W5500.h; I think 30MHz is a good speed since it takes almost
 * a whole frame to transfer one frame at 4MHz CCD clock rate). 
 * Presumably, noise will also improve if all signal traces are kept
 * as short as possible
 * 
 * one could also imagine a low noise mode where frame collecting
 * and streaming never occur at the same time. The laziest implemen-
 * tation of this would be to trigger an interrupt every time the
 * ICG counter fills up (indicating new frame) and to only send data
 * every other frame (always sending from the same j index of the 
 * ping pong buffer)
 * 
 * A more efficient mode would only delay for as much time as neces-
 * sary to stream the data to the ethernet board..
 * 
 */

byte (*packet)[3*PIXELS/2];

void loop(){
  //if ((f == 1) && (j == 1)){ // adding (j==1) ensures that we only send every other frame, and that the one we do send has no spi-associated adc noise..
  if (f == 1){
    Serial.println("sending frame!");
    f = 0;
    int ts = micros();
    packet = &vals[!j];
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(*packet, 3*PIXELS/2);
    Udp.endPacket();
    int tf = micros();
    Serial.print("elapsed time: ");
    Serial.println(tf - ts);
  }
}
