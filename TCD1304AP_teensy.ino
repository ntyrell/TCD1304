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

#define PIXELS 3648 // total number of data samples including dummy outputs
#define N 1 // number of subdivisions of framerate to expose for (electronic shutter)
#define CLOCK GPIOD_PDOR  // use output of GPIOD on Teensy 3.6

#define ADCS (1<<7) // D7, teensy pin 5, ADC start signal pin
#define ADCF (1<<0) // D7, teensy pin 2, ADC finish signal pin

#define ICG (1<<4)  // D4 (TCD1304 pin 3, teensy pin 6)
#define MCLK (1<<2) // D2 (TCD1304 pin 4, teensy pin 7)
#define SH (1<<3)   // D3 (TCD1304 pin 5, teensy pin 8)  

#define F 4 // clock rate in MHz, 0.5 1 2 or 4 should work..

IntervalTimer frameSampler;
ADC *adc = new ADC(); // adc object

void setup(){
  // set clock pins to output
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  CLOCK |= ICG; // Set the integration clear gate high.
  // Enable the serial port.
  Serial.begin(921600);

  // generate clock frequency of 1MHz on teensy pin 7
  analogWriteFrequency(7, F*1000000);
  analogWrite(7, 124);

  // make ADC very fast
  adc->adc0->setAveraging(1);  
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->startContinuous(A14);   

  // to get longer exposure times, set N = 1 and add some extra time 
  frameSampler.begin(triggerCCD, (float) (6 + (32+PIXELS+14)* 4.0/((float) F)) / (float) N + 0.0);
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

uint16_t vals[32+PIXELS][2];
int i = 0;
int j = 0;
int f = 0;

void sampleCCD(){
  CLOCK ^= ADCS; // toggle ADC start read indicator pin
  // wait for conversion and sample ccd
  while (!adc->adc0->isComplete()); // wait for conversion
  vals[i][j] = (uint16_t)adc->adc0->analogReadContinuous();
  CLOCK ^= ADCF; // toggle ADC finish read indicator pin; scope measurements show that this reading takes about 0.5us! which means that I could get close to the max clock rate of 4MHz (update: I tried 1MHz sampling of a 10kHz sine wave and it has a few glitches.. might be too much
  i += 1;
  if (i == (32+PIXELS)){ // stop sampling before dummy outputs
    CCDsampler.end();
    i = 0;
    // probably trigger a switch to the other array to prevent overwriting, and trigger a send or something
    j = !j;  
    f = 1; 
  }
}

void loop(){
  if (f == 1){
    f = 0;
    for (int ii = 32; ii < (32+PIXELS); ii++){
      // could change this to only send the real pixels, not dummies too
      Serial.print(vals[ii][!j]);
      Serial.print(",");
    }
    Serial.println();
  }
}
