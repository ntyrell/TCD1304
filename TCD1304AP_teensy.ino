/* Original sketch available at:
 * https://hackaday.io/project/18126-dav5-v301-raman-spectrometer/log/53099-using-an-arduino-r3-to-power-the-tcd1304ap-ccd-chip
 * 
 * TCD1304 - Teensy 3.6
 * ------------------------
 * pin 7 (ICG) - D2
 * pin 8 (MCLK) - D3
 * pin 6 (SH)- D4
 */
#include <ADC.h>
#include <ADC_util.h>
ADC *adc = new ADC(); // adc object

#define PIXELS 3694 // total number of data samples including dummy outputs

#define N 10 // number of subdivisions of framerate to expose for (electronic shutter)

#define CLOCK GPIOD_PDOR  // use output of GPIOD on Teensy 3.6
//#include <util/delay_basic.h>

#define ICG (1<<4)  // D4 (TCD1304 pin 3, teensy pin 6)
#define MCLK (1<<2) // D2 (TCD1304 pin 4, teensy pin 7)
#define SH (1<<3)   // D3 (TCD1304 pin 5, teensy pin 8)  

IntervalTimer frameSampler;

void setup(){
  // set clock pins to output
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(6, OUTPUT);
  CLOCK |= ICG; // Set the integration clear gate high.
  // Enable the serial port.
  Serial.begin(115200);

  // generate clock frequency of 1MHz on teensy pin 7
  analogWriteFrequency(7, 1000000);
  analogWrite(7, 124);

  // make ADC very fast
  adc->adc0->setAveraging(1);  
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->startContinuous(A14);   

  // to get longer exposure times, set N = 1 and add some extra time 
  frameSampler.begin(triggerCCD, (float)(7+PIXELS*4) / (float) N);
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
    delayMicroseconds(5); // timing requirement (min 1000ns, typ 5000 ns)
    if (t) {
      CLOCK |= ICG;  // set ICG high
      CCDsampler.begin(sampleCCD, 4); // try calling this here instead of triggering with external interrupt
    }
    c++;
    if (c == N) c = 0; 
}

uint16_t vals[PIXELS][2];
int i = 0;
int j = 0;
int f = 0;

void sampleCCD(){
  // wait for conversion and sample ccd
    while (!adc->adc0->isComplete()); // wait for conversion
    vals[i][j] = (uint16_t)adc->adc0->analogReadContinuous();
    i += 1;
    if (i == PIXELS){
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
    for (int ii = 0; ii < PIXELS; ii++){
      // could change this to only send the real pixels, not dummies too
      Serial.print(vals[ii][!j]);
      Serial.print(",");
    }
    Serial.println();
  }

}
