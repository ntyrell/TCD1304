#define PIXELS 3648
#define MCLK 4 // target TCD1304 clock rate in MHz; we will get as close as possible

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

void setup_ICG() {
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 1; // set pin 5 to be PWM2 A SM 01
  
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM1CTRL2 |= 128; // set force enable

  FLEXPWM2_SM1CTRL = 112; // set prescaler to 128
  FLEXPWM2_SM1CTRL |= 2048; // reload on half count
  
  FLEXPWM2_SM1OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  //FLEXPWM2_SM2OCTRL |= 1024; // set A polarity to 1, should swap high and low
  FLEXPWM2_SM1DTCNT0 = 0;
  FLEXPWM2_SM1INIT = 0; // set counter initialization to 0

  // we think of the waveform 0 as beginning when ICG goes high. We want all clocks to be synchronized relative to this moment
  FLEXPWM2_SM1VAL0 = CNTPF - 2; // set VAL0 to trigger an interrupt that will rest pixel count 128 clock cycles before ICG goes high; this should occur between ADC reads (which at max happen once ever 38*4 = 172 bus clock cycles), be careful if you have a large adc offset but i think that will stay near 0
  FLEXPWM2_SM1VAL1 = CNTPF - 1; 
  FLEXPWM2_SM1VAL2 = 0; 
  FLEXPWM2_SM1VAL3 = CNTPF - CNT_ICG;
  FLEXPWM2_SM1VAL4 = 0;
  FLEXPWM2_SM1VAL5 = 0;

  FLEXPWM2_OUTEN |= 512; // set output enable for A, module 1 

  // set up interrupt
  FLEXPWM2_SM1STS |= 0; // set val0 compare flag to 0
  FLEXPWM2_SM1INTEN = 1; // enable interrupts for val0 compare

  attachInterruptVector(IRQ_FLEXPWM2_1, flexpwm2_1_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_1);

}

int count = 0;

void flexpwm2_1_isr() {
  // resets the pixel count before ICG goes high to signal new frame
  FLEXPWM2_SM1STS = 1;  // clear interrupt flag for sm 0
  count = 0;
  while (FLEXPWM2_SM1STS == 1); // wait for clear
}


void setup_SH() {
  //IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 6; // set pin 7 to be PWM1 B SM 03
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 1; // set pin 4 to be PWM A SM 0

  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM0CTRL2 |= 128; // set force enable

  if (ES == 1) FLEXPWM2_SM0CTRL = 112; // set prescaler to 128
  if (ES == 2) FLEXPWM2_SM0CTRL = 96; // set prescaler to 64
  if (ES == 4) FLEXPWM2_SM0CTRL = 80; // set prescaler to 32
  if (ES == 8) FLEXPWM2_SM0CTRL = 64; // set prescaler to 16
  if (ES == 16) FLEXPWM2_SM0CTRL = 48; // set prescaler to 8
  if (ES == 32) FLEXPWM2_SM0CTRL = 32; // set prescaler to 4
  if (ES == 64) FLEXPWM2_SM0CTRL = 16; // set prescaler to 2

  // SH occurs 128 / SH_PRSC times during one frameout cycle, i.e. 1, 2, 4, 8, 16, 32, 128 times
  // could happen more times but would have to change val1
  
  FLEXPWM2_SM0CTRL |= 2048; // reload on half count (actually reloads immediately since val0 is set to 0)
  
  FLEXPWM2_SM0OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  //FLEXPWM2_SM0OCTRL |= 1024; // set A polarity to 1, should swap high and low
  FLEXPWM2_SM0DTCNT0 = 0;
  FLEXPWM2_SM0INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM0VAL0 = 0; 
  FLEXPWM2_SM0VAL1 = CNTPF - 1; 
  FLEXPWM2_SM0VAL2 = CNTPF - ES*CNT_ICG + off; 
  FLEXPWM2_SM0VAL3 = CNTPF - ES*CNT_ICG + off + CNT_SH; // pulse SH high for CNT_SH counts
  FLEXPWM2_SM0VAL4 = 0;
  FLEXPWM2_SM0VAL5 = 0;

  // 0000 0100 0000 0000
  FLEXPWM2_OUTEN |= 256; // set output enable for A, module 0

}

void setup_MCLK() {
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 2; // set pin 6 to be PWM2 A SM 02
  
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM2CTRL2 |= 128; // set force enable

  // I'll set prescaler to 64 such that the min frequency is 150e6/2^16/16 = 35.8Hz
  // CTRL = 0000 0100 0110 0000
  FLEXPWM2_SM2CTRL |= 2048; // reload on half count
  
  FLEXPWM2_SM2OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  FLEXPWM2_SM2DTCNT0 = 0;
  FLEXPWM2_SM2INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM2VAL0 = 0; // set VAL0 to be halfway through counter??
  FLEXPWM2_SM2VAL1 = CLKPMCLK - 1; // each clock tick is 2/3us with 16x prescaler
  FLEXPWM2_SM2VAL2 = 0; // for now, no phase shift
  FLEXPWM2_SM2VAL3 = CLKPMCLK/2; // pulse ICG low for CNT_ICG counts
  FLEXPWM2_SM2VAL4 = 0;
  FLEXPWM2_SM2VAL5 = 0;

  // 0000 0100 0000 0000
  FLEXPWM2_OUTEN |= 1024; // set output enable for A, module 2 

}

void setup_ADCCLK() {
  // don't need to set up output pin; just using this for interrupt
  // use flexpwm2 A sm 3
  
  FLEXPWM2_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM3CTRL2 |= 128; // set force enable

  // I'll set prescaler to 64 such that the min frequency is 150e6/2^16/16 = 35.8Hz
  // CTRL = 0000 0100 0110 0000
  FLEXPWM2_SM3CTRL |= 2048; // reload on half count
  
  FLEXPWM2_SM3OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  FLEXPWM2_SM3DTCNT0 = 0;
  FLEXPWM2_SM3INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM3VAL0 = 0; // set VAL0 to be halfway through counter??
  FLEXPWM2_SM3VAL1 = 8*CLKPMCLK - 1; // each clock tick is 2/3us with 16x prescaler
  FLEXPWM2_SM3VAL2 = adc_off; 
  FLEXPWM2_SM3VAL3 = adc_off + 4*CLKPMCLK; // set up an edge ever 4 MCLK cycles
  FLEXPWM2_SM3VAL4 = 0; // for now, no phase shift
  FLEXPWM2_SM3VAL5 = 0; // remember B uses 4 and 5

  // don't need to set output; just using this for interrupt

  // set up interrupt
  FLEXPWM2_SM3STS |= 0<<3; // set val3 compare flag to 0
  FLEXPWM2_SM3STS |= 0<<2; // set val2 compare flag to 0
  FLEXPWM2_SM3INTEN = 1<<3; // enable interrupts for val3 compare
  FLEXPWM2_SM3INTEN |= 1<<2; // enable interrupts for val2 compare

  attachInterruptVector(IRQ_FLEXPWM2_3, flexpwm2_3_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_3);

}

int s = 1;

void flexpwm2_3_isr() {
  FLEXPWM2_SM3STS = 12;  // clear all set bits
  // do a fast read of adc here!
  digitalWriteFast(7,s); // toggle pin for scoping
  s = !s;
  count += 1;
  while (FLEXPWM2_SM3STS == 12); // wait for clear
}

void setup() {
  pinMode(7, OUTPUT);
  
  Serial.begin(115200);

  Serial.println(USPF);
  Serial.println(CLKPF);
  Serial.println(CNTPF);
  Serial.println(CNT_ICG);
  Serial.println(CNT_SH);
  Serial.println(off);

  // set up clocks  
  CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON) | CCM_CCGR4_PWM2(CCM_CCGR_ON) | CCM_CCGR4_PWM3(CCM_CCGR_ON) | CCM_CCGR4_PWM4(CCM_CCGR_ON);
  
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
  
  

}

int count_prev;

void loop() {
  // put your main code here, to run repeatedly:
  if (count != count_prev) Serial.println(count);
  count_prev = count;

}
