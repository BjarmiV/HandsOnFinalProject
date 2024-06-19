
#include "Timer5.h"
#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>

//Pin assignments
const int DAC_PIN = A0;
const int ADC_PIN = A1;
const int PWM_PIN = A3;
const int RED_LED = 0;
const int GREEN_LED = 1;
const int YELLOW_LED = 3;
const int DEBUG_IRQ_PIN = 2;
//LCD pins
const int DB7 = 4;
const int DB6 = 5;
const int DB5 = 6;
const int DB4 = 8;
const int RS = 9;
const int E = 10;

//LCD initialization
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

//param for 
bool changer = LOW;
const int numSamples = 100; // Number of samples per sine wave cycle
const int targetFrequency = 50; // Target frequency in Hz
const int sampleRate = 10000; // Sample rate in Hz
float sampleRateReal = 10920.0;  //See digital writes in interrupt handler. 
volatile int sampleIndex = 0; // Index for current sample
int sineWave[numSamples]; // Array to store sine wave samples
volatile int tickcounter;
volatile float readval = 0.0; // value read from ADC

//param for zero-crossing and interpolation
volatile int counter = 0;
volatile int frequencyCount = 0;
float prev_val = 0.0;
volatile unsigned long lastCrossingTime = 0;
float currentCrossingTime = 0.0;
float frequency11 = 0.0;
float frequency2[20];
float frequencySum = 0.0;

//param for RMS voltage
volatile float VREF = 1.0; // DAC voltage reference
volatile float VDAC = 0.0; //Voltage from DAC
volatile float VMAX = 3.3; //max voltage
volatile unsigned long SoS = 0; // sum of squares
volatile float meanSquare = 0.0; // mean square product
volatile float rmsVADC = 0.0;  // RMS Vol tage from ADC
volatile int sampleCount = 0; //Counts samples taken for RMS
const float DCOFFSET = 0.9; //DC offset set on Wavegen (V)

//param for midpoint calculation
float max_read = 0.0;
float min_read = 100.0;
float midpoint = 0.0;

//param for lowpass implementation
volatile float filteredread = 0.0; // filtered value (read from ADC)
float prev_filteredread = 0.0; //previous filtered value
float samplePeriod = 1.0/sampleRate; 
float lowpass_cutoff = 50.0; // cutoff frequency in Hz
float RC = 1.0 / (2.0 * 3.14 * lowpass_cutoff); //Resistance and capacitance val
float alpha = samplePeriod / (RC + samplePeriod);  //alpha constant for lowpass filtering

//param for droopcontrol and pwm signalling
float kdroop1 = 0.6133; //droop scaling for slope 1
float kdroop2 = 2.5; //droop scaling for slope 2
float current = 0.0; //scaled for 240 V
float current_01 = 0.6133; //base current for slope 1 (negative sign removed)
float current_02 = 160.0; //base current for slope 2 (negative sign removed)
float voltage_scaled = 0.0; //scaled for 240 V
float dutycycle = 0.0; //PWM duty cycle


//sets the period of the PWM signal, PWM period = wPer / gen clock rate 
volatile unsigned long wPer = 255;
//This variable is to generate the duty cycle of the PWM signal 0.5 --> 50%
volatile float pWMDC = 0.5;
//selects the gen clock for setting the waveform generator clock or sample rate
const unsigned char gClock = 4;
//sets the divide factor for the gen clk, 48MHz / 3 = 16MHz
const unsigned char dFactor = 3;

void setup() {
  analogWriteResolution(10);

  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);
  pinMode(DEBUG_IRQ_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DEBUG_IRQ_PIN, HIGH);
  pinMode(3, OUTPUT);

  AdcBooster();
  PWM_setup();
  lcd.begin(16, 2);

  Serial.begin(9600);
  delay(1500);
}

void loop() {
  if (counter >= 200) {
    frequencyCount++;
    if (frequencyCount <= 20) {
      frequency2[frequencyCount - 1] = (counter / ((tickcounter / sampleRateReal)));
    }

    frequencySum = 0.0;
    for (int i = 0; i < 20; i++) {
      frequencySum += frequency2[i];
    }
    frequency11 = frequencySum;
    frequencyCount = 0;

    if (frequency11 >= 49.95 && frequency11 < 50.05) {
      frequency11 = 50.0;
    } else if (frequency11 >= 49.85 && frequency11 < 49.95) {
      frequency11 = 49.9;
    } else if (frequency11 >= 50.05 && frequency11 <= 50.15) {
      frequency11 = 50.1;
    }

    float meanSquare = (float)SoS / sampleCount;
    rmsVADC = sqrt(meanSquare);
    VDAC = (rmsVADC / 1023.0) * VREF * (VMAX / VREF);

    tickcounter = 0;
    counter = 0;
    SoS = 0;
    sampleCount = 0;
  }

  if (tickcounter >= 300) {
    current = (frequency11 - 49.88) / 0.0027;

    if (current < 52 && current > 6) {
      dutycycle = (current - current_01) / kdroop1;
    } else if (current > 52.5 && current < 80) {
      dutycycle = (current - current_02) / kdroop2;
    } else if (current < 52.5 && current > 50) {
      dutycycle = 85.0;
    } else if (current < 6) {
      dutycycle = 0.0;
    } else if (current >= 80) {
      dutycycle = 97.0;
    }

    REG_TCC0_CCB3 = 2400 * (dutycycle / 100.0);
    while (TCC0->SYNCBUSY.bit.CCB3);
  }

  if (tickcounter >= 250) {
    if (frequency11 >= 49.9 && frequency11 <= 50.1) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
    } else if (frequency11 < 49.9) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
    } else if (frequency11 > 50.1) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
    }
  }

  LCD_update(frequency11, VDAC);
  Serial.println(VDAC, 3);
}

void LCD_update(float freq, float volt) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.print((float)(((int)(freq * 100.0)) / 100.0));
  lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print("V_RMS: ");
  lcd.print(volt);
  lcd.print(" V");
}

void TickTock() {
  tickcounter++;

  readval = analogRead(ADC_PIN);
  filteredread = alpha * readval + (1 - alpha) * filteredread;

  if (tickcounter % 10 == 0) {
    sampleCount++;
    SoS += ((readval - midpoint) * (readval - midpoint));
  }

  if (sampleCount > 1000) {
    if (filteredread > max_read) {
      max_read = filteredread;
    }
    if (filteredread < min_read) {
      min_read = filteredread;
    }
    midpoint = ((max_read - min_read) / 2);
  }

  if (prev_val < midpoint && filteredread > midpoint) {
    counter++;
  }

  prev_val = filteredread;
}

void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_RESSEL_10BIT;
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);
  ADC->SAMPCTRL.reg = 0x00;
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void PWM_setup() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(10) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY);

  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) | TCC_WAVE_WAVEGEN_DSBOTH;
  while (TCC0->SYNCBUSY.bit.WAVE);

  REG_TCC0_CCB3 = 1200;
  while (TCC0->SYNCBUSY.bit.CCB3);

  REG_TCC0_PER = 2400;
  while (TCC0->SYNCBUSY.bit.PER);

  REG_TCC0_CC3 = 48;
  while (TCC0->SYNCBUSY.bit.CC3);

  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);
}
