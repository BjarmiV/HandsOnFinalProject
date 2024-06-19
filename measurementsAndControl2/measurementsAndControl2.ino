/* 
  Sketch generated by the Arduino IoT Cloud Thing "Untitled"
  https://create.arduino.cc/cloud/things/8afda3c4-a957-4f49-aa3a-df0224b7b66e 

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  float frequency1;
  float vDAC;
  bool gREEN_LED;
  bool rED_LED;
  bool yELLOW_LED;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

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
float frequency2[60];
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
  // Initialize DAC
  analogWriteResolution(10); // Set DAC resolution to 10 bits

  // Initialize Timer5
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);
  pinMode(DEBUG_IRQ_PIN, OUTPUT);  //Use for debugging interrupt handler.
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DEBUG_IRQ_PIN, HIGH);
  pinMode(3, OUTPUT);

  AdcBooster();
  PWM_setup();
  lcd.begin(16,2);

  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h

  // Connect to Arduino IoT Cloud
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
}

void loop() {

  
  // Your code here 
  
 


    if (counter >= 400) {
    // Frequency calculation
    frequencyCount++;
    if (frequencyCount <= 40) {
        frequency2[frequencyCount-1] = (counter / ((tickcounter / sampleRateReal)));
    }
    frequencySum = 0.0;
    for (int i = 0; i < 40; i++) {
        frequencySum += frequency2[i];
    }
    frequency11 = frequencySum;
    frequencyCount = 0;


    if ((frequency11 >= 49.95) && (frequency11 < 50.05)){
      frequency11 = 50.0;
    }
    else if ((frequency11 >= 49.85) && (frequency11 < 49.95)){
      frequency11 = 49.9;
    }
    else if ((frequency11 >= 50.05) && (frequency11 <= 50.15)){
      frequency11 = 50.1;
    } 
    else {
      frequency11 = frequency11;
    }

    // RMS Voltage calculation
    meanSquare = (float)SoS / sampleCount;
    rmsVADC = sqrt(meanSquare);
    VDAC = (rmsVADC / 1023.0) * VREF * (VMAX / VREF);

    // Resetting counters and sums
    tickcounter = 0;
    counter = 0;
    SoS = 0;
    sampleCount = 0;
}

    
    

  if (tickcounter >= 300){
    //calculating PWM duty cycle and current.
    current = (frequency11 - 49.88) / 0.0027;

    //current switch statement which deterines dutycycle
    if((current < 52) && (current > 6)){
      dutycycle = (current - current_01) / kdroop1;
    }
    else if((current > 52.5) && (current < 80)){
      dutycycle = (current - current_02) / kdroop2;
    }
    else if((current < 52.5) && (current > 50)){
      dutycycle = 85.0;
    }
    else if(current < 6){
      dutycycle = 0.0;
    }
    else if (current >= 80){
      dutycycle = 97.0;
    }
    //swtting up the new dutycycle. 2400 is from the period, PER Reg. (counter)
    REG_TCC0_CCB3 = 2400 * (dutycycle / 100.0);
    while (TCC0->SYNCBUSY.bit.CCB3);
  }

  // updating LEDs based on frequency  
  if (tickcounter >= 250){
    if ((frequency11 >= 49.9) && (frequency11 <= 50.1)){
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
  
    }
    else if (frequency11 < 49.9){
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
 
    }
    else if (frequency11 > 50.1){
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      
    }
  }

  //LCD update
  LCD_update(frequency11, VDAC);

     /*Serial.print("  ");
     Serial.print(frequencySum);
     Serial.print("  ");*/
     Serial.println(VDAC, 3);
     /*Serial.println(frequency11, 4);
     if (frequency11 < 49.975){
      digitalWrite(1, LOW);
     }
     else if(frequency11 >50.025){
      digitalWrite(1, HIGH);
     }*/
     




  
}





void LCD_update(float freq, float volt){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Freq: ");
  lcd.print((float)(((int)(freq * 100.0)) / 100.0));
  lcd.print(" Hz");
  lcd.setCursor(0,1);
  lcd.print("V_RMS: ");
  lcd.print(volt);
  lcd.print(" V");
}

void TickTock() {
  //digitalWrite(DEBUG_IRQ_PIN, HIGH); // use with lowest write. To debug if Handler is doing too much.
  tickcounter++;
  
  readval = analogRead(ADC_PIN); //
  filteredread = alpha*readval + (1-alpha)*filteredread; //from arduino
  if (tickcounter % 10 == 0){
    sampleCount++;
    SoS += ((readval-midpoint) * (readval-midpoint)); //sum of squares (inside square) here for unfiltered (just to compare with oscilloscope).
  }
  
  if (sampleCount > 1000){
    if (filteredread > max_read){
      max_read = filteredread;
    }
    if (filteredread < min_read){
      min_read = filteredread;
    }
    midpoint = ((max_read - min_read) / 2);
  }
  if((prev_val < midpoint) && (filteredread > midpoint)){
     counter++;
  }/*
  else if((prev_val > midpoint) && (filteredread < midpoint)){
     counter++;
  }*/
  
  prev_val = filteredread; 
  //analogWrite(DAC_PIN, filteredread);

  //digitalWrite(DEBUG_IRQ_PIN, LOW);  // use with most above in Handler. For debugging.
}

//Boosts ADC read speed.
void AdcBooster()
{
 ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
 ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16.
 ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
 ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
 ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
 ADC->SAMPCTRL.reg = 0x00; // Sampling Time Length = 0
 ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
} // AdcBooster

//This function sets up PWM with default 50% dutycycle. NOTE TCC0 is used and so analogWrite doesn't work. 
void PWM_setup(){
    // Set the PWM pin as an output
 REG_GCLK_GENDIV = GCLK_GENDIV_DIV(10) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D7 
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  REG_TCC0_CCB3 = 1200;
  while (TCC0->SYNCBUSY.bit.CCB3);
  
  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = 2400;         // Set the frequency of the PWM on TCC0 to 250kHz
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
  REG_TCC0_CC3 = 48;         // TCC0 CC3 - on D7
  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for sync
}