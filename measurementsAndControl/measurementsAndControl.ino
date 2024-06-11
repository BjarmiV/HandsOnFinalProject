#include <Timer5.h>
#include <Arduino.h>
#include <math.h>

const int DAC_PIN = A0;
const int ADC_PIN = A1;
const int numSamples = 100; // Number of samples per sine wave cycle
const int targetFrequency = 50; // Target frequency in Hz
const int sampleRate = 10000; // Sample rate in Hz
volatile int sampleIndex = 0; // Index for current sample
int sineWave[numSamples]; // Array to store sine wave samples
int tickcounter;
volatile float readval = 0.0; // value read from ADC

//param for lowpass implementation
volatile float filteredread = 0.0; // filtered value (read from ADC)
float prev_filteredread = 0.0; //previous filtered value
float samplePeriod = 1.0/sampleRate; 
float lowpass_cutoff = 50.0; // cutoff frequency in Hz
float RC = 1.0 / (2.0 * 3.14 * lowpass_cutoff); //Resistance and capacitance val
float alpha = samplePeriod / (RC + samplePeriod);  //alpha constant for lowpass filtering
//float alpha = 0.03;  //alpha constant for lowpass filtering


void setup() {
  /*const float pi2 = 2*3.14;
  
  // Compute sine wave samples for 10-bit DAC
  for (int i = 0; i < numSamples; i++) {
    sineWave[i] = (int)(511.5 + 511.5 * sin(2 * PI * i / numSamples)); // Generate samples from 0 to 1023
  }*/

  // Initialize DAC pin
  analogWriteResolution(10); // Set DAC resolution to 10 bits

  // Initialize Timer5
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);

  AdcBooster();

  Serial.begin(9600);
  
}

void loop() {

  Serial.println(readval);

}

void TickTock() {
  tickcounter++;
  
  readval = analogRead(ADC_PIN);
  filteredread = alpha*readval + (1-alpha)*filteredread;
  analogWrite(DAC_PIN, filteredread);
}

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
