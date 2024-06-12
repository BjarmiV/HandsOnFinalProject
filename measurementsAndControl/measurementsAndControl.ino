#include <Timer5.h>
#include <Arduino.h>
#include <math.h>

bool changer = LOW;
const int DAC_PIN = A0;
const int ADC_PIN = A1;
const int numSamples = 100; // Number of samples per sine wave cycle
const int targetFrequency = 50; // Target frequency in Hz
const int sampleRate = 10000; // Sample rate in Hz
float sampleRateReal = 10920.0;  //See digital writes in interrupt handler. 
volatile int sampleIndex = 0; // Index for current sample
int sineWave[numSamples]; // Array to store sine wave samples
volatile int tickcounter;
volatile float readval = 0.0; // value read from ADC

//param for zero-crossing
volatile int counter = 0;
float prev_val = 0.0;
unsigned long time_after_zero_crossing = 0;
unsigned long last_time_crossing = 0;
volatile unsigned long lastCrossingTime = 0;
volatile unsigned long currentCrossingTime = 0;
float frequency1 = 0.0;
unsigned long frequency2 = 0.0;

// random
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
//float alpha = 0.03;  //alpha constant for lowpass filtering


void setup() {

  // Initialize DAC
  analogWriteResolution(10); // Set DAC resolution to 10 bits

  // Initialize Timer5
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);
  //pinMode(2, OUTPUT);  //Use for debugging interrupt handler.
  //digitalWrite(2, HIGH);

  AdcBooster();

  Serial.begin(9600);
  
}

void loop() {

     if (counter >= 1000){
    frequency1 = (counter / (tickcounter / sampleRateReal) / 2.0);
    tickcounter = 0;
    counter = 0;
    
    
  }

     //Serial.println(frequency1);
     
 

}

void TickTock() {
  //digitalWrite(2, HIGH); // use with lowest write. To debug if Handler is doing too much.
  tickcounter++;
  
  readval = analogRead(ADC_PIN); //from signal generator
  filteredread = alpha*readval + (1-alpha)*filteredread; //from arduino


  if (filteredread > max_read){
    max_read = filteredread;
  }
  if (filteredread < min_read){
    min_read = filteredread;
  }
  midpoint = (max_read - min_read) / 2;
 
  if((prev_val < midpoint) && (filteredread > midpoint)){
     counter++;
  }
  else if((prev_val > midpoint) && (filteredread < midpoint)){
     counter++;
  }
  
    prev_val = filteredread;    
  
  analogWrite(DAC_PIN, filteredread);

  //digitalWrite(2, LOW);  // use with most above in Handler. For debugging.
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
