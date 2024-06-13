#include <Timer5.h>
#include <Arduino.h>
#include <math.h>

bool changer = LOW;
const int DAC_PIN = A0;
const int ADC_PIN = A1;
const int PWM_PIN = 9;
const unsigned long DCOFFSET = 0.9;
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
float prev_val = 0.0;
volatile unsigned long lastCrossingTime = 0;
float currentCrossingTime = 0.0;
float frequency1 = 0.0;
unsigned long frequency2 = 0.0;

//param for RMS voltage
volatile float VREF = 1.0; // DAC voltage reference
volatile float VDAC = 0.0; //Voltage from DAC
volatile float VMAX = 3.3; //max voltage
volatile unsigned long SoS = 0; // sum of squares
volatile float meanSquare = 0.0; // mean square product
volatile float rmsVADC = 0.0;  // RMS Voltage from ADC
volatile int sampleCount = 0;

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

void setup() {

  // Initialize DAC
  analogWriteResolution(10); // Set DAC resolution to 10 bits

  // Initialize Timer5
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);
  pinMode(2, OUTPUT);  //Use for debugging interrupt handler.
  pinMode(1, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(1, HIGH);
  AdcBooster();

  Serial.begin(9600);
  
}

void loop() {


    
     if (counter >= 400){
    // Frequency calculation
    frequency1 = (counter / (tickcounter / sampleRateReal) / 2.0);
    //RMS Voltage calculation
    meanSquare = (float)SoS / sampleCount;
    rmsVADC = sqrt(meanSquare);
    VDAC = (rmsVADC / 1023.0) * VREF * (VMAX / VREF); 
    //Resetting counters and sums
    tickcounter = 0;
    counter = 0;
    SoS = 0;
    sampleCount = 0;
    
    
  }
  if (tickcounter >= 1000){
    //calculating PWM duty cycle and current - every 100 ms. 
    current = (frequency1 - 49.88) / 0.0027;
    
    if(current < 52){
      dutycycle = (current - current_01) / kdroop1;
    }
    else if(current > 52.5){
      dutycycle = (current - current_02) / kdroop2;
    }
    else if((current < 52.5) && (current > 50)){
      dutycycle = 85.0;
    }
    else if(current < 6){
      dutycycle = 0.0;
    }
  }

  analogWrite(PWM_PIN, 127);

     /*Serial.print(rmsVoltageADC);
     Serial.print("  ");
     Serial.print(tickcounter);
     Serial.print("  ");*/
     Serial.println(dutycycle, 3);
     /*Serial.println(frequency1, 4);
     if (frequency1 < 49.975){
      digitalWrite(1, LOW);
     }
     else if(frequency1 >50.025){
      digitalWrite(1, HIGH);
     }*/
     
    
}

void TickTock() {
  digitalWrite(2, HIGH); // use with lowest write. To debug if Handler is doing too much.
  tickcounter++;
  
  readval = analogRead(ADC_PIN); //
  filteredread = alpha*readval + (1-alpha)*filteredread; //from arduino
  if (tickcounter % 10 == 0){
    sampleCount++;
    SoS += (unsigned long)((readval-DCOFFSET) * (readval-DCOFFSET)); //sum of squares (inside square) here for unfiltered (just to compare with oscilloscope).
  }
  


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

  digitalWrite(2, LOW);  // use with most above in Handler. For debugging.
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
