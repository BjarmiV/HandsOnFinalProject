#include "thingProperties.h"
#include "Timer5.h"
#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>

// Pin assignments
const int DAC_PIN = A0;        // Digital-to-Analog Converter pin
const int ADC_PIN = A1;        // Analog-to-Digital Converter pin
const int PWM_PIN = A3;        // PWM output pin
const int RED_LED = 0;         // Red LED pin
const int GREEN_LED = 1;       // Green LED pin
const int YELLOW_LED = 3;      // Yellow LED pin
const int DEBUG_IRQ_PIN = 2;   // Debug interrupt pin

// LCD pins
const int DB7 = 4;
const int DB6 = 5;
const int DB5 = 6;
const int DB4 = 8;
const int RS = 9;
const int E = 10;

// LCD initialization
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// Parameters
bool changer = LOW;
const int numSamples = 100; // Number of samples per sine wave cycle
const int targetFrequency = 50; // Target frequency in Hz
const int sampleRate = 10000; // Sample rate in Hz
float sampleRateReal = 10920.0; // Actual sample rate considering digital writes in the interrupt handler
volatile int sampleIndex = 0; // Index for current sample in the sine wave array
int sineWave[numSamples]; // Array to store sine wave samples
volatile int tickcounter; // Counter for ticks
volatile float readval = 0.0; // Value read from ADC

// Parameters for zero-crossing and interpolation
volatile int counter = 0;
volatile int frequencyCount = 0;
float prev_val = 0.0;
volatile unsigned long lastCrossingTime = 0;
float currentCrossingTime = 0.0;
float frequency11 = 0.0;
float frequency2[10];
float frequencySum = 0.0;
float freqtemp = 0.0;

// Parameters for moving average calculation
const int movingAverageSize = 10; // Size of the moving average window
float frequencyBuffer[movingAverageSize]; // Buffer for storing frequency values
int frequencyBufferIndex = 0; // Index for the moving average buffer
int frequencyBufferCount = 0; // Count of entries in the frequency buffer
float movingAverageFrequency = 0.0; // Moving average frequency

// Parameters for RMS voltage calculation
volatile float VREF = 1.0; // DAC voltage reference
volatile float VDAC = 0.0; // Voltage from DAC
volatile float VMAX = 3.3; // Maximum voltage
volatile unsigned long SoS = 0; // Sum of squares for RMS calculation
volatile float meanSquare = 0.0; // Mean square value
volatile float rmsVDAC = 0.0; // RMS Voltage from ADC
volatile int sampleCount = 0; // Sample count for RMS calculation
const float DCOFFSET = 0.9; // DC offset set on Wavegen (V)

// Parameters for midpoint calculation
float max_read = 0.0;
float min_read = 100.0;
float midpoint = 0.0;

// Parameters for lowpass filter implementation
volatile float filteredread = 0.0; // Filtered value (read from ADC)
float prev_filteredread = 0.0; // Previous filtered value
float samplePeriod = 1.0 / sampleRate; 
float lowpass_cutoff = 50.0; // Cutoff frequency in Hz
float RC = 1.0 / (2.0 * 3.14 * lowpass_cutoff); // Resistance and capacitance value for the filter
float alpha = samplePeriod / (RC + samplePeriod); // Alpha constant for lowpass filtering

// Parameters for droop control and PWM signaling
float kdroop1 = 0.6133; // Droop scaling for slope 1
float kdroop2 = 2.5; // Droop scaling for slope 2
float current = 0.0; // Scaled current for 240 V
float current_01 = 0.1305; // Base current for slope 1 (negative sign removed)
float current_02 = 160.0; // Base current for slope 2 (negative sign removed)
float voltage_scaled = 0.0; // Scaled voltage for 240 V
float dutycycle = 0.0; // PWM duty cycle
bool manual = false; // Manual control flag

// PWM signal parameters
volatile unsigned long wPer = 255; // Period of the PWM signal
volatile float pWMDC = 0.5; // Duty cycle of the PWM signal
const unsigned char gClock = 4; // Clock source for the PWM generator
const unsigned char dFactor = 3; // Divider factor for the PWM generator

void setup() {
  analogWriteResolution(10); // Set analog write resolution to 10 bits

  // Initialize Timer5 with the sample rate and attach interrupt
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);

  // Pin mode setup
  pinMode(DEBUG_IRQ_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DEBUG_IRQ_PIN, HIGH);

  // Initialize LCD
  lcd.begin(16, 2);

  // Initialize Serial communication
  Serial.begin(9600);
  delay(1500); // Delay to stabilize the serial communication

  // Initialize frequency buffer for moving average
  for (int i = 0; i < movingAverageSize; i++) {
    frequencyBuffer[i] = 0.0;
  }

  // Setup cloud properties
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  // Set debug message level and print debug info
  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update(); // Update cloud properties

  // Update max and min readings for midpoint calculation
  if (filteredread > max_read) {
    max_read = filteredread;
  }
  if (filteredread < min_read) {
    min_read = filteredread;
  }
  midpoint = ((max_read - min_read) / 2);

  // RMS voltage calculation
  if (tickcounter % 10 == 0) {
    sampleCount++;
    SoS += ((readval - midpoint) * (readval - midpoint));
  }
  
  // Frequency calculation with moving average
  if (counter >= 100) {
    frequencyBuffer[frequencyBufferIndex] = (counter / (tickcounter / sampleRateReal));
    frequencyBufferIndex = (frequencyBufferIndex + 1) % movingAverageSize;
    if (frequencyBufferCount < movingAverageSize) {
      frequencyBufferCount++;
    }

    // Moving average calculation
    float movingSum = 0.0;
    for (int i = 0; i < frequencyBufferCount; i++) {
      movingSum += frequencyBuffer[i];
    }
    movingAverageFrequency = movingSum / frequencyBufferCount;
    frequency11 = movingAverageFrequency;

    // Frequency adjustment for display
    if (frequency11 >= 49.95 && frequency11 < 50.05) {
      frequency11 = 50.0;
    } else if (frequency11 >= 49.85 && frequency11 < 49.95) {
      frequency11 = 49.9;
    } else if (frequency11 >= 50.05 && frequency11 <= 50.15) {
      frequency11 = 50.1;
    }

    // RMS voltage (AC)
    meanSquare = (float)SoS / sampleCount;
    rmsVDAC = sqrt(meanSquare);
    VDAC = (rmsVDAC / 1023.0) * VREF * (VMAX / VREF);

    // Resetting various counters and sums
    tickcounter = 0;
    counter = 0;
    SoS = 0;
    sampleCount = 0;
  }

  // Duty cycle calculation for droop control
  if (tickcounter >= 300) {
    manual = mANUAL;
    if (!manual) {
      current = (frequency11 - 49.88) / 0.0027;
      cURRENT = current;
    } else {
      current = cURRENT;
    }
    if (current < 52 && current >= 6) {
      dutycycle = (current + current_01) / kdroop1;
    } else if (current > 52.5 && current < 80) {
      dutycycle = (current + current_02) / kdroop2;
    } else if (current < 52.5 && current > 50) {
      dutycycle = 85.0;
    } else if (current < 6) {
      dutycycle = 0.0;
    } else if (current >= 80) {
      dutycycle = 97.0;
    }
    
    // Setting duty cycle in PWM
    REG_TCC0_CCB3 = 2400 * (dutycycle / 100.0);
    while (TCC0->SYNCBUSY.bit.CCB3);
  }

  // Updating LEDs based on frequency
  if (tickcounter >= 350) {
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

  // Updating LCD with frequency and RMS voltage
  LCD_update(frequency11, VDAC);

  // Updating values for cloud
  dUTYCYCLE = dutycycle;
  vDAC = VDAC; 
  frequency1 = frequency11;
}

// Function to update the LCD display with frequency and voltage
void LCD_update(float freq, float volt){
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

// Interrupt service routine (ISR) for Timer5
void TickTock() {
  tickcounter++;
  
  readval = analogRead(ADC_PIN); // Read value from ADC
  filteredread = alpha * readval + (1 - alpha) * filteredread; // Apply lowpass filter to the read value
  
  // RMS calculation
  if (tickcounter % 10 == 0){
    sampleCount++;
    SoS += ((readval - midpoint) * (readval - midpoint)); // Sum of squares
  }
  
  // Zero-crossing detection
  if ((prev_val < midpoint) && (filteredread > midpoint)){
     counter++;
  }
  
  prev_val = filteredread; // Update previous value
}

// Boosts ADC read speed by adjusting settings
void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1); // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16
                    ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00; // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1); // Wait for synchronization
}

// Sets up PWM with default 50% duty cycle
void PWM_setup(){
  // Set the PWM pin as an output
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(10) | // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4); // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN | // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M | // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4); // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

  // Enable the port multiplexer for the digital pin D7 
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1, and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 | // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization

  // Dual slope PWM operation: timers continuously count up to PER register value then down to 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) | // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH; // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE); // Wait for synchronization

  REG_TCC0_CCB3 = 1200; // Set duty cycle value
  while (TCC0->SYNCBUSY.bit.CCB3);
  
  // Set the frequency of the PWM on TCC0 to 250kHz
  REG_TCC0_PER = 2400;
  while (TCC0->SYNCBUSY.bit.PER); // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
  REG_TCC0_CC3 = 48; // TCC0 CC3 - on D7
  while (TCC0->SYNCBUSY.bit.CC3); // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE; // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE); // Wait for sync
}

/*
  Callback functions to handle changes in cloud variables.
  These functions are called whenever a new value is received from the IoT Cloud.
*/

// Handle changes to the current variable
void onCURRENTChange() {
  // Add your code here to act upon Current change
}

// Handle changes to the duty cycle variable
void onDUTYCYCLEChange() {
  // Add your code here to act upon Dutycycle change
}

// Handle changes to the frequency variable
void onFrequency1Change() {
  // Add your code here to act upon Frequency1 change
}

// Handle changes to the VDAC variable
void onVDACChange() {
  // Add your code here to act upon VDAC change
}

// Handle changes to the manual control variable
void onMANUALChange() {
  // Add your code here to act upon MANUAL change
}
