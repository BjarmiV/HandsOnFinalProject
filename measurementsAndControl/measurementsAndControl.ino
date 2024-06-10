#include <Timer5.h>
#include <Arduino.h>
#include <math.h>

const int DAC_PIN = A0;
const int numSamples = 100; // Number of samples per sine wave cycle
const int targetFrequency = 50; // Target frequency in Hz
const int sampleRate = targetFrequency * numSamples; // Sample rate in Hz
volatile int sampleIndex = 0; // Index for current sample
int sineWave[numSamples]; // Array to store sine wave samples

void setup() {
  
  // Compute sine wave samples for 10-bit DAC
  for (int i = 0; i < numSamples; i++) {
    sineWave[i] = (int)(511 + 511 * sin(2 * PI * i / numSamples)); // Generate samples from 0 to 1023
  }

  // Initialize DAC pin
  analogWriteResolution(10); // Set DAC resolution to 10 bits
  analogWrite(DAC_PIN, sineWave[0]); // Start with the first sample

  // Initialize Timer5
  MyTimer5.begin(sampleRate);
  MyTimer5.attachInterrupt(TickTock);
}

void loop() {
  
}

void TickTock() {
  analogWrite(DAC_PIN, sineWave[sampleIndex]); // Output the next sample
  sampleIndex = (sampleIndex + 1) % numSamples; // Increment and wrap the sample index and loops from 0-100
}
