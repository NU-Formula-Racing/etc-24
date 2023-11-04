#include <esp32Encoder.h>
#include <iostream>
#include <Arduino.h>

// PIN NUMBERS

// Encoder 1 - 5120 PPR
#define E1_CHANNEL_A 26
#define E1_CHANNEL_B 25
#define E1_REV_PULSE 23
#define PPR_A 5120 // PPR for encoder 1

// Encoder 2 - 2560 PPR
#define E2_CHANNEL_A 33
#define E2_CHANNEL_B 32
#define E2_REV_PULSE 22
#define PPR_B 2560 // PPR for encoder 2

#define THROTTLE_TRAVEL 180 // max throttle travel angle from 0-360

// degrees / tick
// TICKS = (PPR / 360) * 30 -> ticksA in 30 degrees
// range -> (0, TICKS)
// % of travel = throttle_count / TICKS -> (0, 100)

ESP32Encoder encoder1;
ESP32Encoder encoder2;
uint16_t ticksA;
uint16_t ticksB;
uint8_t percent_throttleA;
int8_t percent_throttleB;
int16_t percentThrottle(uint16_t count, uint16_t ticksA);

void setup() {
  encoder1.attachSingleEdge(E1_CHANNEL_A, E1_CHANNEL_B);
  encoder2.attachSingleEdge(E2_CHANNEL_A, E2_CHANNEL_B);

  Serial.begin(9600);
}

void loop() {
  // Encoder 1
  if (encoder1.getCount() < 0) {
    encoder1.clearCount();
  }

  ticksA = (uint16_t) ((PPR_A / 360) * THROTTLE_TRAVEL); // ticksA per throttle travel
  percent_throttleA = percentThrottle(encoder1.getCount(), ticksA); // gets the percent of throttle traveled

  if (percent_throttleA > 100) {
    encoder1.setCount(ticksA);
    percent_throttleA = percentThrottle(encoder1.getCount(), ticksA);
  }

  // ENCODER 2

  if (encoder2.getCount() > 0) {
    encoder2.clearCount();
  }

  ticksB = (uint16_t) ((PPR_B / 360) * THROTTLE_TRAVEL);
  percent_throttleB = percentThrottle(encoder2.getCount(), ticksB);

if (percent_throttleB < -100) {
  encoder2.setCount(-ticksB);
  percent_throttleB = percentThrottle(encoder2.getCount(), ticksB);
}
  Serial.printf("count: %d, ticks: %d, percent: %d\n", encoder2.getCount() * 100 / ticksB, ticksB, percent_throttleB);
  // Encoder 2
  // if (encoder2.getCount() < 0) {
  //   encoder2.clearCount();
  // }

  // ticksB = (uint16_t) ((PPR_B / 360) * THROTTLE_TRAVEL);
  // // Serial.printf("ticks: %d\n", ticksB);
  // percent_throttleB = percentThrottle(encoder2.getCount(), ticksB);

  // if (percent_throttleB > 100) {
  //   encoder2.setCount(ticksB);
  //   percent_throttleB = percentThrottle(encoder2.getCount(), ticksB);
  // }
  
  // Serial.printf("throttle1: %d, throttle2: %d\n", percent_throttleA, percent_throttleB);
}

int16_t percentThrottle(uint16_t count, uint16_t ticks) {
  return (count * 100) / ticks;
}

// TODO

// Check that both sensors are within correct range of each other. 
// Check if brake and accel. are pressed at same time?

// DONE
// Check if throttle is pressed as the car turns on
// Write function to set new lowest point as 0.
