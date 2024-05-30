#include <Arduino.h>
#include "teensy_can.h"
#include "VirtualTimer.h"
#include <SPI.h>

#define MIN_90D 1240
#define MAX_90D 1790
#define MIN_40D 356
#define MAX_40D 1352
#define SENSOR40_CS 10
#define SENSOR90_CS 5
#define BRAKE_PIN 19
#define BRAKE_VALID_PIN 6
#define MAX_THROTTLE 32767

// global sensor variables
int16_t sensor40;
int16_t sensor90;

// initialize CAN and timers
TeensyCAN<1> p_bus{};
TeensyCAN<2> g_bus{};
VirtualTimerGroup p_timer_group{};
VirtualTimerGroup g_timer_group{};

// priority CAN signals
MakeSignedCANSignal(int16_t, 0, 16, 1, 0) p_throttle_percent{};
MakeSignedCANSignal(bool, 16, 8, 1, 0) p_throttle_active{};
MakeSignedCANSignal(bool, 24, 8, 1, 0) p_brake_pedal{};

// general CAN signals
MakeSignedCANSignal(int16_t, 0, 16, 1, 0) g_throttle_percent{};
MakeSignedCANSignal(bool, 16, 8, 1, 0) g_throttle_active{};
MakeSignedCANSignal(bool, 24, 8, 1, 0) g_brake_pedal{};

// priority messages
CANTXMessage<3> throttle_tx_p{
  p_bus, 0x010, 4, 10, p_timer_group, p_throttle_percent, p_throttle_active, p_brake_pedal};

// general messages
CANTXMessage<3> throttle_tx_g{
  g_bus, 0x010, 4, 10, g_timer_group, g_throttle_percent, g_throttle_active, g_brake_pedal};

// raw voltage outputs from the sensors, not used in calculations, but useful for testing
float sensor_voltage_90D;
float sensor_voltage_40D;

// throttle percent variables for implausibility checking
float throttle_scaled_90D;
float throttle_scaled_40D;

// variables to keep track of elapsed time for implausibilities
unsigned long current_millis = 0;
unsigned long previous_millis = 0;
const long interval = 100;
bool counting = false;

// brake implausibility variables
bool counting_brake = false;
unsigned long previous_millis_brake = 0;
unsigned long interval_brake = 100;

// brake valid
bool brake_valid;

// brake implausibility global
bool brake_implausible_25;

// values to send over CAN
int16_t throttle_percent;
bool brake_pressed;
bool t_active;



void p_ten_ms_task() {
  p_throttle_percent = throttle_percent;
  p_throttle_active = t_active;
  p_brake_pedal = brake_pressed;
  p_bus.Tick();
}

void g_ten_ms_task() {
  g_throttle_percent = throttle_percent;
  g_throttle_active = t_active;
  g_brake_pedal = brake_pressed;
  g_bus.Tick();
}

// read raw values from ADCs over SPI, write to global variables
void adc_read() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));

  // read from 40d adc
  digitalWrite(SENSOR40_CS, LOW);
  int16_t adc40_raw = SPI.transfer16(0x0000);
  digitalWrite(SENSOR40_CS, HIGH);

  // read from 90d adc
  digitalWrite(SENSOR90_CS, LOW);
  int16_t adc90_raw = SPI.transfer16(0x0000);
  digitalWrite(SENSOR90_CS, HIGH);

  SPI.endTransaction();

  // select correct bits and write to global sensor variables
  sensor40 = int16_t(adc40_raw << 2) >> 4;
  sensor90 = int16_t(adc90_raw << 2) >> 4;
}

void setup() {

  // start serial monitor
  Serial.begin(115200);

  // initialize CAN busses
  p_bus.Initialize(ICAN::BaudRate::kBaud1M);
  p_timer_group.AddTimer(10, p_ten_ms_task);

  g_bus.Initialize(ICAN::BaudRate::kBaud1M);
  g_timer_group.AddTimer(10, g_ten_ms_task);

  // start SPI bus and set CS pins
  SPI.begin();
  pinMode(SENSOR40_CS, OUTPUT);
  pinMode(SENSOR90_CS, OUTPUT);
  digitalWrite(SENSOR40_CS, HIGH);
  digitalWrite(SENSOR90_CS, HIGH); // Set both CS pins to HIGH initially

  // set brake pin and brake valid pin as input
  pinMode(BRAKE_PIN, INPUT);
  pinMode(BRAKE_VALID_PIN, INPUT);

}

void loop() {

  // record current time
  current_millis = millis();

  // read from ADCs and set global variables
  adc_read();

  // convert sensor values to voltage -- not used in calculations
  sensor_voltage_40D = (sensor40*6.6)/4096;
  sensor_voltage_90D = (sensor90*6.6)/4096;

  // throttle percent scaled from 0 to 100 for implausibility checks
  throttle_scaled_40D = float(sensor40-MIN_40D)/(MAX_40D-MIN_40D)*100;//-MIN_40D)/(MAX_40D-MIN_40D);
  throttle_scaled_90D = float(sensor90-MIN_90D)/(MAX_90D-MIN_90D)*100;

  // set throttle percent value to send, scaled from 0 to 32767
  if (throttle_scaled_40D <= 0.0) {
    throttle_percent = 0;
  } else if (throttle_scaled_40D >= 100.0) {
    throttle_percent = MAX_THROTTLE;
  } else {
    throttle_percent = ((sensor40*MAX_THROTTLE)-MIN_40D*MAX_THROTTLE)/(MAX_40D-MIN_40D);
  }
  // set brake pressed value
  brake_pressed = digitalRead(BRAKE_PIN);
  brake_valid = digitalRead(BRAKE_VALID_PIN);

  // 10% rule
  // if difference in sensors is >10%, set throttle_active=0
  if (throttle_scaled_40D-throttle_scaled_90D > 10.0 || throttle_scaled_40D-throttle_scaled_90D < -10.0 || !brake_valid) {
    if (!counting) {
      previous_millis = current_millis; // start counting
      counting = true;
    } else {
      if (current_millis-previous_millis >= interval) {
        t_active = false;
      }
    }
    // if brake is pressed while throttle is >25% -> set t_active=0, wait until throttle is 5% before changing t_active back to 1
  } else if (brake_pressed && throttle_scaled_40D > 25.0) {
      if (!counting_brake) {
        previous_millis_brake = current_millis; // start counting
        counting_brake = true;
      } else {
        if (current_millis-previous_millis_brake >= interval_brake) {
          t_active = false;
          brake_implausible_25 = true;
        }
      }
      
  // if throttle is <5% -> set t_active=1
  } else if (brake_implausible_25 && throttle_scaled_40D < 5.0) {
      t_active = true;
      brake_implausible_25 = false;

  } else if (brake_implausible_25) {
      t_active = false;
  }
  // no implausibility!!!!!
  else {
    // if difference is <10%, set t_active=1
    counting = false;
    counting_brake = false;
    t_active = true;
  }

  // keep throttle active true for testing with VCU, THIS SHOULD NOT BE IN FINAL CODE
  //t_active = true;

  // tick CAN bus
  p_timer_group.Tick(millis());
  g_timer_group.Tick(millis());

  // print sensor readings and throttle stuff
  Serial.print("adc40: ");
  Serial.print(sensor40);
  Serial.printf("\tadc90: ");
  Serial.print(sensor90);
  Serial.printf("\tt40: ");
  Serial.print(throttle_scaled_40D);
  Serial.printf("\tt90: ");
  Serial.print(throttle_scaled_90D);
  Serial.printf("\tt_active: ");
  Serial.print(t_active);
  Serial.printf("\tdiff: ");
  Serial.print(throttle_scaled_40D-throttle_scaled_90D);
  Serial.printf("\tt_perc: ");
  Serial.print(throttle_percent);
  Serial.printf("\tbrake: ");
  Serial.print(brake_pressed);
  Serial.printf("\tb_val: ");
  Serial.println(brake_valid);
  // Serial.print(sensor90, BIN);
  // Serial.print("\t");
  // Serial.println(sensor90);
  delay(100);

}
