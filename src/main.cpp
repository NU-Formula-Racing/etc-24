#include <Arduino.h>
#include "teensy_can.h"
#include "VirtualTimer.h"
#include <SPI.h>

#define MIN_90D 107
#define MAX_90D 729
#define MIN_40D 300
#define MAX_40D 1180
#define SENSOR40_CS 10
#define SENSOR90_CS 5
#define MAX_THROTTLE 100

int16_t sensor40;
int16_t sensor40_shifted;
int16_t sensor90;
int16_t sensor90_shifted;

TeensyCAN<1> p_bus{};
TeensyCAN<2> g_bus{};
VirtualTimerGroup p_timer_group{};
VirtualTimerGroup g_timer_group{};

// priority CAN sigs
MakeSignedCANSignal(int16_t, 0, 16, 1, 0) p_throttle_percent{};
MakeSignedCANSignal(bool, 16, 8, 1, 0) p_throttle_active{};
MakeSignedCANSignal(bool, 0, 8, 1, 0) p_brake_pedal{};

// general CAN sigs
MakeSignedCANSignal(int16_t, 0, 16, 1, 0) g_throttle_percent{};
MakeSignedCANSignal(bool, 16, 8, 1, 0) g_throttle_active{};
MakeSignedCANSignal(bool, 0, 8, 1, 0) g_brake_pedal{};

// priority messages
CANTXMessage<2> throttle_tx_p{
  p_bus, 0x010, 3, 10, p_timer_group, p_throttle_percent, p_throttle_active};

CANTXMessage<1> brake_tx_p{
  p_bus, 0x011, 1, 10, p_timer_group, p_brake_pedal};

// general messages
CANTXMessage<2> throttle_tx_g{
  g_bus, 0x010, 3, 10, g_timer_group, g_throttle_percent, g_throttle_active};

CANTXMessage<1> brake_tx_g{
  g_bus, 0x011, 1, 10, g_timer_group, g_brake_pedal};

// utility global variables
float sensor_voltage_90D;
float sensor_voltage_40D;
int16_t throttle_percent_40D;
int16_t throttle_percent_90D;

unsigned long current_millis = 0;
unsigned long previous_millis = 0;
const long interval = 100;
bool counting = false;

// values to send over CAN
int16_t throttle_percent;
bool brake_pressed;
bool t_active;

// throttle percent variables for sensor testing
float throttle_scaled_90D;
float throttle_scaled_40D;

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



void setup() {
  pinMode(19, INPUT);
  pinMode(18, INPUT);
  Serial.begin(115200);

  p_bus.Initialize(ICAN::BaudRate::kBaud1M);
  p_timer_group.AddTimer(10, p_ten_ms_task);

  g_bus.Initialize(ICAN::BaudRate::kBaud1M);
  g_timer_group.AddTimer(10, g_ten_ms_task);

  SPI.begin();
  pinMode(SENSOR40_CS, OUTPUT);
  pinMode(SENSOR90_CS, OUTPUT);
  digitalWrite(SENSOR40_CS, HIGH);
  digitalWrite(SENSOR90_CS, HIGH); // Set both CS pins to HIGH initially


}

void loop() {
  t_active = true;
  current_millis = millis();

  brake_pressed = false;
  t_active = true;


  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));

  digitalWrite(SENSOR40_CS, LOW);
  sensor40 = SPI.transfer16(0x0000);
  digitalWrite(SENSOR40_CS, HIGH);

  digitalWrite(SENSOR90_CS, LOW);
  sensor90 = SPI.transfer16(0x0000);
  digitalWrite(SENSOR90_CS, HIGH);

  sensor40_shifted = (sensor40 >> 2) & 0xFFF;
  sensor90_shifted = (sensor90 >> 2) & 0xFFF;

  // convert sensor values to voltage -- not used in calculations
  sensor_voltage_40D = (sensor40_shifted*6.6)/4096;
  sensor_voltage_90D = (sensor90_shifted*6.6)/4096;

  // throttle percent scaled from 0 to 100 for implausibility checks
  throttle_scaled_40D = float(sensor40_shifted-MIN_40D)/(MAX_40D-MIN_40D)*100;//-MIN_40D)/(MAX_40D-MIN_40D);
  throttle_scaled_90D = float(sensor90_shifted-MIN_90D)/(MAX_90D-MIN_90D)*100;

  SPI.endTransaction();

  Serial.print("Sensor 40: ");
  Serial.print(sensor40_shifted);
  Serial.print(" Sensor 90: ");
  Serial.print(sensor90_shifted);
  Serial.print(" throttle_scaled_40D: ");
  Serial.print(throttle_scaled_40D);
  Serial.print(" Throttle Percent: ");
  Serial.println(throttle_percent);

  //Serial.println(throttle_percent);

  // set throttle percent value to send, scaled from 0 to 32767
  if (throttle_scaled_40D <= 0.0) {
    throttle_percent = 0;
  } else if (throttle_scaled_40D >= 100.0) {
    throttle_percent = MAX_THROTTLE;
  } else {
    throttle_percent = ((sensor40_shifted*MAX_THROTTLE)-MIN_40D*100)/(MAX_40D-MIN_40D);
  }
  
  // Serial.printf("90D: %d,\t40D: %d\n",sensor90_shifted, sensor40_shifted);

  // 10% rule
  // if difference in sensors is >10%, set throttle_active=0
  if (throttle_scaled_40D-throttle_scaled_90D > 0.1 || throttle_scaled_40D-throttle_scaled_90D < -0.1) {
    if (!counting) {
      previous_millis = current_millis; // start counting
      counting = true;
    } else {
      if (current_millis-previous_millis >= interval) {
        t_active = false;
      }
    }
  } else {
    counting = false;
    t_active = true;
  }
  t_active = true;
  p_timer_group.Tick(millis());
  g_timer_group.Tick(millis());
}
