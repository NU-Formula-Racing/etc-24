#include <Arduino.h>
#include "teensy_can.h"
#include "VirtualTimer.h"

#define MIN_90D 80
#define MAX_90D 720
#define MIN_40D 80
#define MAX_40D 720


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
  p_bus, 0x010, 3, 10, g_timer_group, g_throttle_percent, g_throttle_active};

CANTXMessage<1> brake_tx_g{
  p_bus, 0x011, 1, 10, g_timer_group, g_brake_pedal};

// utility global variables
float sensor_voltage_90D;
float sensor_voltage_40D;

unsigned long previous_millis = 0;
const long interval = 100;
bool first_switch = true;

// values to send over CAN
int16_t throttle_percent;
bool brake_pressed;
bool t_active;

// throttle percent variables for sensor testing
int16_t throttle_percent_90D;
int16_t throttle_percent_40D;

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

}

void loop() {

  unsigned long current_millis = millis();
  // if (current_millis-previous_millis >= interval) {
  //   if (first_switch) {
  //     throttle_percent = 5;

  //   } else {
  //     throttle_percent = 10;
  //   }
    
  //   brake_pressed = !brake_pressed;
  //   t_active = !t_active;
  //   previous_millis = current_millis;
  //   first_switch = !first_switch;
  // }

  sensor_voltage_90D = analogRead(19);
  sensor_voltage_40D = analogRead(18);
   // scales throttle signal to a value between zero and one based on max and min input voltages
  double sensor_scaling_90D = (sensor_voltage_90D - MIN_90D)/(MAX_90D-MIN_90D);
  double sensor_scaling_40D = (sensor_voltage_40D - MIN_40D)/(MAX_40D-MIN_40D);

  if (sensor_scaling_90D <= 0.0) {
    sensor_scaling_90D = 0.0;
  } else if (sensor_scaling_90D >= 1.0) {
    sensor_scaling_90D = 1.0;
  }

  if (sensor_scaling_40D <= 0.0) {
    sensor_scaling_40D = 0.0;
  } else if (sensor_scaling_40D >= 1.0) {
    sensor_scaling_40D = 1.0;
  }

  // printing throttle percents, scaled from 0 to 32768
  throttle_percent_90D = sensor_scaling_90D*32768;
  throttle_percent_40D = sensor_scaling_40D*32768;
  Serial.printf("90D: %d,\t40D: %d\n",throttle_percent_90D, throttle_percent_40D);

  // 10% rule
  // if difference in sensors is >10%, set throttle_active=0
  if (sensor_scaling_40D-sensor_scaling_90D > 0.1) {
    if (current_millis-previous_millis >= interval) {
      t_active = false;
    } 
    }
  } else {
    t_active = true;
  }

  //Serial.println(current_millis-previous_millis);

  // throttle_percent = 5;
  // brake_pressed = false;
  // t_active = false;

  p_timer_group.Tick(millis());
  g_timer_group.Tick(millis());
}
