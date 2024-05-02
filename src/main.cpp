#include <Arduino.h>
#include "teensy_can.h"
#include "VirtualTimer.h"

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
float sensor_voltage;

unsigned long previous_millis = 0;
const long interval = 2000;
bool first_switch = true;

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



void setup() {
  pinMode(19, INPUT);
  Serial.begin(115200);

  p_bus.Initialize(ICAN::BaudRate::kBaud1M);
  p_timer_group.AddTimer(10, p_ten_ms_task);

  g_bus.Initialize(ICAN::BaudRate::kBaud1M);
  g_timer_group.AddTimer(10, g_ten_ms_task);

}

void loop() {

  unsigned long current_millis = millis();
  if (current_millis-previous_millis >= interval) {
    if (first_switch) {
      throttle_percent = 5;

    } else {
      throttle_percent = 10;
    }
    
    brake_pressed = !brake_pressed;
    t_active = !t_active;
    previous_millis = current_millis;
    first_switch = !first_switch;
  }

  sensor_voltage = analogRead(19);
  sensor_voltage = (sensor_voltage/1023)*6.6;
  Serial.println(throttle_percent);
  //Serial.println(current_millis-previous_millis);

  // throttle_percent = 5;
  // brake_pressed = false;
  // t_active = false;

  p_timer_group.Tick(millis());
  g_timer_group.Tick(millis());
}
