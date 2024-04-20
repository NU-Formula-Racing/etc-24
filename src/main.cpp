#include <Arduino.h>
#include "teensy_can.h"
#include "VirtualTimer.h"
TeensyCAN<1> priority_bus{};
// TeensyCAN<2> general_bus{};
VirtualTimerGroup timer_group{};
MakeSignedCANSignal(int16_t, 0, 16, 1, 0) priority_throttle_percent{};
MakeSignedCANSignal(bool, 16, 8, 1, 0) Throttle_active{};
MakeSignedCANSignal(bool, 0, 8, 1, 0) Brake_pedal{};

CANTXMessage<1> throttle_tx_p{
  priority_bus, 0x010, 2, 10, timer_group, priority_throttle_percent};


// CANTXMessage<2> throttle_tx_p{
//   priority_bus, 0x010, 3, 100, timer_group, Throttle_percent, Throttle_active};

// CANTXMessage<1> brake_tx_p{
//   priority_bus, 0x010, 1, 100, timer_group, Brake_pedal};

// CANTXMessage<2> throttle_tx_g{
//   general_bus, 0x010, 2, 100, timer_group, Throttle_percent, Throttle_active};

// CANTXMessage<1> brake_tx_g{
//   general_bus, 0x010, 1, 100, timer_group, Brake_pedal};


int16_t sensor_voltage;
bool brake_pressed;
bool t_active;

void ten_ms_task() {
  //Throttle_percent = sensor_voltage;
  priority_throttle_percent = 5;
  Throttle_active = t_active;
  Brake_pedal = brake_pressed;
  priority_bus.Tick();
  // Serial.println("task running!");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(19, INPUT);
  Serial.begin(115200);

  priority_bus.Initialize(ICAN::BaudRate::kBaud1M);
  //general_bus.Initialize(ICAN::BaudRate::kBaud1M);
  timer_group.AddTimer(10, ten_ms_task);
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  // sensor_voltage = analogRead(19);
  // sensor_voltage = (sensor_voltage/1023)*6.6;
  //Serial.println(sensor_voltage);
  timer_group.Tick(millis());
}
