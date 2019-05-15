#include <Wire.h>
#include "softMS5837.h"

softMS5837 pressure_sensor;

#define CMD_PRESSURE        0x20
#define CMD_TEMPERATURE     0x21
#define CMD_WHOAMI          0x77
#define WHOAMI_ID           0x69

volatile uint8_t lastByte = 0x00;

void setup() {
  Wire.begin(0x75);                // join i2c bus with address 0x75
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  while (!pressure_sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  
  pressure_sensor.setModel(softMS5837::MS5837_30BA);
  pressure_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

int32_t pressure = 0;
int32_t temperature = 0;

void loop() {
  delay(100);

  // read pressure sensor over software i2c
  pressure_sensor.read();
  pressure = round(pressure_sensor.pressure() * 100);
  temperature = round(pressure_sensor.temperature() * 10);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire.available()) {
    lastByte = Wire.read();
  }
}

void requestEvent(void) {
  switch (lastByte) {
    case CMD_WHOAMI:
        Wire.write(WHOAMI_ID);
        break;
    case CMD_PRESSURE:
        Wire.write((uint8_t *)&pressure, 4);
    case CMD_TEMPERATURE:
        Wire.write((uint8_t *)&temperature, 4);
  } 
}
