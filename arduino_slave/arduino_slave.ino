#include "MS5837.h"
#include "ping1d.h"
#include <DS3231.h>
#include <Wire.h>
#include "SdFat.h"

#define FILENAME_LEN 40

SdFat sd;
SdFile logfile;
char filename[FILENAME_LEN];
const int chipSelect = 4;

DS3231 rt_clock;

static Ping1D ping { Serial2 };
MS5837 pressure_sensor;

#define CMD_PRESSURE        0x20
#define CMD_TEMPERATURE     0x21
#define CMD_DISTANCE        0x22
#define CMD_CONFIDENCE      0x23
#define CMD_WHOAMI          0x77
#define WHOAMI_ID           0x69

volatile uint8_t lastByte = 0;
uint8_t ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  Wire1.begin(0x75);                // join i2c bus with address 0x75
  Wire1.onReceive(receiveEvent); // register event
  Wire1.onRequest(requestEvent);
  
  Serial2.begin(115200);
  Serial.begin(115200);

  Serial.println("starting");

  while (!ping.initialize()) {
      Serial.println("\nPing device failed to initialize!");
      Serial.println("Are the Ping rx/tx wired correctly?");
      Serial.print("Ping rx is the green wire, and should be connected to Arduino pin tx");
      Serial.print("Ping tx is the white wire, and should be connected to Arduino pin rx");
      delay(2000);
  }

  while (!pressure_sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  
  pressure_sensor.setModel(MS5837::MS5837_30BA);
  pressure_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  Serial.println("initting loop");
  digitalWrite(ledPin, LOW);

  bool h12, PM;
  bool century = false;

  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    Serial.println("SD Card not found, continuing without datalogging");
    return;
  }

  
  sprintf(filename, "%02d-%02d-%02d %02d-%02d-%02d.csv", 
                          rt_clock.getYear(),
                          rt_clock.getMonth(century),
                          rt_clock.getDate(),
                          rt_clock.getHour(h12, PM),
                          rt_clock.getMinute(),
                          rt_clock.getSecond());

  if (!logfile.open(filename, O_WRONLY | O_CREAT | O_EXCL)) {
   Serial.println("Could not open file");
  }

  Serial.println(filename);
  logfile.println("millis, temperature, pressure, sonar, confidence");
  logfile.sync();
}

int32_t pressure = 0;
int32_t temperature = 0;
uint32_t distance = 0;
uint16_t confidence = 0;
char buff[100];
uint32_t lastmillis = 0;

void loop() {
  // read pressure sensor over software i2c
  pressure_sensor.read();
  pressure = round(pressure_sensor.pressure() * 100);
  temperature = round(pressure_sensor.temperature() * 10);

  if (ping.update()) {
    distance = ping.distance();
    confidence = ping.confidence();

    digitalWrite(ledPin, HIGH);

    sprintf(buff, "%u, %d, %d, %u, %u",
                    millis(),
                    temperature,
                    pressure,
                    distance,
                    confidence);
    logfile.print(buff);

//    if (ping.request(Ping1DNamespace::Profile)) {
//      uint8_t *pd = ping.profile_data();
//      uint16_t len = ping.profile_data_length();
//      Serial.println(len);
//      for (uint16_t i = 0; i < len; i++) {
//        logfile.write(',');
//        logfile.print(pd[i]);
//      }
//    }

    logfile.println("");

   
    logfile.sync();
    digitalWrite(ledPin, LOW);

    Serial.println(buff);
  }

  Serial.println(millis()-lastmillis);
  lastmillis = millis();
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire1.available()) {
    lastByte = Wire1.read();
  }
}

void requestEvent(void) {
//  Serial.print("request ");
//  Serial.println(lastByte, HEX);
  
  switch (lastByte) {
    case CMD_WHOAMI:
        Wire1.write(WHOAMI_ID);
        break;
    case CMD_PRESSURE:
        Wire1.write((uint8_t *)&pressure, 4);
        break;
    case CMD_TEMPERATURE:
        Wire1.write((uint8_t *)&temperature, 4);
        break;
    case CMD_DISTANCE:
        Wire1.write((uint8_t *)&distance, 4);
        break;
    case CMD_CONFIDENCE:
        Wire1.write((uint8_t *)&confidence, 2);
        break;
  } 
}
