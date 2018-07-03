#include <SPI.h>
#include <SD.h>
#include "GPS.h"
#include <Adafruit_GPS.h>
#include <WString.h>
#define MOTOR_TRIGGER_ALTITUDE 250

const int chipSelect = 4;
myGPS module(11, 12);
bool motor_flag = false;

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = module.GPS->read();
}


void setup() {
  Serial.begin(9600);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
  String dataString = "";
  Serial.println(module.getGpsData());
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);

    //Motor handling logic
    if (module.GPS->altitude > MOTOR_TRIGGER_ALTITUDE) {
      if (!motor_flag) {
        motor_flag = true;
        dataFile.println("Dishes opened");
      }
    } else {
      if(motor_flag) {
        dataFile.println("Dishes closed");
      }
    }

    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}
