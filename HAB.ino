#include <SPI.h>
#include <SD.h>
#include "GPS.h"
#include <Adafruit_GPS.h>
#include <WString.h>

const int chipSelect = 4;
myGPS test(11, 12);

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = test.GPS->read();
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
  Serial.println(test.getGpsData());
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}
