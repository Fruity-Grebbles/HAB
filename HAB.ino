// Based on Ladyada & Bill Greimanfor's logger
// for the Adafruit Ultimate GPS Shield

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "StemHab.h"

#define MOTOR_TRIGGER_ALTITUDE 250

const int chipSelect = 4;
bool motor_flag = false;

char filename[15];

SoftwareSerial mySerial(10, 11);


File logfile;

void setup() {
  Serial.begin(57600);
  Serial.print("Initializing SD card...");
  while(!Serial){;}
  mySerial.begin(4800);


  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");

  //The following prevents clobbering old log files
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }




  // enable interrupt
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  Serial.println("Ready!");
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = mySerial.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void loop() {
//  logfile = SD.open(filename, FILE_WRITE);
  if (mySerial.available()) {
    logfile = SD.open(filename, FILE_WRITE);
    int x = (int) mySerial.read();
    Serial.write(x);
    logfile.println(x);
    logfile.close();
  }
//  if (Serial.available()) {
//    mySerial.write(Serial.read());
//  }
//  logfile.println(mySerial.read());
//  logfile.close();
}
