// Based on Ladyada & Bill Greimanfor's logger
// for the Adafruit Ultimate GPS Shield

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

#define MOTOR_TRIGGER_ALTITUDE 250

const int chipSelect = 4;
bool motor_flag = false;

char filename[15];

SoftwareSerial mySerial(11, 12);
Adafruit_GPS GPS(&mySerial);

File logfile;

void setup() {
  Serial.begin(115200);
  Serial.print("Initializing SD card...");

  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");

  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA); // Turn off updates on antenna status
  
  // enable interrupt
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  Serial.println("Ready!");
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
        if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
}

void loop() {
  logfile = SD.open(filename, FILE_WRITE);
  Serial.println(getGpsData());
  logfile.println(getGpsData());
  logfile.close();
}

String getGpsData() {
  if (!GPS.parse(GPS.lastNMEA() )  ) {
    Serial.println("No connection");
    return "0,0,0,0,0,0,0,0,0,0,0,0";
  }
  String str = "";
  str += GPS.hour;
  str += ":";
  str += GPS.minute;
  str += ":";
  str += GPS.seconds;
  str += ",";
  str += GPS.month;
  str += "/";
  str += GPS.day + "/";
  str += GPS.year + ",";
  str += (int) GPS.fix + ",";
  str += (int) GPS.fixquality + ",";
  str += GPS.latitude;
  str += GPS.lat + ",";
  str += GPS.longitude;
  str += GPS.lon;
  str += ",";
  str += GPS.latitudeDegrees;
  str += ",";
  str += GPS.longitudeDegrees;
  str += ",";
  str += GPS.altitude;
  str += ",";
  str += GPS.speed;
  str += ",";
  str += GPS.angle;
  str += ",";
  str += (int)GPS.satellites;
  Serial.println("Log");
  return str;
}
