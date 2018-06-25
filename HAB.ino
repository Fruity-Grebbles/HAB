// GPS Utility Functions Start
#include "Adafruit_GPS.h"
#define GPS_TX 2
#define GPS_RX 3
//
//SoftwareSerial mySerial( GPS_TX, GPS_RX );
//Adafruit_GPS GPS(&mySerial);
Adafruit_GPS GPS(&Serial1);

//#define mySerial Serial2    // For Mega
//HardwareSerial mySerial = Serial1;
//Adafruit_GPS GPS(&mySerial);

String oldGpsData = "";

void startGPS() {
  /*
    int rate[] = {2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200 };
  for(int i = 0; i < 8; i+= 1)
  {
    GPS.begin(rate[i]);
    Serial.println("testing");
    if (GPS.available()) {
         char c = GPS.read();
         Serial.println(rate[i]);
         Serial.println(c);
    }
    else {
      Serial.println("no connection");
     }
  }
  */
  GPS.begin(4800);
//    mySerial.begin(4800);

  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
}

String getGpsData() {
  if (!GPS.parse(GPS.lastNMEA() )  ) {
    printDebug("Error Fetching GPS data or no new data recieved");
    Serial.println("No connection");
  
    // return oldGpsData;
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
  
  str += GPS.day;
  str += "/";
  
  str += GPS.year;
  str += ",";
  
  str += (int) GPS.fix;
  str += ",";
  
  str += (int) GPS.fixquality;
  str += ",";
  
  str += GPS.latitude;
  str += GPS.lat;
  str += ",";
  
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
  
  oldGpsData = str;
  return str;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}
