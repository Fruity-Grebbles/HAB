
void startGPS(int gps_tx, int gps_rx) {
  SoftwareSerial mySerial(gps_tx, gps_rx); //The Adalogger 32u4 has no available hardware serial pins, so we use SoftwareSerial
  Adafruit_GPS GPS(&mySerial);
  GPS.begin(4800);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
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