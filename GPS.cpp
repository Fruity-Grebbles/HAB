#include <Adafruit_GPS.h>

class myGPS {
  private:
    Adafruit_GPS* GPS;
  public:
    myGPS(int gps_tx, int gps_rx);
    String getGpsData();
    void handler();
};

myGPS::myGPS(int gps_tx, int gps_rx) {
  SoftwareSerial mySerial(gps_tx, gps_rx); //The Adalogger 32u4 has no available hardware serial pins, so we use SoftwareSerial
  GPS = new Adafruit_GPS(&mySerial);
  GPS->begin(4800);
  // Set the update rate
  GPS->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  //attatchInterrupt(TIMER0_COMPA_vect, handler);
}

String myGPS::getGpsData() {
  if (!GPS->parse(GPS->lastNMEA() )  ) {
    Serial.println("No connection");
    return "0,0,0,0,0,0,0,0,0,0,0,0";
  }
  String str = "";

  str += GPS->hour;
  str += ":";
  str += GPS->minute;
  str += ":";
  str += GPS->seconds;
  str += ",";
  str += GPS->month;
  str += "/";
  str += GPS->day + "/";
  str += GPS->year + ",";
  str += (int) GPS->fix + ",";
  str += (int) GPS->fixquality + ",";
  str += GPS->latitude;
  str += GPS->lat + ",";
  str += GPS->longitude;
  str += GPS->lon;
  str += ",";
  str += GPS->latitudeDegrees;
  str += ",";
  str += GPS->longitudeDegrees;
  str += ",";
  str += GPS->altitude;
  str += ",";
  str += GPS->speed;
  str += ",";
  str += GPS->angle;
  str += ",";
  str += (int)GPS->satellites;
  return str;
}
static void myGPS::handler() {
  char c = GPS->read();
}
