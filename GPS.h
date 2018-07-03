
#include <Adafruit_GPS.h>
#include <WString.h>
#include <stdio.h>

class myGPS {
  public:
    Adafruit_GPS* GPS;
    myGPS(int gps_tx, int gps_rx);

    String getGpsData();
};


