
#include <Adafruit_GPS.h>
#include <WString.h>
#include <stdio.h>

class myGPS {
  private:
    Adafruit_GPS* GPS;
  public:
    myGPS(int gps_tx, int gps_rx);

    String getGpsData();
    void handler();
  
};


