#include "GPS.h"
#include <Adafruit_GPS.h>
#include <WString.h>


myGPS test(11, 12);

void setup() {

  Serial.begin(9600);
  test.getGpsData();


  return 0;

}
void loop() {
  //Serial.print("Contructor");


}


