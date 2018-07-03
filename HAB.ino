#include "GPS.h"
#include <Adafruit_GPS.h>
#include <WString.h>


myGPS test(11, 12);

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = test.GPS->read();
}


void setup() {

  Serial.begin(9600);

  return 0;

}

void loop() {
  Serial.println(test.getGpsData());
}


