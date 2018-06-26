#include "./GPS.h"

void setup() {
  Serial.begin(9600);
  GPS test(11, 12);
  test.getGpsData();

  
 






}
void loop() {

}
