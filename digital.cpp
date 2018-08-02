String getSensorData() {
  String str = "";

  Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }

  str += temp0;
  str += ",";
  str += tempF0;
  str += ",";
  str += temp1;
  str += ",";
  str += tempF1;
  str += ",";
  str += temp2;
  str += ",";
  str += tempF2;
  str += ",";
  str += pressure;

  return str;
}