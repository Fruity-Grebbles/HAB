// Stem High Altitude Balloon Project 2017
// Modified for Stem High Altitude Balloon 2018 by Joshua Grebler

#include <math.h> 
// Temperature 
// resistance at 25 degrees C
#define THERMISTORNOMINAL 1020 
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
// This value is from the dataSheet
#define BCOEFFICIENT 3348
// the value of the 'other' resistor
#define SERIESRESISTOR 1020

// Return the temperature from Analog pin
float getTemp( float rawTemp, float Vin ) {
	float tt = rawTemp;
	tt = 0.0048828*tt;
	// Formula to find the value of the resistor (thermistor) or How to reduce voltage
	// http://www.learningaboutelectronics.com/Articles/How-to-reduce-voltage-with-resistors.php
	// Formula to find the Resistor Value
	tt = (SERIESRESISTOR * tt) /( Vin - tt );   
 	float steinhart;
	
	steinhart = tt / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;                 // Invert
	steinhart -= 273.15;                         // convert to C
	return steinhart;
}

// Return Temp in Farenheit 
float getTempF( float tempC ) {
	return ( tempC * 1.8 + 32 ); 
}

// The sensor Honeywell SSC 1.6BAAA3 (Analog 3.3v)
// read the value from the sensor:
// CApture the raw data from the analog Pin 3  0-3.3v value
// The arduino read analog signal from 0-5v for that reason doesnt need resistor bridge to drop the voltage
// Formula from the beaglebone was Pressure = (sensorValue * 1.8*2 -.33)/1.65)*100; (KPascal)
// the 1.8 * 2 was because the resistor divider to drop the voltage to 1.8 and the reading from the p9-39 return 
// value from 0-1.
float getPressure( int rawValue ) {
	float pressure = ((rawValue * 0.0049 -.33)/1.65)*100;
	return pressure;
}

// GPS Utility Functions Start

#define GPS_TX 2
#define GPS_RX 3
//

Adafruit_GPS GPS(&mySerial);
void startGPS(int gps_tx, int gps_rx) {
  SoftwareSerial mySerial(gps_tx, gps_rx); //The Adalogger 32u4 has no available hardware serial pins, so we use SoftwareSerial
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
