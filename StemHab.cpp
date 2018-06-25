// Stem High Altitude Balloon Project 2017

// SD CARD READ/WRITER
//		CLK --> PIN 13
//		DO --> 12
//		DI --> 11
//		CS --> 10 (Main Pin)
// PIN 11: MIC IN
// PIN 9: SPEAKER OUT
// PIN 8: LED
// IC2 MAGNETOMETER (3.3V) ADDRESS: 0x1E
// I2C C02 DETECT ADDRESS: 0X15
// ACCELEROMETER/GYRO
//  A1: X-AXIS
//  A2: Y-AXIS
//  A3: Z-AXIS

// SDA --> A4
// SCL --> A5

// Main Arduino Pin 13 --> SD Arduino Pin 7
// Main Arduino Pin 12 --> SD Arduino Pin 8


#include <Wire.h>
#include <Serial.h>


#include "RTClib.h"
#ifdef __AVR__
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#elif defined(ARDUINO_ARCH_SAMD)

#elif defined(ARDUINO_SAM_DUE)
 #define PROGMEM
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #define Wire Wire1
#endif

/*
//#include <SPI.h>
*/

bool isDebug = 0;
/*
void initSerial() {
	Serial.begin(9600);
	while(!Serial);
}

void initI2C() {
	Wire.begin();
}
*/
// Debug Mode will print out more specific details to the serial console
bool SET_DEBUG(bool debug) {
	isDebug = debug;
}

void printDebug(String str) {
	if (isDebug)
		Serial.println("DEBUG: " + str);
}

/*
// LED Utility Functions Start
#define LED_PIN 8 // (Using PWM Pin)
void startLED() {
	pinMode(LED_PIN,OUTPUT);
}
void setLED(bool val) {
	digitalWrite(LED_PIN,val?HIGH:LOW);
}
void flashLED() {
	// Time: Milliseconds
	for(unsigned int i = 0; i < 1000; i+=60) {
		setLED(true);
		delay(30);
		setLED(false);
		delay(30);
	}
	setLED(false);
}
// LED Utility Functions End

*/


// TEmperature 

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

float getRawTemp(int ThermistorPin) {
	return analogRead(ThermistorPin);
}

// Return Temp in Farenheit 
float getTempF( float tempC ) {

	return ( tempC * 1.8 + 32 ); 
}


#define pressurePin A3
// The sensor pressure is connected to A3 vout
//Create an instance of the object
int getRawPressure() {
	// Return a value from 0-1024 
	return analogRead( pressurePin );
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


// Speaker Utility Functions Start
#define SPEAKER_PIN 10

void speakerStartSound() {

//	pinMode(SPEAKER_PIN, OUTPUT);
	tone( SPEAKER_PIN, 3800);

//	setSpeaker(0);
}
// Speaker Utility Functions End




// Real Time Clock Utility functions start


//#include "RTClib.h"
/*
RTC_DS1307 RTC;

bool isRTCActive = 0;

void setRTCTime() {
		RTC.adjust(DateTime(F(__DATE__),F(__TIME__) ) );
}

bool startRTC() {
	bool res = RTC.begin();
	
	printDebug(res?"RTC Is Working":"RTC Is not Working");
	
	isRTCActive = res;
	
	// Debug Mode will automatically adjust the RTC's time to the
	// time the code was compiled. Turn debug mode off before balloon launch
	if (isDebug)
		setRTCTime();
	
	return res;
}

bool RTCRunning() {
	return RTC.isrunning();
}

String getDate(char separator) {
	DateTime now = RTC.now();
	String str = "";
	
	str += now.month();
	str += separator;
	
	str += now.day();
	str += separator;
	
	str += now.year();
	
	return str;
}

String getDate() {
	return getDate('/');
}

String getTime(char separator) {
	DateTime now = RTC.now();
	String str = "";
	
	str += now.hour();
	str += separator;
	
	str += now.minute();
	str += separator;
	
	str += now.second();
	
	return str;
}

String getTime() {
	return getTime(':');
}
// Real Time Clock Utility functions end
*/

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
//  	mySerial.begin(4800);

	
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
// GPS Utility Functions End

/*
// Mic Utility Functions Start
#define MIC_PIN 0 // (Analog)

unsigned int sample;
int noise;

#define SAMPLE_TOTAL 1000
#define SAMPLE_RECORD_SECONDS 1

String getMicData() {
	
	
	unsigned long startMillis= millis();  // Start of sample window
	unsigned int peakToPeak = 0;   // peak-to-peak level

	unsigned int signalMax = 0;
	unsigned int signalMin = 1024;

	// collect data for 50 mS
	for(int i = 0; i < SAMPLE_TOTAL; i+= 1)
	{
		sample = analogRead(MIC_PIN);
		noise = analogRead(MIC_PIN);

		if (sample < 1024)  // toss out spuriousi readings
		{
			if (sample > signalMax)
				signalMax = sample;  // save just the max levels
			else if (sample < signalMin)
				signalMin = sample;  // save just the min levels
		}
		
		//delay(1);
		
		delay( (SAMPLE_RECORD_SECONDS * 1000) / SAMPLE_TOTAL);
	}
	peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
	double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
	
	return (String(noise) + "," + String(sample) + "," + String(volts) );
}

*/

/*
#define SAMPLE_TOTAL 500
#define SAMPLE_RECORD_SECONDS 1


long a = 0;
String getMicData() {
	//int data[SAMPLE_TOTAL];
	
	String out = "";
	//unsigned long total = 0;
	for(int i = 0; i < SAMPLE_TOTAL; i++) {
		//data[i] = analogRead(MIC_PIN);
		//total += analogRead(MIC_PIN);
		
		//out += String(analogRead(MIC_PIN) ) + ",";
		
		Serial.println(String(a++) + "," + String(analogRead(MIC_PIN) ) );
		delay( (SAMPLE_RECORD_SECONDS * 1000) / SAMPLE_TOTAL);
	}
	
	//int avg = total / SAMPLE_TOTAL;
	
	//return String(out);
	return "";
}*/
