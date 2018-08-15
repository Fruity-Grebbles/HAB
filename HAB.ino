// Based on Ladyada & Bill Greimanfor's logger
// for the Adafruit Ultimate GPS Shield

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "StemHab.h"

const int MOTOR_TRIGGER_ALTITUDE = 46;

void SDwrite();
void openDoor();
void closeDoor();

const int chipSelect = 4;
bool motor_flag = false;

SoftwareSerial mySerial(10, 9);
int motor_one_p1 = A4;
int motor_one_p2 = 12;
int motor_two_p1 = A5;
int motor_two_p2 = 11;

String singleLine = "";

File logfile;
String fileName = "GPSLOG";
bool flag = true;
bool done = false;

void setup() {
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  while (!Serial) {
    ;
  }


  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");

  for (uint8_t i = 0; i < 1000; i++) {
    if (!SD.exists(fileName + i + ".csv")) {
      fileName = fileName + i + ".csv";
      break;
    }
  }

  SDwrite("Type,Time,Latitude,Longitude,Quality,Number Of Satellites,Horizontal Dilution of Precision (HDOP),Altitude,Height of geoid above WGS84 ellipsoid,DGPS reference station id,Checksum,Temp 1 (A1), Temp 1 (A1 RAW), Temp 2 (A2), Temp 2 (A2 RAW),Door Status");
  mySerial.begin(4800);
  Serial.println("Ready!");
}

struct singleLineParsed {
  String full;
  String type;
  String time;
  String lat;
  String lng;
  String quality;
  String numOfSatellites;
  String HDOP;
  String altitude;
  String WGS84_ellipsoid;
  String DGPS_reference;
  String checkSum;
};

void SDwrite(String x) {
  Serial.println(x);
  logfile = SD.open(fileName, FILE_WRITE);
  logfile.println(x);
  logfile.close();
}

void openDoor() {
  digitalWrite(A4, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(A5, HIGH);
  digitalWrite(11, LOW);
}

void closeDoor() {
  digitalWrite(motor_one_p1, LOW);
  digitalWrite(motor_one_p2, HIGH);
  digitalWrite(motor_two_p1, LOW);
  digitalWrite(motor_two_p2, HIGH);
}

void loop() {
  //  Serial.println(getTemp(analogRead(A1), 5));
  //  Serial.println(getTemp(analogRead(A2), 5));
  openDoor();
  if (mySerial.available()) {
    char x = (char) mySerial.read();
    if (x == '\n') {
      singleLineParsed newLine;
      if (singleLine.substring(0, 10).indexOf("$GPGGA") != -1) {
        for (int i = 0; i < 20; i++) {
          if (i == 0) {
            newLine.full = singleLine;
          }
          int split = singleLine.indexOf(",");
          String y = singleLine.substring(0, split);
          singleLine = singleLine.substring(split + 1, singleLine.length());
          switch (i) {
            case 0:
              newLine.type =  y;
              break;
            case 1:
              newLine.time = y;
              break;
            case 2:
              newLine.lat = y;
              break;
            case 3:
              break;
            case 4:
              newLine.lng = y;
              break;
            case 5:
              break;
            case 6:
              newLine.quality = y;
              break;
            case 7:
              newLine.numOfSatellites = y;
              break;
            case 8:
              newLine.HDOP = y;
              break;
            case 9:
              newLine.altitude = y;
              break;
            case 10:
              break;
            case 11:
              newLine.WGS84_ellipsoid = y;
              break;
            case 12:
              break;
            case 13:
              break;
            case 14:
              newLine.DGPS_reference = singleLine.substring(0, 4);
              newLine.checkSum = singleLine.substring(4, singleLine.length() - 1);
              break;
          }

          if (singleLine.length() == 0)
            break;
        }
        if ((newLine.lng == "") != 1) {

          if (newLine.altitude.toInt() > MOTOR_TRIGGER_ALTITUDE) {
            SDwrite(
              newLine.type + "," + newLine.time + "," + newLine.lat + "," + newLine.lng + "," + newLine.quality + "," + newLine.numOfSatellites + "," + newLine.HDOP + "," + newLine.altitude + "," + newLine.WGS84_ellipsoid + "," + newLine.DGPS_reference + "," + newLine.checkSum + "," + getTemp(analogRead(A1), 5) + "," + analogRead(A1) + "," + getTemp(analogRead(A2), 5) + "," + analogRead(A2) + "," + "1");
            //            Serial.println(getTemp(analogRead(A0), 5));
            //            Serial.println(getTemp(analogRead(A1), 5));
            //            Serial.println(getTemp(analogRead(A2), 5));
//            openDoor();
          }
          if (newLine.altitude.toInt() < MOTOR_TRIGGER_ALTITUDE) {
            SDwrite(
              newLine.type + "," + newLine.time + "," + newLine.lat + "," + newLine.lng + "," + newLine.quality + "," + newLine.numOfSatellites + "," + newLine.HDOP + "," + newLine.altitude + "," + newLine.WGS84_ellipsoid + "," + newLine.DGPS_reference + "," + newLine.checkSum + "," + getTemp(analogRead(A1), 5) + "," + analogRead(A1) + "," + getTemp(analogRead(A2), 5) + "," + analogRead(A2) + "," + "0"
            );
            //            Serial.println(getTemp(analogRead(A0), 5));
            //            Serial.println(getTemp(analogRead(A1), 5));
            //            Serial.println(getTemp(analogRead(A2), 5));
//            closeDoor();
          }
          //          Serial.println(analogRead(A0));
        }
      }
      singleLine = "";
    } else {
      singleLine += x;
    }
  }
}
