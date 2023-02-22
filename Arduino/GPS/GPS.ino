#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int chipSelect = 6; // chip select for SD module
String filename = "gpsLog.csv"; // filename for saving to SD card

static const int RXPin = 2, TXPin = 1; // pins for ATGM336H GPS device
static const uint32_t GPSBaud = 9600; // default baudrate of ATGM336H GPS device

TinyGPSPlus gps;
SoftwareSerial ss(TXPin, RXPin);

void setup() {
  Serial.begin(9600); // start serial monitor for debugging
  ss.begin(GPSBaud);

}

void loop() {
  while (ss.available() > 0){
    if (gps.encode(ss.read()) && gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
      String data_to_save = ""; // data string for saving
      data_to_save += String(gps.date.month())+"/"+String(gps.date.day())+"/"+
                      String(gps.date.year())+",";
      data_to_save += String(gps.time.hour())+":"+String(gps.time.minute())+":"+
                      String(gps.time.second())+"."+String(gps.time.centisecond())+",";
      data_to_save += String(gps.location.lat(),8)+","; // latitude
      data_to_save += String(gps.location.lng(),8); // longitude
      
      Serial.println(data_to_save); // uncomment to print GPS data
     
    }

  }

}
