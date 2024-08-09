#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SensorQMI8658.hpp>
#include <TFT_eSPI.h>
#include <LoRa.h>
#include <SPI.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and aSerial1umes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 17, TXPin = 18;
static const uint32_t GPSBaud = 4800;
void displayInfo();

//const LoRaWANBand_t Region = US915;
//const uint8_t subBand = 2;  // For US915, change this to 2, otherwise leave on 0

SPIClass rspi(SPI2_HOST);

// The TinyGPSPlus object
TinyGPSPlus gps;
TFT_eSPI tft(240, 240);

// The serial connection to the GPS device

void setup()
{
  Serial.begin(9600);
  
  Serial1.begin(9600);
  Serial1.setPins(17, 18);

  //Serial.println(F("DeviceExample.ino"));
  //Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  //Serial.println(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  //Serial.println(F("by Mikal Hart"));
  //Serial.println();
  pinMode(40, OUTPUT);
  pinMode(16, OUTPUT);
  //pinMode(0, INPUT);
  digitalWrite(40, HIGH);
  digitalWrite(16, HIGH);
  
  tft.begin();
  tft.fillScreen(TFT_RED);
  rspi.begin(21, 35, 36);
  LoRa.setSPI(rspi);
  LoRa.setPins(13, 37, 2);
   if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);
}
uint64_t timer = 0;
String pL;
bool newData = true;
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial.available() > 0) Serial1.write(Serial.read());
  while (Serial1.available() > 0){
    int c = Serial1.read();
    Serial.write(c);
    if (gps.encode(c))
        newData = true;
  }
  if(newData) displayInfo();
  newData = false;
  if(millis() - timer >= 5000) {
    LoRa.beginPacket();
  LoRa.print(String(gps.location.lat(), 6));
  LoRa.print(F(","));
  LoRa.print(String(gps.location.lng(), 6));
  LoRa.endPacket();
    Serial.println("\n------------------------------\n Packet Sent" + String("\n---------------------------------------"));
    timer = millis();
  }
  else {
    
    if(LoRa.parsePacket() != 0) {
      pL = "";
      while(LoRa.available()) pL += char(LoRa.read());
      displayInfo();
  }
}
}

void displayInfo()
{
  tft.fillRect(0, 100, 240, 140, TFT_BLACK);
  tft.setCursor(0, 100);
  
  tft.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    tft.print(gps.location.lat(), 6);
    tft.print(F(","));
    tft.print(gps.location.lng(), 6);
  }
  else
  {
    tft.print(F("INVALID"));
  }

 tft.println(F("\nDate/Time: "));
  if (gps.date.isValid())
  {
   tft.print(gps.date.month());
   tft.print(F("/"));
   tft.print(gps.date.day());
   tft.print(F("/"));
   tft.print(gps.date.year());
  }
  else
  {
   tft.print(F("INVALID"));
  }

 tft.println(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)tft.print(F("0"));
   tft.print(gps.time.hour());
   tft.print(F(":"));
    if (gps.time.minute() < 10)tft.print(F("0"));
   tft.print(gps.time.minute());
   tft.print(F(":"));
    if (gps.time.second() < 10)tft.print(F("0"));
   tft.print(gps.time.second());
   tft.print(F("."));
    if (gps.time.centisecond() < 10)tft.print(F("0"));
   tft.print(gps.time.centisecond());
  }
  else
  {
   tft.print(F("INVALID"));
  }
  tft.print("\nStatus:");
  tft.print(gps.satellites.value());
  tft.print(" | " + String(gps.satellites.age()));
  tft.print(" | " + String(gps.satellites.isValid()));
  tft.println(" | " + String(gps.satellites.isUpdated()));
  tft.println("Last Recieved: \"" + pL + "\"");
  tft.print("RSSI: " + String(LoRa.packetRssi()));


  //Serial.println();
}