#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft(240, 240);

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(9600);
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  tft.begin();
  tft.fillScreen(0xFF0000);
  //tft.setTextColor(0xFFFFFF);
  tft.setTextSize(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello World!");
  tft.fillScreen(random(0xFFFFFF));
  
  tft.drawString("Hello World!", 90, 110);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}