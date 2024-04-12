#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"
#include <MadgwickAHRS.h>

SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

TFT_eSPI tft(240, 240);

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  tft.begin();
  tft.fillScreen(0xFF0000);
  //tft.setTextColor(0xFFFFFF);
  tft.setTextSize(1);
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, 6, 7)) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
  }
  Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    qmi.configAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        SensorQMI8658::ACC_RANGE_2G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
        * */
        SensorQMI8658::ACC_ODR_1000Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        * */
        SensorQMI8658::LPF_MODE_0,
        // selfTest enable
        true);


    qmi.configGyroscope(
        /*
        * GYR_RANGE_16DPS
        * GYR_RANGE_32DPS
        * GYR_RANGE_64DPS
        * GYR_RANGE_128DPS
        * GYR_RANGE_256DPS
        * GYR_RANGE_512DPS
        * GYR_RANGE_1024DPS
        * */
        SensorQMI8658::GYR_RANGE_256DPS,
        /*
         * GYR_ODR_7174_4Hz
         * GYR_ODR_3587_2Hz
         * GYR_ODR_1793_6Hz
         * GYR_ODR_896_8Hz
         * GYR_ODR_448_4Hz
         * GYR_ODR_224_2Hz
         * GYR_ODR_112_1Hz
         * GYR_ODR_56_05Hz
         * GYR_ODR_28_025H
         * */
        SensorQMI8658::GYR_ODR_896_8Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        * */
        SensorQMI8658::LPF_MODE_3,
        // selfTest enable
        true);


    // In 6DOF mode (accelerometer and gyroscope are both enabled),
    // the output data rate is derived from the nature frequency of gyroscope
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    // Print register configuration information
    qmi.dumpCtrlRegister();

    // start  filter
    filter.begin(25);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();

    Serial.println("Read data now...");
}
int state;
int lstate;
void loop() {
  // put your main code here, to run repeatedly:
  float roll, pitch, heading;

    // check if it's time to read data and update the filter
    if (micros() - microsPrevious >= microsPerReading) {

        // read raw data from IMU
        if (qmi.getDataReady()) {
            qmi.getAccelerometer(acc.x, acc.y, acc.z);
            qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
            // update the filter, which computes orientation
            filter.updateIMU(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);

            // print the heading, pitch and roll
            roll = filter.getPitch();
            if(filter.getRoll() > 0) pitch = 180 - filter.getRoll();
            else pitch = (180 + filter.getRoll()) * -1;
            heading = filter.getYaw();
            Serial.print("Orientation: ");
            Serial.print(heading);
            Serial.print(" ");
            Serial.print(pitch);
            Serial.print(" ");
            Serial.print(roll);
            Serial.print(" ");
            Serial.println(state);
        }

                if(abs(pitch) > abs(roll)) {
      if(pitch > 3) state = 1;
      else if(pitch < -17) state = 3;
      else state = 0;
    }
    else{
      if(roll > 10) state = 2;
      else if(roll < -10) state = 4;
      else state = 0;
    }
    if(state != lstate){
      switch (state)
      {
      case 1:
        tft.fillScreen(TFT_RED);
        break;
      case 2:
        tft.fillScreen(TFT_YELLOW);
        break;
      case 3:
        tft.fillScreen(TFT_GREEN);
        break;
      case 4:
        tft.fillScreen(TFT_BLUE);
        break;
      case 0:
        tft.fillScreen(0x0000);
        break;
      }
    }
    lstate = state;
    }
        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
}