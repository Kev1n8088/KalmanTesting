#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BNO080_Arduino_Library.h"  

#define SEALEVELPRESSURE_HPA (1013.25)

float gyroBias[3] = {0,0,0};
float oldGyroBias[3] = {0,0,0};

float gyroMeasurement[3] = {0,0,0}; 
float eulerOrientation[3] = {0,0,0};
float quanterionOrientation[4] = {0,0,0,0};  

// put function declarations here:

BNO080 IMU;


void updateBiases(){
  //called every 5 seconds, updates gyroBias with new gyroBias, and oldGyroBias with gyroBias. Always use oldGyroBias for launch.
  if (IMU.dataAvailable()){
    for(int i = 0; i < 3; i++){
      oldGyroBias[i] = gyroBias[i];
    }

    gyroBias[0] = IMU.getUncalibratedGyroBiasX();
    gyroBias[1] = IMU.getUncalibratedGyroBiasY();
    gyroBias[2] = IMU.getUncalibratedGyroBiasZ();
  }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  IMU.begin();
  IMU.enableUncalibratedGyro(50);
}

void loop() {
  // put your main code here, to run repeatedly:
}
