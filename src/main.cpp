#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Kalman.h> // Include the Kalman library by Romain Fetick
#include <math.h>   // Include math library for trigonometric functions

#define SEALEVELPRESSURE_HPA (1013.25)

float gyroBias[3] = {0, 0, 0};
float oldGyroBias[3] = {0, 0, 0};

float gyroMeasurement[3] = {0, 0, 0};
float eulerOrientation[3] = {0, 0, 0};
float quanterionOrientation[4] = {0, 0, 0, 0};

float baroMeaurement = 0; // baro measurement in meters adjusted by sealevel HPA

// put function declarations here:

BNO080 IMU;

Adafruit_BMP3XX bmp;

void updateBiases()
{
  // called every 5 seconds, updates gyroBias with new gyroBias, and oldGyroBias with gyroBias. Always use oldGyroBias for launch.
  for (int i = 0; i < 3; i++)
  {
    oldGyroBias[i] = gyroBias[i];
  }

  gyroBias[0] = IMU.getUncalibratedGyroBiasX();
  gyroBias[1] = IMU.getUncalibratedGyroBiasY();
  gyroBias[2] = IMU.getUncalibratedGyroBiasZ();
}

void getGyroRates()
{
  gyroMeasurement[0] = IMU.getGyroX();
  gyroMeasurement[1] = IMU.getGyroY();
  gyroMeasurement[2] = IMU.getGyroZ();
}

void gryoFilter()
{
  /*eulerOrientation[0] = kalmanX.getAngle(gyroMeasurement[0] - oldGyroBias[0], 0.01);
  eulerOrientation[1] = kalmanY.getAngle(gyroMeasurement[1] - oldGyroBias[1], 0.01);
  eulerOrientation[2] = kalmanZ.getAngle(gyroMeasurement[2] - oldGyroBias[2], 0.01);*/
}

void gyroQuanternion()
{
  float cy = cos(eulerOrientation[2] * 0.5);
  float sy = sin(eulerOrientation[2] * 0.5);
  float cp = cos(eulerOrientation[1] * 0.5);
  float sp = sin(eulerOrientation[1] * 0.5);
  float cr = cos(eulerOrientation[0] * 0.5);
  float sr = sin(eulerOrientation[0] * 0.5);

  quanterionOrientation[0] = cr * cp * cy + sr * sp * sy;
  quanterionOrientation[1] = sr * cp * cy - cr * sp * sy;
  quanterionOrientation[2] = cr * sp * cy + sr * cp * sy;
  quanterionOrientation[3] = cr * cp * sy - sr * sp * cy;
}

void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  Wire.setClock(400000); // Increase I2C data rate to 400kHz
  IMU.begin();
  IMU.enableUncalibratedGyro(50);

  if (!bmp.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
    // if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    // if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!IMU.dataAvailable())
  {
    return;
  }
  getGyroRates();
  gryoFilter();
  gyroQuanternion();
  // Now eulerOrientation contains the filtered orientation data
  Serial.print("Orientation X: ");
  Serial.print(eulerOrientation[0]);
  Serial.print(" Y: ");
  Serial.print(eulerOrientation[1]);
  Serial.print(" Z: ");
  Serial.print(eulerOrientation[2]);
  // Now quanterionOrientation contains the updated quaternion orientation data
  Serial.print("   Quaternion W: ");
  Serial.print(quanterionOrientation[0]);
  Serial.print(" X: ");
  Serial.print(quanterionOrientation[1]);
  Serial.print(" Y: ");
  Serial.print(quanterionOrientation[2]);
  Serial.print(" Z: ");
  Serial.println(quanterionOrientation[3]);

  if (!bmp.performReading())
  {
    return;
  }

  getBaro();
}
