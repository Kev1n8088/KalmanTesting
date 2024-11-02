#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include <Kalman.h>
#include <math.h> // Include math library for trigonometric functions

using namespace BLA;

#define BNO08X_INT -1
#define BNO08X_RST -1

// #define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

BNO08x IMU;

#define SEALEVELPRESSURE_HPA (1013.25)

float gyroBias[3] = {0, 0, 0};
float oldGyroBias[3] = {0, 0, 0};

float gyroMeasurement[3] = {0, 0, 0};
float filteredGyro[3] = {0, 0, 0};
float orientationEuler[3] = {0, 0, 0};
float orientationQuanternion[4] = {0, 0, 0, 0};

float baroMeaurement = 0; // baro measurement in meters adjusted by sealevel HPA

float accelMeasurement[3] = {0, 0, 0};
float oldAccelQuaternion[4] = {0, 0, 0, 0};
float accelQuaternion[4] = {0, 0, 0, 0};

// put function declarations here:

#define BNO08X_RESET -1

Adafruit_BMP3XX bmp;
void updateBiases()
{
  // called every 5 seconds, updates gyroBias with new gyroBias, and oldGyroBias with gyroBias. Always use oldGyroBias for launch.
  for (int i = 0; i < 3; i++)
  {
    oldGyroBias[i] = gyroBias[i];
  }
  gyroBias[0] = IMU.getUncalibratedGyroX();
  gyroBias[1] = IMU.getUncalibratedGyroY();
  gyroBias[2] = IMU.getUncalibratedGyroZ();
}

void getGyroRates()
{
  gyroMeasurement[0] = IMU.getUncalibratedGyroX();
  gyroMeasurement[1] = IMU.getUncalibratedGyroY();
  gyroMeasurement[2] = IMU.getUncalibratedGyroZ();
}

void gryoFilter()
{
  filteredGyro[0] = gyroMeasurement[0];
  filteredGyro[1] = gyroMeasurement[1];
  filteredGyro[2] = gyroMeasurement[2];
}

void gyroEuler(float dt)
{
  orientationEuler[0] += (filteredGyro[0] * dt);
  orientationEuler[1] += (filteredGyro[1] * dt);
  orientationEuler[2] += (filteredGyro[2] * dt);
}

void gyroQuanternion()
{
  float cy = cos(orientationEuler[2] * 0.5);
  float sy = sin(orientationEuler[2] * 0.5);
  float cp = cos(orientationEuler[1] * 0.5);
  float sp = sin(orientationEuler[1] * 0.5);
  float cr = cos(orientationEuler[0] * 0.5);
  float sr = sin(orientationEuler[0] * 0.5);

  orientationQuanternion[0] = cr * cp * cy + sr * sp * sy;
  orientationQuanternion[1] = sr * cp * cy - cr * sp * sy;
  orientationQuanternion[2] = cr * sp * cy + sr * cp * sy;
  orientationQuanternion[3] = cr * cp * sy - sr * sp * cy;
}

void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void updateAccelQuaternion()
{
  // called every 5 seconds, updates accelQuanterion with new accelQuanterion, and oldaccelQuanterion with accelQuanterion. Always use oldaccelQuanterion for launch.

  for (int i = 0; i < 3; i++)
  {
    oldAccelQuaternion[i] = accelQuaternion[i];
  }

  accelMeasurement[0] = IMU.getAccelX();
  accelMeasurement[1] = IMU.getAccelY();
  accelMeasurement[2] = IMU.getAccelZ();

  float cy = cos(accelMeasurement[2] * 0.5);
  float sy = sin(accelMeasurement[2] * 0.5);
  float cp = cos(accelMeasurement[1] * 0.5);
  float sp = sin(accelMeasurement[1] * 0.5);
  float cr = cos(accelMeasurement[0] * 0.5);
  float sr = sin(accelMeasurement[0] * 0.5);

  accelQuaternion[0] = cr * cp * cy + sr * sp * sy;
  accelQuaternion[1] = sr * cp * cy - cr * sp * sy;
  accelQuaternion[2] = cr * sp * cy + sr * cp * sy;
  accelQuaternion[3] = cr * cp * sy - sr * sp * cy;
}

unsigned long oldtime;
unsigned long newtime;
float dt;
float printTime;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Wire.begin();
  if (IMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false)
  {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }

  /*if (!bmp.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
    // if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    // if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");ff
    while (1)
      ;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);*/

  oldtime = micros();
  if (IMU.enableUncalibratedGyro(1000) == true)
  {
    Serial.println(F("Gyro enabled"));
  }
  else
  {
    Serial.println("Could not enable gyro");
  }
}

void setReports()
{

  if (IMU.enableUncalibratedGyro(1000) == true)
  {
    Serial.println(F("Gyro enabled"));
  }
  else
  {
    Serial.println("Could not enable gyro");
  }
  if (IMU.enableAccelerometer(1000) == true)
  {
    Serial.println(F("Accelerometer enabled"));
  }
  else
  {
    Serial.println("Could not enable accelerometer");
  }
}

void printGyroData()
{
  Serial.print("Orientation X: ");
  Serial.print(orientationEuler[0]);
  Serial.print(" Y: ");
  Serial.print(orientationEuler[1]);
  Serial.print(" Z: ");
  Serial.println(orientationEuler[2]);
  printTime = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (IMU.getSensorEvent() == true)
  {
    Serial.println("Sensor event");
    getGyroRates();
    gryoFilter();
    newtime = micros();
    dt = newtime - oldtime;
    gyroEuler(dt / 1000000);
    oldtime = newtime;

    // gyroQuanternion();
    //  Now orientationEuler contains the filtered orientation data
    updateAccelQuaternion();
  }

  if (millis() - printTime > 50)
  {
    // Serial.println(filteredGyro[0]);
    printGyroData();
    // Serial.print(filteredGyro[0]);
  }

  // Now orientationQuanternion contains the updated quaternion orientation data
  /*Serial.print("   Quaternion W: ");
  Serial.print(orientationQuanternion[0]);
  Serial.print(" X: ");
  Serial.print(orientationQuanternion[1]);
  Serial.print(" Y: ");
  Serial.print(orientationQuanternion[2]);
  Serial.print(" Z: ");
  Serial.println(orientationQuanternion[3]);*

  if (!bmp.performReading())
  {
    return;
  }

  getBaro();*/
}
