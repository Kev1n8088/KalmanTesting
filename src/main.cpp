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
float orientationQuaternion[4] = {1, 0, 0, 0};

float baroMeaurement = 0; // baro measurement in meters adjusted by sealevel HPA

float accelMeasurement[3] = {0, 0, 0};

float initialOrientation[4] = {0, 0, 0, 0};

// float accelQuaternion[4] = {0, 0, 0, 0};

// put function declarations here:

#define BNO08X_RESET -1

Adafruit_BMP3XX bmp;
void updateBiases()
{
  // called every 5 seconds, updates gyroBias with new gyroBias, and oldGyroBias with gyroBias. Always use oldGyroBias for launch.
  if (IMU.getSensorEventID() == SENSOR_REPORTID_UNCALIBRATED_GYRO)
  {
    for (int i = 0; i < 3; i++)
    {
      oldGyroBias[i] = gyroBias[i];
    }
    gyroBias[0] = IMU.getUncalibratedGyroX();
    gyroBias[1] = IMU.getUncalibratedGyroY();
    gyroBias[2] = IMU.getUncalibratedGyroZ();
  }
}

void getGyroRates()
{
  if (IMU.getSensorEventID() == SENSOR_REPORTID_UNCALIBRATED_GYRO)
  {
    gyroMeasurement[0] = IMU.getUncalibratedGyroX();
    gyroMeasurement[1] = IMU.getUncalibratedGyroY();
    gyroMeasurement[2] = IMU.getUncalibratedGyroZ();
  }
}

void gryoFilter()
{
  filteredGyro[0] = gyroMeasurement[0] - oldGyroBias[0];
  filteredGyro[1] = gyroMeasurement[1] - oldGyroBias[1];
  filteredGyro[2] = gyroMeasurement[2] - oldGyroBias[2];
}

void gyroQuaternion(float dt)
{
  float dq[4] = {0, 0, 0, 0};
  float v[3] = {0, 0, 0};
  float gyroMag = sqrt(filteredGyro[0] * filteredGyro[0] + filteredGyro[1] * filteredGyro[1] + filteredGyro[2] * filteredGyro[2]);

  if (gyroMag == 0)
  {
    return;
  }

  float theta = gyroMag * dt;
  v[0] = filteredGyro[0] / gyroMag;
  v[1] = filteredGyro[1] / gyroMag;
  v[2] = filteredGyro[2] / gyroMag;
  dq[0] = cos(0.5 * theta);
  dq[1] = sin(0.5 * theta) * v[0];
  dq[2] = sin(0.5 * theta) * v[1];
  dq[3] = sin(0.5 * theta) * v[2];

  orientationQuaternion[0] = orientationQuaternion[0] * dq[0] - orientationQuaternion[1] * dq[1] - orientationQuaternion[2] * dq[2] - orientationQuaternion[3] * dq[3];
  orientationQuaternion[1] = orientationQuaternion[0] * dq[1] + orientationQuaternion[1] * dq[0] + orientationQuaternion[2] * dq[3] - orientationQuaternion[3] * dq[2];
  orientationQuaternion[2] = orientationQuaternion[0] * dq[2] - orientationQuaternion[1] * dq[3] + orientationQuaternion[2] * dq[0] + orientationQuaternion[3] * dq[1];
  orientationQuaternion[3] = orientationQuaternion[0] * dq[3] + orientationQuaternion[1] * dq[2] - orientationQuaternion[2] * dq[1] + orientationQuaternion[3] * dq[0];

  float mag = sqrt(orientationQuaternion[0] * orientationQuaternion[0] + orientationQuaternion[1] * orientationQuaternion[1] + orientationQuaternion[2] * orientationQuaternion[2] + orientationQuaternion[3] * orientationQuaternion[3]);
  for (int i = 0; i < 4; i++)
  {
    orientationQuaternion[i] = orientationQuaternion[i] / mag;
  }
}

void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void updateInitialOrientation()
{
  // USE ONLY BEFORE LAUNCH! This function is only accurate when the rocket is on the ground. It sets the initial orientation of the rocket.
  for (int i = 0; i < 4; i++)
  {
    orientationQuaternion[i] = initialOrientation[i];
  }

  if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
  {
    initialOrientation[0] = IMU.getQuatReal();
    initialOrientation[1] = IMU.getQuatI();
    initialOrientation[2] = IMU.getQuatJ();
    initialOrientation[3] = IMU.getQuatK();
  }
}

// void getAccelQuaternion()
// {
//   if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
//   {
//     accelMeasurement[0] = IMU.getAccelX();
//     accelMeasurement[1] = IMU.getAccelY();
//     accelMeasurement[2] = IMU.getAccelZ();

//     float cy = cos(accelMeasurement[2] * 0.5);
//     float sy = sin(accelMeasurement[2] * 0.5);
//     float cp = cos(accelMeasurement[1] * 0.5);
//     float sp = sin(accelMeasurement[1] * 0.5);
//     float cr = cos(accelMeasurement[0] * 0.5);
//     float sr = sin(accelMeasurement[0] * 0.5);

//     accelQuaternion[0] = cr * cp * cy + sr * sp * sy;
//     accelQuaternion[1] = sr * cp * cy - cr * sp * sy;
//     accelQuaternion[2] = cr * sp * cy + sr * cp * sy;
//     accelQuaternion[3] = cr * cp * sy - sr * sp * cy;
//   }
// }

unsigned long oldtime;
unsigned long newtime;
float dt;
float printTime;

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
  if (IMU.enableRotationVector() == true)
  {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  }
  else
  {
    Serial.println("Could not enable rotation vector");
  }
  // if (IMU.enableGravity() == true)
  // {
  //   Serial.println(F("Gravity enabled"));
  //   Serial.println(F("Output in form x, y, z, accuracy"));
  // }
  // else
  // {
  //   Serial.println("Could not enable gravity");
  // }
}

void printData()
{
  Serial.print("Quaternion: ");
  Serial.print(orientationQuaternion[0]);
  Serial.print(", ");
  Serial.print(orientationQuaternion[1]);
  Serial.print(", ");
  Serial.print(orientationQuaternion[2]);
  Serial.print(", ");
  Serial.println(orientationQuaternion[3]);
}

void printAccelData()
{
}

void setup()
{
  // put your setup code here, to run once:
  // pinMode(13, INPUT_PULLDOWN);
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

  setReports();

  for (int i = 0; i < 1000; i++)
  {
    if (IMU.getSensorEvent() == true)
    {
      updateInitialOrientation();
    }
  }
  oldtime = micros();
}

void loop()
{
  if (IMU.getSensorEvent() == true)
  {
    // Serial.println("Sensor event");
    getGyroRates();
    gryoFilter();
    newtime = micros();
    dt = newtime - oldtime;
    // gyroEuler(dt / 1000000);
    gyroQuaternion(dt / 1000000);
    oldtime = newtime;
    // updateGravityQuaternion();
  }

  if (millis() - printTime > 100)
  {
    printData();
  }

  /*if (!bmp.performReading())
  {
    return;
  }

  getBaro();
  */
}
