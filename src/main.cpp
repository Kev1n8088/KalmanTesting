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
float orientationQuanternion[4] = {1, 0, 0, 0};

float baroMeaurement = 0; // baro measurement in meters adjusted by sealevel HPA

float accelMeasurement[3] = {0, 0, 0};
float oldGravQuaternion[4] = {0, 0, 0, 0};
float gravQuaternion[4] = {0, 0, 0, 0};
float accelQuaternion[4] = {0, 0, 0, 0};

float orientationTimesAccel[4] = {0, 0, 0, 0};
float conjugateOrientationQuanternion[4] = {0, 0, 0, 0};
float worldQuanternion[4] = {0, 0, 0, 0};

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

void gyroQuanternion(float dt)
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

  orientationQuanternion[0] = orientationQuanternion[0] * dq[0] - orientationQuanternion[1] * dq[1] - orientationQuanternion[2] * dq[2] - orientationQuanternion[3] * dq[3];
  orientationQuanternion[1] = orientationQuanternion[0] * dq[1] + orientationQuanternion[1] * dq[0] + orientationQuanternion[2] * dq[3] - orientationQuanternion[3] * dq[2];
  orientationQuanternion[2] = orientationQuanternion[0] * dq[2] - orientationQuanternion[1] * dq[3] + orientationQuanternion[2] * dq[0] + orientationQuanternion[3] * dq[1];
  orientationQuanternion[3] = orientationQuanternion[0] * dq[3] + orientationQuanternion[1] * dq[2] - orientationQuanternion[2] * dq[1] + orientationQuanternion[3] * dq[0];
}

void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void getAccelQuaternion()
{
  if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
  {
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
}

void updateGravityQuaternion()
{

  if (IMU.getSensorEventID() == SENSOR_REPORTID_GRAVITY)
  {
    for (int i = 0; i < 4; i++)
    {
      oldGravQuaternion[i] = gravQuaternion[i];
    }
    gravQuaternion[0] = 0;
    gravQuaternion[1] = IMU.getGravityX();
    gravQuaternion[2] = IMU.getGravityY();
    gravQuaternion[3] = IMU.getGravityZ();

    float mag = sqrt(gravQuaternion[1] * gravQuaternion[1] + gravQuaternion[2] * gravQuaternion[2] + gravQuaternion[3] * gravQuaternion[3]);
    for (int i = 0; i < 4; i++)
    {
      gravQuaternion[i] = gravQuaternion[i] / mag;
    }
  }
}

void getWorldQuanternion()
{
  // get the world quanternion from the accelQuanternion and the gyroQuanternion
  orientationTimesAccel[0] = orientationQuanternion[0] * oldGravQuaternion[0] - orientationQuanternion[1] * oldGravQuaternion[1] - orientationQuanternion[2] * oldGravQuaternion[2] - orientationQuanternion[3] * oldGravQuaternion[3];
  orientationTimesAccel[1] = orientationQuanternion[0] * oldGravQuaternion[1] + orientationQuanternion[1] * oldGravQuaternion[0] + orientationQuanternion[2] * oldGravQuaternion[3] - orientationQuanternion[3] * oldGravQuaternion[2];
  orientationTimesAccel[2] = orientationQuanternion[0] * oldGravQuaternion[2] - orientationQuanternion[1] * oldGravQuaternion[3] + orientationQuanternion[2] * oldGravQuaternion[0] + orientationQuanternion[3] * oldGravQuaternion[1];
  orientationTimesAccel[3] = orientationQuanternion[0] * oldGravQuaternion[3] + orientationQuanternion[1] * oldGravQuaternion[2] - orientationQuanternion[2] * oldGravQuaternion[1] + orientationQuanternion[3] * oldGravQuaternion[0];

  conjugateOrientationQuanternion[0] = orientationQuanternion[0];
  conjugateOrientationQuanternion[1] = -orientationQuanternion[1];
  conjugateOrientationQuanternion[2] = -orientationQuanternion[2];
  conjugateOrientationQuanternion[3] = -orientationQuanternion[3];

  worldQuanternion[0] = orientationTimesAccel[0] * conjugateOrientationQuanternion[0] - orientationTimesAccel[1] * conjugateOrientationQuanternion[1] - orientationTimesAccel[2] * conjugateOrientationQuanternion[2] - orientationTimesAccel[3] * conjugateOrientationQuanternion[3];
  worldQuanternion[1] = orientationTimesAccel[0] * conjugateOrientationQuanternion[1] + orientationTimesAccel[1] * conjugateOrientationQuanternion[0] + orientationTimesAccel[2] * conjugateOrientationQuanternion[3] - orientationTimesAccel[3] * conjugateOrientationQuanternion[2];
  worldQuanternion[2] = orientationTimesAccel[0] * conjugateOrientationQuanternion[2] - orientationTimesAccel[1] * conjugateOrientationQuanternion[3] + orientationTimesAccel[2] * conjugateOrientationQuanternion[0] + orientationTimesAccel[3] * conjugateOrientationQuanternion[1];
  worldQuanternion[3] = orientationTimesAccel[0] * conjugateOrientationQuanternion[3] + orientationTimesAccel[1] * conjugateOrientationQuanternion[2] - orientationTimesAccel[2] * conjugateOrientationQuanternion[1] + orientationTimesAccel[3] * conjugateOrientationQuanternion[0];
}

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
  if (IMU.enableGravity() == true)
  {
    Serial.println(F("Gravity enabled"));
    Serial.println(F("Output in form x, y, z, accuracy"));
  }
  else
  {
    Serial.println("Could not enable gravity");
  }
}

void printData()
{
  // Serial.print("Quaternion: ");
  // Serial.print(orientationQuanternion[0]);
  // Serial.print(", ");
  // Serial.print(orientationQuanternion[1]);
  // Serial.print(", ");
  // Serial.print(orientationQuanternion[2]);
  // Serial.print(", ");
  // Serial.println(orientationQuanternion[3]);
  /*printTime = millis();*/

  // Serial.print("W: ");
  // Serial.print(oldGravQuaternion[0]);
  // Serial.print(" X: ");
  // Serial.print(oldGravQuaternion[1]);
  // Serial.print(" Y: ");
  // Serial.print(oldGravQuaternion[2]);
  // Serial.print(" Z: ");
  // Serial.println(oldGravQuaternion[3]);

  Serial.print("Quaternion: ");
  Serial.print(worldQuanternion[0]);
  Serial.print(", ");
  Serial.print(worldQuanternion[1]);
  Serial.print(", ");
  Serial.print(worldQuanternion[2]);
  Serial.print(", ");
  Serial.println(worldQuanternion[3]);
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
  // while (IMU.getSensorEvent() == false)
  // {
  //   delay(5);
  // }
  // while (!IMU.getSensorEventID() == SENSOR_REPORTID_GRAVITY)
  // {
  //   Serial.println("awaiting gravity");
  //   delay(5);
  // }
  // updateGravityQuaternion();
  // delay(5000);
  // while (IMU.getSensorEvent() == false)
  // {
  //   delay(5);
  // }
  // while (!IMU.getSensorEventID() == SENSOR_REPORTID_GRAVITY)
  // {
  //   delay(5);
  // }
  // updateGravityQuaternion();
  // printData();

  for (int i = 0; i < 1000; i++)
  {
    if (IMU.getSensorEvent() == true)
    {
      updateGravityQuaternion();
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
    gyroQuanternion(dt / 1000000);
    oldtime = newtime;
    // updateGravityQuaternion();
    getWorldQuanternion();
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
