#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include <Kalman.h>
#include <math.h> // Include math library for trigonometric functions

#include "driver/uart.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_1
#define TXD_PIN (5)
#define RXD_PIN (22)
#define BUF_SIZE (1024)

using namespace BLA;

#define BNO08X_INT -1
#define BNO08X_RST -1
// not using SPI so both pins are set to -1

// #define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

BNO08x IMU;

#define SEALEVELPRESSURE_HPA (1013.25)

#define BNO08X_RESET -1

#define Nstate 3 // position, speed, acceleration
#define Nobs 2   // position, acceleration

// measurement std of the noise
#define n_p 0.5 // position measurement noise
#define n_a 5.0 // acceleration measurement noise

// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

#define gravity 9.81
#define targetAlt 250.0
#define targetTime 41.0
#define logTime 150.0 // log data every x milliseconds

#define Serialx Serial // change to serial1 for elrs.

unsigned long oldGyroTime;
unsigned long newGyroTime;
float printTime;

float gyroBias[3] = {0, 0, 0};
float oldGyroBias[3] = {0, 0, 0};

float gyroMeasurement[3] = {0, 0, 0};
float filteredGyro[3] = {0, 0, 0};
float orientationQuaternion[4] = {1, 0, 0, 0};

float baroMeaurement = 0; // baro measurement in meters adjusted by sealevel HPA

float accelMeasurement[3] = {0, 0, 0};
float correctedAccelQuaternion[4] = {0, 0, 0, 0};

float initialOrientation[4] = {0, 0, 0, 0};

bool armed = false;
bool launchDetected = false;
float launchAccelThreshold = 20.0;
float currentEstimate = 0;

// float accelQuaternion[4] = {0, 0, 0, 0};

// put function declarations here:

BLA::Matrix<Nobs> obs;
KALMAN<Nstate, Nobs> K;
unsigned long oldKalmanTime;
unsigned long newKalmanTime;

Adafruit_BMP3XX bmp;

void init_uart()
{
  const uart_config_t uart_config = {
      .baud_rate = 14400,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  // Configure UART parameters
  uart_param_config(UART_NUM, &uart_config);

  // Set UART pins
  uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Install UART driver using DMA
  uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, ESP_INTR_FLAG_IRAM);
}

void send_data(const char *data)
{
  int len = strlen(data);
  uart_write_bytes(UART_NUM, data, len);
}

// USE ONLY BEFORE LAUNCH! Called every 5 seconds, updates gyroBias with new gyroBias, and oldGyroBias with gyroBias. Called together with updateInitialOrientation. Older data is kept to prevent bad data from entering after launch.
void updateBiases()
{
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

// measures gyro rates and updates gyroMeasurement
void getGyroRates()
{
  if (IMU.getSensorEventID() == SENSOR_REPORTID_UNCALIBRATED_GYRO)
  {
    gyroMeasurement[0] = IMU.getUncalibratedGyroX();
    gyroMeasurement[1] = IMU.getUncalibratedGyroY();
    gyroMeasurement[2] = IMU.getUncalibratedGyroZ();
  }
}

// removes prerecorded bias from gyroMeasurement
void gryoFilter()
{
  filteredGyro[0] = gyroMeasurement[0] - oldGyroBias[0];
  filteredGyro[1] = gyroMeasurement[1] - oldGyroBias[1];
  filteredGyro[2] = gyroMeasurement[2] - oldGyroBias[2];
}

// updates orientationQuaternion with new orientationQuaternion via gyro integration
void gyroQuaternion()
{
  newGyroTime = micros();
  float dt = (newGyroTime - oldGyroTime) / 1000000.0;
  oldGyroTime = newGyroTime;

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

// updates baroMeasurement with new baroMeasurement from sensor
void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

//  USE ONLY BEFORE LAUNCH! Sets the initial orientation of the rocket. Must be real time, called up until ARM signal recieved.
void updateInitialOrientation()
{

  if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
  {
    initialOrientation[0] = IMU.getQuatReal();
    initialOrientation[1] = IMU.getQuatI();
    initialOrientation[2] = IMU.getQuatJ();
    initialOrientation[3] = IMU.getQuatK();
  }

  for (int i = 0; i < 4; i++)
  {
    orientationQuaternion[i] = initialOrientation[i];
  }
}

// gets corrected accel quaternion rotated to be in world reference frame (gravity is removed)
void getAccelQuaternion()
{
  if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
  {
    accelMeasurement[0] = IMU.getAccelX();
    accelMeasurement[1] = IMU.getAccelY();
    accelMeasurement[2] = IMU.getAccelZ();
  }

  float uncorrectedAccelQuaternion[4] = {0, 0, 0, 0};
  uncorrectedAccelQuaternion[0] = 0;
  uncorrectedAccelQuaternion[1] = accelMeasurement[0];
  uncorrectedAccelQuaternion[2] = accelMeasurement[1];
  uncorrectedAccelQuaternion[3] = accelMeasurement[2];

  float orientationTimesAccel[4] = {0, 0, 0, 0};
  orientationTimesAccel[0] = orientationQuaternion[0] * uncorrectedAccelQuaternion[0] - orientationQuaternion[1] * uncorrectedAccelQuaternion[1] - orientationQuaternion[2] * uncorrectedAccelQuaternion[2] - orientationQuaternion[3] * uncorrectedAccelQuaternion[3];
  orientationTimesAccel[1] = orientationQuaternion[0] * uncorrectedAccelQuaternion[1] + orientationQuaternion[1] * uncorrectedAccelQuaternion[0] + orientationQuaternion[2] * uncorrectedAccelQuaternion[3] - orientationQuaternion[3] * uncorrectedAccelQuaternion[2];
  orientationTimesAccel[2] = orientationQuaternion[0] * uncorrectedAccelQuaternion[2] - orientationQuaternion[1] * uncorrectedAccelQuaternion[3] + orientationQuaternion[2] * uncorrectedAccelQuaternion[0] + orientationQuaternion[3] * uncorrectedAccelQuaternion[1];
  orientationTimesAccel[3] = orientationQuaternion[0] * uncorrectedAccelQuaternion[3] + orientationQuaternion[1] * uncorrectedAccelQuaternion[2] - orientationQuaternion[2] * uncorrectedAccelQuaternion[1] + orientationQuaternion[3] * uncorrectedAccelQuaternion[0];

  float inverseOrientation[4] = {0, 0, 0, 0};
  float orientationMag = sqrt(orientationQuaternion[0] * orientationQuaternion[0] + orientationQuaternion[1] * orientationQuaternion[1] + orientationQuaternion[2] * orientationQuaternion[2] + orientationQuaternion[3] * orientationQuaternion[3]);
  inverseOrientation[0] = orientationQuaternion[0] / (orientationMag * orientationMag);
  inverseOrientation[1] = -orientationQuaternion[1] / (orientationMag * orientationMag);
  inverseOrientation[2] = -orientationQuaternion[2] / (orientationMag * orientationMag);
  inverseOrientation[3] = -orientationQuaternion[3] / (orientationMag * orientationMag);

  correctedAccelQuaternion[0] = orientationTimesAccel[0] * inverseOrientation[0] - orientationTimesAccel[1] * inverseOrientation[1] - orientationTimesAccel[2] * inverseOrientation[2] - orientationTimesAccel[3] * inverseOrientation[3];
  correctedAccelQuaternion[1] = orientationTimesAccel[0] * inverseOrientation[1] + orientationTimesAccel[1] * inverseOrientation[0] + orientationTimesAccel[2] * inverseOrientation[3] - orientationTimesAccel[3] * inverseOrientation[2];
  correctedAccelQuaternion[2] = orientationTimesAccel[0] * inverseOrientation[2] - orientationTimesAccel[1] * inverseOrientation[3] + orientationTimesAccel[2] * inverseOrientation[0] + orientationTimesAccel[3] * inverseOrientation[1];
  correctedAccelQuaternion[3] = orientationTimesAccel[0] * inverseOrientation[3] + orientationTimesAccel[1] * inverseOrientation[2] - orientationTimesAccel[2] * inverseOrientation[1] + orientationTimesAccel[3] * inverseOrientation[0];

  // correct for gravity
  correctedAccelQuaternion[3] -= gravity;
}

// sets the reports that the IMU should send
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

// debugging, disable for flight
void printData()
{
  // Serial.print("Quaternion: ");
  // Serial.print(orientationQuaternion[0]);
  // Serial.print(", ");
  // Serial.print(orientationQuaternion[1]);
  // Serial.print(", ");
  // Serial.print(orientationQuaternion[2]);
  // Serial.print(", ");
  // Serial.println(orientationQuaternion[3]);

  Serial.print("Quaternion: ");
  Serial.print(correctedAccelQuaternion[0]);
  Serial.print(", ");
  Serial.print(correctedAccelQuaternion[1]);
  Serial.print(", ");
  Serial.print(correctedAccelQuaternion[2]);
  Serial.print(", ");
  Serial.println(correctedAccelQuaternion[3]);
}

// sets up kalman filter model
void setupKalman()
{

  // time evolution matrix (whatever... it will be updated inloop)
  K.F = {1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  // measurement matrix n the position (e.g. GPS) and acceleration (e.g. accelerometer)
  K.H = {1.0, 0.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_p * n_p, 0.0,
         0.0, n_a * n_a};
  // model covariance matrix
  K.Q = {m_p * m_p, 0.0, 0.0,
         0.0, m_s * m_s, 0.0,
         0.0, 0.0, m_a * m_a};
}

// updates kalman filter model with new observation
void updateKalman()
{
  newKalmanTime = micros();
  float dkt = (newKalmanTime - oldKalmanTime) / 1000000.0;
  oldKalmanTime = newKalmanTime;

  K.F = {1.0, dkt, 0.5 * dkt * dkt,
         0.0, 1.0, dkt,
         0.0, 0.0, 1.0};

  getBaro();
  obs(0) = baroMeaurement;
  obs(1) = correctedAccelQuaternion[3];

  K.update(obs);
}

// run servos based on kalman state
void runServos()
{
  // K.x is kalman state. (0) is position, (1) is speed, (2) is acceleration

  currentEstimate = -((K.x(1) * K.x(1)) / (K.x(2) - gravity)) + K.x(0);
  if (currentEstimate > targetAlt)
  {
    Serial.println("Extend Servos ");
  }
  else
  {
    Serial.println("Retract Servos ");
  }
}

// updates launchDetected based on raw accelerometer
void detectLaunch()
{
  float accelMag = sqrt(accelMeasurement[0] * accelMeasurement[0] + accelMeasurement[1] * accelMeasurement[1] + accelMeasurement[2] * accelMeasurement[2]);
  if (accelMag > launchAccelThreshold && armed)
  {
    launchDetected = true;
  }
}

// in flight communications. Update to serial1
void serialComms()
{
  if (Serial.available() > 0)
  {
    char inChar = Serial.read();
    if (inChar == 'a')
    {
      armed = true;
    }
    if (inChar == 'd')
    {
      armed = false;
    }
  }
  if (armed)
  {
    Serialx.print(K.x(0));
    Serialx.print(",");
    Serialx.print(K.x(1));
    Serialx.print(",");
    Serialx.print(K.x(2));
    Serialx.print(",");
    Serialx.print(currentEstimate);
    Serialx.print(",");
    Serialx.print(baroMeaurement);
    Serialx.print(",");
    Serialx.print(correctedAccelQuaternion[3]);
    Serialx.println();
  }
  else
  {
    launchDetected = false;
    Serial.println("Not armed!");
  }
}

void setup()
{
  // put your setup code here, to run once:
  // pinMode(13, INPUT_PULLDOWN);
  Serial.begin(115200);
  Serial1.begin(14400);

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
  setupKalman();

  for (int i = 0; i < 1000; i++)
  {
    if (IMU.getSensorEvent() == true)
    {
      updateInitialOrientation();
    }
  }
  oldGyroTime = micros();
  oldKalmanTime = micros();
}

void loop()
{
  if (IMU.getSensorEvent() == true)
  {
    // Serial.println("Sensor event");
    getGyroRates();
    gryoFilter();
    gyroQuaternion();
    getAccelQuaternion();
  }

  if (millis() - printTime > logTime)
  {
    printData();
  }

  /*if (bmp.performReading())
  {
    getBaro();
  }

  */
}
