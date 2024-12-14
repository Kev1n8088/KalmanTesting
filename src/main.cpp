#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include <Kalman.h>
#include <math.h> // Include math library for trigonometric functions
#include <ESP32Servo.h> // Include servo library for controlling servo

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

#define BMP3XX_ADDR 0x76 

BNO08x IMU;

#define SEALEVELPRESSURE_HPA (1013.25)

#define BNO08X_RESET -1

#define Nstate 3 // position, speed, acceleration
#define Nobs 2   // position, acceleration

// measurement std of the noise
#define n_p 0.5 // position measurement noise
#define n_a 3.0 // acceleration measurement noise

// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

#define GRAVITY 9.81 // gravity in m/s^2
#define GRAVITY_OFFSET 0.0 // compensates for noise
#define TARGET_ALT 250.0 // target altitude in meters
#define TARGET_TIME 41.0 // target time in seconds

#define LOG_TIME 150.0   // log data every x milliseconds
#define TARE_TIME 5000.0 // tare gyro every x milliseconds

#define Serialx Serial // change to serial1 for elrs.

#define BARO_ADJUST_N 10 // number of baro values to average

#define APOGEE_DETECT_ALT_DOWN 2.0 //how much below max alt to detect apogee passed
#define APOGEE_DETECT_VEL_DOWN -1.0 //how much negative velocity to detect apogee passed

#define SERVO_PIN A1 // servo pin
#define SERVO_MIN 1000 // servo min pulse width
#define SERVO_MAX 2000 // servo max pulse width

#define SERVO_RETRACT 20 // servo retract position
#define SERVO_DEPLOY 50 // servo deploy position
#define SERVO_STEP 1 // servo step size

#define BAT_PIN A3 // battery voltage pin
#define MAX_VOLT 10.321 // voltage divider max voltage, analog in maxes out at 4095 so map to that

unsigned long oldGyroTime; // old gyro time, used for gyro integration
unsigned long newGyroTime; // new gyro time, used for gyro integration
unsigned long printTime; // time of last printing for debugging
unsigned long biasTime; // time of last added gyro biases

float gyroBias[3] = {0.0, 0.0, 0.0}; //stored gyro biases, stored before launch
float oldGyroBias[3] = {0.0, 0, 0.0}; //actual used gyro biases to ensure no bad data enters after launch

float gyroMeasurement[3] = {0.0, 0.0, 0.0}; // raw gyro measurements
float filteredGyro[3] = {0.0, 0.0, 0.0}; // filtered gyro measurements adjusted by bias
float orientationQuaternion[4] = {1.0, 0.0, 0.0, 0.0}; // orientation quaternion

float baroMeaurement = 0.0; // baro measurement in meters adjusted by sealevel HPA
float baroAdjust = 0.0;     // baro adjustment in meters

int baroCurrent = 0; // current baro index
int baroCount = 0; // current baro count
float baroSum = 0; // current baro sum
float values[BARO_ADJUST_N]; // array of baro values

float accelMeasurement[3] = {0.0, 0.0, 0.0}; // raw accel measurements
float correctedAccelQuaternion[4] = {0.0, 0.0, 0.0, 0.0}; // corrected accel quaternion with gravity removed

float initialOrientation[4] = {0.0, 0.0, 0.0, 0.0}; // initial orientation quaternion

bool armed = false; // rocket is armed
bool pastApogee = false; // rocket has passed apogee
bool launchDetected = false; // launch has been detected
float launchAccelThreshold = 15.0; // acceleration threshold to detect launch in m/s^2
float currentEstimate = 0.0; // current estimate of apogee
float launchTime = -1.0; // time of launch

float maxAlt = 0.0; // maximum altitude

Servo servo; // servo object
int servoPos = SERVO_RETRACT; // servo position

// float accelQuaternion[4] = {0, 0, 0, 0};

// put function declarations here:

BLA::Matrix<Nobs> obs; // observation matrix
KALMAN<Nstate, Nobs> K; // kalman filter matrix
unsigned long oldKalmanTime; // old kalman time, used for kalman filtering
unsigned long newKalmanTime; // new kalman time, used for kalman filtering

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
void gyroFilter()
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

  float dq[4] = {0.0, 0.0, 0.0, 0.0};
  float v[3] = {0.0, 0.0, 0.0};
  float gyroMag = sqrt(filteredGyro[0] * filteredGyro[0] + filteredGyro[1] * filteredGyro[1] + filteredGyro[2] * filteredGyro[2]);

  if (gyroMag == 0.0)
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
  if (mag == 0.0)
  {
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    orientationQuaternion[i] = orientationQuaternion[i] / mag;
  }
}

// updates baroMeasurement with new baroMeasurement from sensor
void getBaro()
{
  baroMeaurement = bmp.readAltitude(SEALEVELPRESSURE_HPA) - baroAdjust;
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
    for (int i = 0; i < 4; i++)
    {
      orientationQuaternion[i] = initialOrientation[i];
    }
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
  if (orientationMag == 0.0)
  {
    return;
  }
  inverseOrientation[0] = orientationQuaternion[0] / (orientationMag * orientationMag);
  inverseOrientation[1] = -orientationQuaternion[1] / (orientationMag * orientationMag);
  inverseOrientation[2] = -orientationQuaternion[2] / (orientationMag * orientationMag);
  inverseOrientation[3] = -orientationQuaternion[3] / (orientationMag * orientationMag);

  correctedAccelQuaternion[0] = orientationTimesAccel[0] * inverseOrientation[0] - orientationTimesAccel[1] * inverseOrientation[1] - orientationTimesAccel[2] * inverseOrientation[2] - orientationTimesAccel[3] * inverseOrientation[3];
  correctedAccelQuaternion[1] = orientationTimesAccel[0] * inverseOrientation[1] + orientationTimesAccel[1] * inverseOrientation[0] + orientationTimesAccel[2] * inverseOrientation[3] - orientationTimesAccel[3] * inverseOrientation[2];
  correctedAccelQuaternion[2] = orientationTimesAccel[0] * inverseOrientation[2] - orientationTimesAccel[1] * inverseOrientation[3] + orientationTimesAccel[2] * inverseOrientation[0] + orientationTimesAccel[3] * inverseOrientation[1];
  correctedAccelQuaternion[3] = orientationTimesAccel[0] * inverseOrientation[3] + orientationTimesAccel[1] * inverseOrientation[2] - orientationTimesAccel[2] * inverseOrientation[1] + orientationTimesAccel[3] * inverseOrientation[0];

  // correct for gravity
  correctedAccelQuaternion[3] -= GRAVITY;
  correctedAccelQuaternion[3] += GRAVITY_OFFSET;
}

// gets baro adjustment for altitude, using a rolling average
void getBaroAdjustment(){
  if(!armed){
    float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    baroSum += reading;
    if (baroCount == BARO_ADJUST_N){
      baroSum -= values[baroCurrent];
    }

    values[baroCurrent] = reading;
    
    if (++baroCurrent >= BARO_ADJUST_N){
      baroCurrent = 0;
    }

    if (baroCount < BARO_ADJUST_N){
      baroCount++;
    }

    baroAdjust = baroSum / baroCount;

  }
}

// sets the reports that the IMU should send
void setReports(bool enableRotationVector)
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
  if (enableRotationVector)
  {
    if (IMU.enableRotationVector() == true)
    {
      Serial.println(F("Rotation vector enabled"));
      Serial.println(F("Output in form i, j, k, real, accuracy"));
    }
    else
    {
      Serial.println("Could not enable rotation vector");
    }
  }
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

  // Serial.print("Gyro: ");
  // Serial.print(orientationQuaternion[0]);
  // Serial.print(", ");
  // Serial.print(orientationQuaternion[1]);
  // Serial.print(", ");
  // Serial.print(orientationQuaternion[2]);
  // Serial.print(", ");
  // Serial.println(orientationQuaternion[3]);

  // Serial.print("Quaternion: ");
  // Serial.print(correctedAccelQuaternion[0]);
  // Serial.print(", ");
  // Serial.print(correctedAccelQuaternion[1]);
  // Serial.print(", ");
  // Serial.print(correctedAccelQuaternion[2]);
  // Serial.print(", ");
  // Serial.println(correctedAccelQuaternion[3]);

  Serial.println(baroMeaurement);
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
  currentEstimate = -((K.x(1) * K.x(1)) / (K.x(2) - GRAVITY)) + K.x(0);
  if(launchDetected){
    if (!pastApogee)
    {
      if (currentEstimate > TARGET_ALT)
      {
        servoPos = constrain(servoPos + SERVO_STEP, SERVO_RETRACT, SERVO_DEPLOY);
        //Serial.println("Extend Servos ");
      }
      else
      {
        servoPos = constrain(servoPos - SERVO_STEP, SERVO_RETRACT, SERVO_DEPLOY);
        //Serial.println("Retract Servos ");
      }
    }
    else
    {
      float landEstimate = (-K.x(0) / K.x(1)) + millis();
      float landTarget = TARGET_TIME * 1000.0 + launchTime;

      if (landEstimate > landTarget)
      {
        servoPos = constrain(servoPos - SERVO_STEP, SERVO_RETRACT, SERVO_DEPLOY);
      }
      else
      {
        servoPos = constrain(servoPos + SERVO_STEP, SERVO_RETRACT, SERVO_DEPLOY);
      }
    }

    servo.write(servoPos);
  }
}

// updates launchDetected based on raw accelerometer
void detectLaunch()
{
  float accelMag = sqrt(accelMeasurement[0] * accelMeasurement[0] + accelMeasurement[1] * accelMeasurement[1] + accelMeasurement[2] * accelMeasurement[2]);
  if (accelMag > launchAccelThreshold && armed)
  {
    launchDetected = true;
    if (launchTime == -1.0)
    {
      launchTime = millis();
    }
  }
}

void detectApogee(){
  if (maxAlt < K.x(0)){
    maxAlt = K.x(0);
  }

  if (maxAlt - K.x(0) > APOGEE_DETECT_ALT_DOWN && K.x(1) < APOGEE_DETECT_VEL_DOWN){
    pastApogee = true;
  }

}


void resetVariables()
{
  for (int i = 0; i < 3; i++)
  {
    gyroBias[i] = 0.0;
    oldGyroBias[i] = 0.0;
    gyroMeasurement[i] = 0.0;
    filteredGyro[i] = 0.0;
    accelMeasurement[i] = 0.0;
  }
  for (int i = 0; i < 4; i++)
  {
    orientationQuaternion[i] = 0.0;
    correctedAccelQuaternion[i] = 0.0;
    initialOrientation[i] = 0.0;
  }
  orientationQuaternion[0] = 1.0;
  initialOrientation[0] = 1.0;

  baroMeaurement = 0.0; // baro measurement in meters adjusted by sealevel HPA
}

void initAtArm()
{
  resetVariables();
  IMU.softReset();
  delay(1000);
  setReports(true);
  for (int i = 0; i < 100; i++)
  {
    if (IMU.getSensorEvent() == true)
    {
      updateInitialOrientation();
    }
    delay(10);
  }
  IMU.softReset();
  delay(1000);
  setReports(false);
  for (int i = 0; i < 100; i++)
  {
    if (IMU.getSensorEvent() == true)
    {
      updateBiases();
    }
    delay(10);
  }
  setupKalman();
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
      initAtArm();
      oldGyroTime = micros();
      oldKalmanTime = micros();
    }
    if (inChar == 'd')
    {
      armed = false;
    }
  }
  if (armed)
  {
    // Serialx.print(K.x(0));
    // Serialx.print(",");
    // Serialx.print(K.x(1));
    // Serialx.print(",");
    // Serialx.print(K.x(2));
    // Serialx.print(",");
    // Serialx.print(currentEstimate);
    // Serialx.print(",");
    // Serialx.print(baroMeaurement);
    // Serialx.print(",");
    // Serialx.print(correctedAccelQuaternion[3]);
    // Serialx.println();
    printData();
  }
  else
  {
    launchDetected = false;
    launchTime = -1.0;
    pastApogee = false;
    Serial.println("Not armed!");
  }
}

float getBatteryVoltage()
{
  int raw = analogRead(BAT_PIN);
  return (raw / 4095.0) * MAX_VOLT;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(14400);

  pinMode(BAT_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo.setPeriodHertz(50);    // standard 50 hz servo
	servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX); // attaches the servo on pin 18 to the servo object

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Wire.begin();
  Wire1.begin();
  if (IMU.begin(BNO08X_ADDR, Wire1, BNO08X_INT, BNO08X_RST) == false)
  {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }

  if (!bmp.begin_I2C(BMP3XX_ADDR))
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
  if (millis() - printTime > LOG_TIME)
  {
    serialComms();
    printTime = millis();
  }
  if (armed)
  {

    if (bmp.performReading())
    {
      getBaro();
    }

    if (IMU.getSensorEvent() == true)
    {
      getGyroRates();
      gyroFilter();
      gyroQuaternion();
      getAccelQuaternion();
      updateKalman();
      detectLaunch();
    }

    if (!launchDetected)
    {
      if (millis() - biasTime > TARE_TIME)
      {

        IMU.getSensorEvent();
        for (int i = 0; i < 500; i++)
        {
          if (IMU.getSensorEvent() == true)
          {
            updateBiases();
            biasTime = millis();
            break;
          }
          delay(1);
        }
      }else{
        runServos();
        detectApogee();
      }
    }
  }else{
    if (bmp.performReading())
    {
    getBaroAdjustment();
    }
  }
}
