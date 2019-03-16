#include <QTRSensors.h>

// Sensors:
#define NUM_SENSORS 5                  // number of sensors used
#define TIMEOUT 2500                   // waits for X microseconds for sensor outputs to go low
#define EMITTER_PIN QTR_NO_EMITTER_PIN // no emitter

const int S2 = 10; // Sensor 2 pin
const int S3 = 11; // Sensor 3 pin
const int S4 = 12; // Sensor 4 pin
const int S5 = 2;  // Sensor 5 pin
const int S6 = 8;  // Sensor 6 pin

// sensors 0 through 4 are connected to digital pins 2 through 6, respectively
QTRSensorsRC qtrrc((unsigned char[]){S6, S5, S4, S3, S2},
                   NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motors:
const int PWMA = 3;
const int AIN1 = 5;
const int AIN2 = 4;

const int PWMB = 9;
const int BIN1 = 6;
const int BIN2 = 7;

const int baseSpeed = 105;

const int maxSpeed = 255;
const int minSpeed = 0;
const int speedDiff = maxSpeed - minSpeed;

float prevError = 0;

void setup()
{

  delay(500);

  // Arduino led:
  pinMode(13, OUTPUT);

  // All motor pins should be outputs:
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  const int cS = 50;

  // Calibration start
  for (int i = 0; i < 150; i++)
  {
    qtrrc.calibrate();

    qtrrc.read(sensorValues);

    if (i == 0)
    { // Spin left
      rightWheel(cS);
      leftWheel(-1 * cS);
    }
    else if (sensorValues[4] > 800)
    { // Spin right
      rightWheel(-1 * cS);
      leftWheel(cS);
    }
    else if (sensorValues[0] > 800)
    { // Spin left
      rightWheel(cS);
      leftWheel(-1 * cS);
    }
  }

  rightWheel(0);
  leftWheel(0);

  delay(1000);

  int position = qtrrc.readLine(sensorValues);

  // Calibration is done, recenter the robot
  while (!(position > 1700 && position < 2300))
  {
    if (position < 2000)
    {
      // Spin left
      rightWheel(cS);
      leftWheel(-1 * cS);
    }
    else if (position > 2000)
    {
      // Spin left
      rightWheel(-1 * cS);
      leftWheel(cS);
    }
    position = qtrrc.readLine(sensorValues);
  }

  rightWheel(0);
  leftWheel(0);

  digitalWrite(13, LOW);

  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 4000
  unsigned int position = qtrrc.readLine(sensorValues);
  int error = 2000 - position; // 2000 == max error | 0 == no error

  if (error < 0)
    error = error * -1;

  const float errorPercent = error / 2000.0;
  const float errorDif = errorPercent - prevError;

  if (position >= 1950 || position <= 2050)
  {
    rightWheel(baseSpeed);
    leftWheel(baseSpeed);
  }
  else if (position < 2000)
  { // Spin left
    rightWheel(motorSpeed(1, errorPercent, errorDif));
    leftWheel(motorSpeed(0, errorPercent, errorDif));
  }
  else if (position > 2000)
  { // Spin right
    rightWheel(motorSpeed(0, errorPercent, errorDif));
    leftWheel(motorSpeed(1, errorPercent, errorDif));
  }

  prevError = errorPercent;
}

// opt == 0 (slow) / 1 (fast), eP = errorPercent, eD = errorDif
int motorSpeed(int opt, float eP, float eD)
{
  // Increase error percent factor
  eP = eP * 1.08;

  // Error difference factor
  int eDFactor = 100;

  if (eD < 0)
    eDFactor = 60;

  // j == -1 -> fast speed | j == 1 -> slow speed
  int j = -1;

  if (opt == 0)
    j = 1;

  int mS = j * (baseSpeed * eP * eP - 2 * baseSpeed * eP - eDFactor * eD * baseSpeed) + baseSpeed;

  if (mS < minSpeed)
    mS = minSpeed;

  return mS;
}

void rightWheel(int speed)
{
  if (speed > 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  { // Negative speed, go backwards
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    speed = speed * -1;
  }

  analogWrite(PWMA, speed);
}

void leftWheel(int speed)
{
  if (speed > 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  { // Negative speed, go backwards
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    speed = speed * -1;
  }

  // Compensate for left wheel having less torque
  analogWrite(PWMB, speed + 4);
}
