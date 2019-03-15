#include <QTRSensors.h>

// This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
// QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
// The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2,
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to
// QTR_NO_EMITTER_PIN.

// The setup phase of this example calibrates the sensor for ten seconds and turns on
// the LED built in to the Arduino on pin 13 while calibration is going on.
// During this phase, you should expose each reflectance sensor to the lightest and
// darkest readings they will encounter.
// For example, if you are making a line follower, you should slide the sensors across the
// line during the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in poor readings.
// If you want to skip the calibration phase, you can get the raw sensor readings
// (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
// qtrrc.readLine(sensorValues).

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance)
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

// Sensors:
#define NUM_SENSORS 5                  // number of sensors used
#define TIMEOUT 2000                   // waits for X microseconds for sensor outputs to go low
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
const int BIN1 = 7;
const int BIN2 = 6;

const int maxSpeed = 120;
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

  for (int i = 0; i < 400; i++)
  {
    qtrrc.calibrate();

    if (i == 0)
    { // Spin left
      rightWheel(cS);
      leftWheel(-1 * cS);
    }
    else if (digitalRead(S2) == 1 && digitalRead(S3) == 0)
    { // Spin right
      rightWheel(-1 * cS);
      leftWheel(cS);
    }
    else if (digitalRead(S6) == 1 && digitalRead(S5) == 0)
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
  while ( ! (position > 1900 && position < 2100) ) {
    position = qtrrc.readLine(sensorValues);
    if ( position < 2000 ) {
      // Spin left
      rightWheel(cS);
      leftWheel(-1 * cS);
    } else if ( position > 2000 ) {
      // Spin left
      rightWheel(-1 * cS);
      leftWheel(cS);
    }
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

  if (error == 0)
  {
    rightWheel(maxSpeed);
    leftWheel(maxSpeed);
  }
  else if (position < 2000)
  { // Spin left
    rightWheel(maxSpeed);
    leftWheel(smallerSpeed(errorPercent));
  }
  else if (position > 2000)
  { // Spin right
    rightWheel(smallerSpeed(errorPercent));
    leftWheel(maxSpeed);
  }
}

int smallerSpeed(float eP)
{
  int mS = -1 * (maxSpeed / 2) * log10(eP + 0.01);
  return mS;
}

int biggerSpeed(float eP)
{
  int mS = maxSpeed - (speedDiff * (eP / 2));
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
