#include <Arduino.h>

// =====================
// Sensor pins (confirmed correct)
// =====================
const int SENSOR_PINS[5] = {32, 33, 25, 26, 27};

// =====================
// Motor pins (confirmed correct)
// =====================
const int ENA = 14;
const int IN1 = 12;
const int IN2 = 13;

const int ENB = 23;
const int IN3 = 15;
const int IN4 = 4;

// =====================
// RGB LED pins
// =====================
const int LED_R = 17;
const int LED_G = 16;
const int LED_B = 5;

// =====================
// PWM
// =====================
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// =====================
// Drive tuning
// =====================
const int BASE_SPEED = 170;
const int LEFT_MOTOR_OFFSET = 25;
const float STRAIGHT_ERROR_LIMIT = 0.25;
const float HARD_TURN_ERROR_LIMIT = 1.0;

// Hard-turn power.
const int TURN_FORWARD_MAX = 240;
const int TURN_REVERSE_MAX = -180;

// Lost-line search uses equal forward/reverse power to reduce forward drifting.
const int SEARCH_TURN_SPEED = 210;

// =====================
// PD parameters
// =====================
const float KP = 45.0;
const float KD = 70.0;

float error = 0.0;
float lastError = 0.0;

// =====================
// Lost-line protection
// =====================
unsigned long lostTime = 0;
const unsigned long LOST_LIMIT = 600;

// =====================
// State machine
// =====================
enum CarState
{
  READY,
  TRACKING,
  SEARCHING,
  STOPPED
};

CarState state = READY;

// =====================
// LED control
// =====================
void setLED(bool r, bool g, bool b)
{
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

// =====================
// Motor control
// =====================
void motorDrive(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  if (right >= 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  ledcWrite(ENA, abs(left));
  ledcWrite(ENB, abs(right));
}

// =====================
// Read 5-channel line sensor
// Returns false when all sensors are off the black line.
// =====================
bool readLine(float &err)
{
  int weightSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 5; i++)
  {
    int value = !digitalRead(SENSOR_PINS[i]);

    if (value)
    {
      weightSum += (i - 2);
      activeCount++;
    }
  }

  if (activeCount == 0)
  {
    return false;
  }

  err = (float)weightSum / activeCount;
  return true;
}

void stopAndLock()
{
  state = STOPPED;
  motorDrive(0, 0);
  setLED(true, false, false);
  Serial.println("STOPPED: lost line timeout");
}

void searchLine()
{
  if (state != SEARCHING)
  {
    state = SEARCHING;
    lostTime = millis();
    setLED(true, false, true);
    Serial.println("SEARCHING: line lost");
  }

  if (millis() - lostTime > LOST_LIMIT)
  {
    stopAndLock();
    return;
  }

  if (lastError > 0)
  {
    motorDrive(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED);
  }
  else
  {
    motorDrive(-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED);
  }
}

void trackLine(float newError)
{
  state = TRACKING;
  setLED(false, true, false);

  error = newError;
  float derivative = error - lastError;
  float correction = KP * error + KD * derivative;

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (abs(error) <= STRAIGHT_ERROR_LIMIT)
  {
    leftSpeed = BASE_SPEED + LEFT_MOTOR_OFFSET;
    rightSpeed = BASE_SPEED;
  }
  else if (abs(error) < HARD_TURN_ERROR_LIMIT)
  {
    leftSpeed = BASE_SPEED + correction;
    rightSpeed = BASE_SPEED - correction;
  }
  else if (error > 0)
  {
    leftSpeed = TURN_FORWARD_MAX;
    rightSpeed = TURN_REVERSE_MAX;
  }
  else
  {
    leftSpeed = TURN_REVERSE_MAX;
    rightSpeed = TURN_FORWARD_MAX;
  }

  motorDrive(leftSpeed, rightSpeed);
  lastError = error;

  Serial.printf(
    "TRACKING: E=%.2f D=%.2f C=%.2f L=%d R=%d\n",
    error,
    derivative,
    correction,
    leftSpeed,
    rightSpeed);
}

// =====================
// Setup
// =====================
void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 5; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);

  motorDrive(0, 0);
  setLED(false, false, true);
  Serial.println("READY");
}

// =====================
// Loop
// =====================
void loop()
{
  if (state == STOPPED)
  {
    motorDrive(0, 0);
    setLED(true, false, false);
    delay(20);
    return;
  }

  float newError = 0.0;
  bool lineDetected = readLine(newError);

  if (!lineDetected)
  {
    searchLine();
    delay(5);
    return;
  }

  trackLine(newError);
  delay(5);
}
