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
const int LEFT_MOTOR_POWER_BOOST = 30;
const float STRAIGHT_ERROR_LIMIT = 0.25;
const int MAX_PD_CORRECTION = 55;

// Back-up-align tuning. When the line is on a side sensor, the car backs up
// while correcting its heading until the center sensor sees the line again.
const int RIGHT_ALIGN_LEFT_BACK = 190;
const int RIGHT_ALIGN_RIGHT_BACK = 230;
const int LEFT_ALIGN_LEFT_BACK = 210;
const int LEFT_ALIGN_RIGHT_BACK = 170;
const int CORNER_KICK_FORWARD = 230;
const int CORNER_KICK_REVERSE = -150;
const unsigned long CORNER_KICK_TIME = 120;
const unsigned long ALIGN_BRAKE_TIME = 0;
const unsigned long ALIGN_LIMIT = 650;

// Lost-line recovery: brake, back up briefly, then turn toward last error.
const int SEARCH_BACK_SPEED = 170;
const int SEARCH_TURN_FORWARD = 165;
const int SEARCH_TURN_REVERSE = -70;
const unsigned long SEARCH_BRAKE_TIME = 0;
const unsigned long SEARCH_BACK_TIME = 350;
const unsigned long SEARCH_PULSE_TIME = 80;
const unsigned long SEARCH_REST_TIME = 35;
const unsigned long LOST_LIMIT = 1100;
const unsigned long LOST_LIMIT_FINISH_PHASE = 7000;

// Finish logic: after 60 seconds, stop after seeing stable straight line twice.
const int FINISH_STRAIGHT_CONFIRM_COUNT = 8;
const int FINISH_STRAIGHT_EVENT_TARGET = 2;
const unsigned long FINISH_STRAIGHT_CONFIRM_TIME = 350;
const unsigned long FINISH_ENABLE_TIME = 60000;

// =====================
// PD parameters
// =====================
const float KP = 45.0;
const float KD = 70.0;

float error = 0.0;
float lastError = 0.0;

// =====================
// State machine
// =====================
enum CarState
{
  READY,
  TRACKING,
  ALIGNING,
  SEARCHING,
  STOPPED
};

CarState state = READY;
unsigned long lostTime = 0;
unsigned long alignTime = 0;
unsigned long straightStartTime = 0;
unsigned long finishStraightStartTime = 0;
unsigned long startTime = 0;
int finishConfirmCount = 0;
int finishEventCount = 0;
int alignDir = 0;
bool finishEventLatched = false;

void setLED(bool r, bool g, bool b)
{
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void motorDrive(int left, int right)
{
  if (left > 0)
  {
    left += LEFT_MOTOR_POWER_BOOST;
  }
  else if (left < 0)
  {
    left -= LEFT_MOTOR_POWER_BOOST;
  }

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

void motorBrake()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
}

bool readLine(float &err, int &mask)
{
  int weightSum = 0;
  int activeCount = 0;
  mask = 0;

  for (int i = 0; i < 5; i++)
  {
    int value = !digitalRead(SENSOR_PINS[i]);

    if (value)
    {
      weightSum += (i - 2);
      activeCount++;
      mask |= (1 << i);
    }
  }

  if (activeCount == 0)
  {
    return false;
  }

  err = (float)weightSum / activeCount;
  return true;
}

bool centerOnLine(int mask)
{
  return (mask & 0b00100) != 0;
}

void finishAndLock();

bool straightLineDetected(int mask)
{
  bool center = (mask & 0b00100) != 0;
  bool outerLeft = (mask & 0b00001) != 0;
  bool outerRight = (mask & 0b10000) != 0;
  return center && !outerLeft && !outerRight;
}

void clearFinishLatch()
{
  finishConfirmCount = 0;
  finishStraightStartTime = 0;
  finishEventLatched = false;
}

bool updateFinishCounter(int mask, float lineError)
{
  if (millis() - startTime < FINISH_ENABLE_TIME)
  {
    clearFinishLatch();
    return false;
  }

  if (state != TRACKING)
  {
    clearFinishLatch();
    return false;
  }

  if (!straightLineDetected(mask) || abs(lineError) > STRAIGHT_ERROR_LIMIT)
  {
    clearFinishLatch();
    return false;
  }

  if (finishStraightStartTime == 0)
  {
    finishStraightStartTime = millis();
    return false;
  }

  if (!finishEventLatched)
  {
    finishConfirmCount++;

    if (millis() - finishStraightStartTime >= FINISH_STRAIGHT_CONFIRM_TIME &&
        finishConfirmCount >= FINISH_STRAIGHT_CONFIRM_COUNT)
    {
      finishEventCount++;
      finishEventLatched = true;
      Serial.printf("FINISH STRAIGHT %d\n", finishEventCount);

      if (finishEventCount >= FINISH_STRAIGHT_EVENT_TARGET)
      {
        finishAndLock();
        return true;
      }
    }
  }

  return false;
}

bool needsBackAlign(int mask)
{
  bool outerLeft = (mask & 0b00001) != 0;
  bool outerRight = (mask & 0b10000) != 0;
  return !centerOnLine(mask) && (outerLeft || outerRight);
}

void updateStraightTimer(int mask)
{
  if (centerOnLine(mask) && !needsBackAlign(mask) && state == TRACKING)
  {
    if (straightStartTime == 0)
    {
      straightStartTime = millis();
    }
  }
  else
  {
    straightStartTime = 0;
  }
}

void stopAndLock()
{
  state = STOPPED;
  motorDrive(0, 0);
  setLED(true, false, false);
  Serial.println("STOPPED: line lost");
}

void finishAndLock()
{
  state = STOPPED;
  motorDrive(0, 0);
  setLED(true, false, false);
  Serial.println("FINISHED");
}

unsigned long currentLostLimit()
{
  if (millis() - startTime >= FINISH_ENABLE_TIME)
  {
    return LOST_LIMIT_FINISH_PHASE;
  }

  return LOST_LIMIT;
}

void searchLine()
{
  if (state != SEARCHING)
  {
    state = SEARCHING;
    lostTime = millis();
    setLED(true, false, true);
    motorDrive(-SEARCH_BACK_SPEED, -SEARCH_BACK_SPEED);
    Serial.println("SEARCHING");
    return;
  }

  unsigned long elapsed = millis() - lostTime;

  if (elapsed > currentLostLimit())
  {
    lostTime = millis();
    motorDrive(-SEARCH_BACK_SPEED, -SEARCH_BACK_SPEED);
    Serial.println("SEARCH RETRY");
    return;
  }

  if (elapsed < SEARCH_BRAKE_TIME)
  {
    motorBrake();
    return;
  }

  if (elapsed < SEARCH_BRAKE_TIME + SEARCH_BACK_TIME)
  {
    motorDrive(-SEARCH_BACK_SPEED, -SEARCH_BACK_SPEED);
    return;
  }

  unsigned long phase = (elapsed - SEARCH_BRAKE_TIME - SEARCH_BACK_TIME) % (SEARCH_PULSE_TIME + SEARCH_REST_TIME);

  if (phase >= SEARCH_PULSE_TIME)
  {
    motorBrake();
    return;
  }

  if (lastError > 0)
  {
    motorDrive(SEARCH_TURN_FORWARD, SEARCH_TURN_REVERSE);
  }
  else
  {
    motorDrive(SEARCH_TURN_REVERSE, SEARCH_TURN_FORWARD);
  }
}

void startAligning(int dir)
{
  state = ALIGNING;
  alignDir = dir;
  alignTime = millis();
  setLED(false, true, true);
  Serial.println("BACK ALIGNING");
}

void driveBackAlign()
{
  if (alignDir > 0)
  {
    motorDrive(-RIGHT_ALIGN_LEFT_BACK, -RIGHT_ALIGN_RIGHT_BACK);
  }
  else
  {
    motorDrive(-LEFT_ALIGN_LEFT_BACK, -LEFT_ALIGN_RIGHT_BACK);
  }
}

void alignToLine(int mask)
{
  unsigned long elapsed = millis() - alignTime;

  if (centerOnLine(mask) && elapsed > ALIGN_BRAKE_TIME)
  {
    state = TRACKING;
    motorDrive(BASE_SPEED + LEFT_MOTOR_OFFSET, BASE_SPEED);
    return;
  }

  if (elapsed > ALIGN_LIMIT)
  {
    alignTime = millis();
    driveBackAlign();
    Serial.println("ALIGN RETRY");
    return;
  }

  if (elapsed < ALIGN_BRAKE_TIME)
  {
    motorBrake();
    return;
  }

  if (elapsed < ALIGN_BRAKE_TIME + CORNER_KICK_TIME)
  {
    if (alignDir > 0)
    {
      motorDrive(CORNER_KICK_FORWARD, CORNER_KICK_REVERSE);
    }
    else
    {
      motorDrive(CORNER_KICK_REVERSE, CORNER_KICK_FORWARD);
    }

    return;
  }

  driveBackAlign();
}

void trackLine(float newError)
{
  state = TRACKING;
  setLED(false, true, false);

  error = newError;
  float derivative = error - lastError;
  float correction = KP * error + KD * derivative;
  correction = constrain(correction, -MAX_PD_CORRECTION, MAX_PD_CORRECTION);

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (abs(error) <= STRAIGHT_ERROR_LIMIT)
  {
    leftSpeed = BASE_SPEED + LEFT_MOTOR_OFFSET;
    rightSpeed = BASE_SPEED;
  }
  else
  {
    leftSpeed = BASE_SPEED + correction;
    rightSpeed = BASE_SPEED - correction;
  }

  motorDrive(leftSpeed, rightSpeed);
  lastError = error;

  Serial.printf("E=%.2f D=%.2f L=%d R=%d\n", error, derivative, leftSpeed, rightSpeed);
}

void setup()
{
  Serial.begin(115200);
  startTime = millis();

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
  int sensorMask = 0;
  bool lineDetected = readLine(newError, sensorMask);

  if (!lineDetected)
  {
    clearFinishLatch();
    searchLine();
    delay(5);
    return;
  }

  if (updateFinishCounter(sensorMask, newError))
  {
    delay(5);
    return;
  }

  if (state == ALIGNING)
  {
    straightStartTime = 0;
    alignToLine(sensorMask);
    delay(5);
    return;
  }

  if (needsBackAlign(sensorMask))
  {
    straightStartTime = 0;
    startAligning(newError > 0 ? 1 : -1);
    delay(5);
    return;
  }

  updateStraightTimer(sensorMask);
  trackLine(newError);
  delay(5);
}
