#include <Arduino.h>

// Set to 1 while tuning through Serial Monitor, then set to 0 for competition.
#define DEBUG 0

#if DEBUG
#define DEBUG_BEGIN(...) Serial.begin(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_BEGIN(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

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
// ASL / status LED pins
// =====================
const int LED_R = 17;
const int LED_G = 16;
const int LED_B = 5; // Unused if the car only has red/green LEDs.

// =====================
// Start button
// =====================
// Uses the ESP32 DevKit BOOT button on GPIO0. Press it after power-on to start.
// Do not hold BOOT while resetting or powering on, or the board may enter flash mode.
const int START_BUTTON_PIN = 0;
const int START_BUTTON_ACTIVE_LEVEL = LOW;

// =====================
// PWM
// =====================
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// =====================
// Drive tuning
// =====================
const int BASE_SPEED = 200;
const int LEFT_MOTOR_POWER_BOOST = 30;
const float STRAIGHT_ERROR_LIMIT = 0.40;
const float STRAIGHT_RELEASE_LIMIT = 0.75;
const int MAX_PD_CORRECTION = 44;

// Caster-wheel alignment tuning. When an outer sensor catches the line,
// keep turning into that direction until the center sensor returns.
const int ALIGN_TURN_FORWARD = 225;
const int ALIGN_TURN_REVERSE = -125;
const unsigned long ALIGN_LIMIT = 900;

// Lost-line recovery: make a long sweep instead of a small left/right shake.
const int SEARCH_TURN_FORWARD = 215;
const int SEARCH_TURN_REVERSE = -120;
const unsigned long SEARCH_PRIMARY_TIME = 850;
const unsigned long SEARCH_REST_TIME = 10;
const unsigned long SEARCH_OPPOSITE_TIME = 850;
const unsigned long LOST_LIMIT = 2400;

// Finish logic: stop when the line ends after a stable straight approach.
const unsigned long FINISH_APPROACH_STRAIGHT_TIME = 80;
const unsigned long FINISH_LINE_END_CONFIRM_TIME = 90;
const unsigned long FINISH_RECOVERY_IGNORE_TIME = 900;

// =====================
// PD parameters
// =====================
const float KP = 40.0;
const float KD = 55.0;
const float ERROR_FILTER_ALPHA = 0.30;

float error = 0.0;
float lastError = 0.0;
float filteredError = 0.0;
bool errorFilterReady = false;
bool straightMode = false;

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
unsigned long finishApproachStartTime = 0;
unsigned long finishLineEndStartTime = 0;
unsigned long lastRecoveryTime = 0;
int alignDir = 0;
bool runStarted = false;

void setLED(bool r, bool g, bool b)
{
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void setASLSafe()
{
  setLED(false, true, false);
}

void setASLAutonomous()
{
  setLED(true, false, false);
}

void setASLReady()
{
  setLED(false, true, false);
}

bool startButtonPressed()
{
  return digitalRead(START_BUTTON_PIN) == START_BUTTON_ACTIVE_LEVEL;
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

int activeSensorCount(int mask)
{
  int count = 0;

  for (int i = 0; i < 5; i++)
  {
    if (mask & (1 << i))
    {
      count++;
    }
  }

  return count;
}

bool normalStraightDetected(int mask, float lineError)
{
  return centerOnLine(mask) &&
         activeSensorCount(mask) <= 3 &&
         abs(lineError) <= STRAIGHT_ERROR_LIMIT;
}

void clearFinishLatch()
{
  finishLineEndStartTime = 0;
}

void resetFinishDetection()
{
  finishApproachStartTime = 0;
  finishLineEndStartTime = 0;
}

void updateFinishApproach(int mask, float lineError)
{
  if (state != TRACKING)
  {
    resetFinishDetection();
    return;
  }

  if (normalStraightDetected(mask, lineError))
  {
    clearFinishLatch();
    if (finishApproachStartTime == 0)
    {
      finishApproachStartTime = millis();
    }
    return;
  }

  resetFinishDetection();
}

bool handleFinishLineEnd()
{
  bool timedApproachReady = finishApproachStartTime != 0 &&
                            millis() - finishApproachStartTime >= FINISH_APPROACH_STRAIGHT_TIME;
  bool straightModeApproachReady = straightMode && abs(lastError) <= STRAIGHT_ERROR_LIMIT;
  bool approachReady = timedApproachReady || straightModeApproachReady;
  bool recoveryQuiet = millis() - lastRecoveryTime >= FINISH_RECOVERY_IGNORE_TIME;

  if (!approachReady || !recoveryQuiet)
  {
    return false;
  }

  motorBrake();

  if (finishLineEndStartTime == 0)
  {
    finishLineEndStartTime = millis();
    return true;
  }

  if (millis() - finishLineEndStartTime >= FINISH_LINE_END_CONFIRM_TIME)
  {
    finishAndLock();
  }

  return true;
}

bool needsBackAlign(int mask)
{
  bool outerLeft = (mask & 0b00001) != 0;
  bool outerRight = (mask & 0b10000) != 0;
  return !centerOnLine(mask) && (outerLeft || outerRight);
}

int edgeTurnDirection(int mask, float lineError)
{
  bool outerLeft = (mask & 0b00001) != 0;
  bool outerRight = (mask & 0b10000) != 0;

  if (outerRight && !outerLeft)
  {
    return 1;
  }

  if (outerLeft && !outerRight)
  {
    return -1;
  }

  return lineError >= 0 ? 1 : -1;
}

void stopAndLock()
{
  state = STOPPED;
  motorBrake();
  setASLSafe();
  DEBUG_PRINTLN("STOPPED: line lost");
}

void finishAndLock()
{
  state = STOPPED;
  motorBrake();
  setASLSafe();
  DEBUG_PRINTLN("FINISHED");
}

void startRun()
{
  runStarted = true;
  state = TRACKING;
  lostTime = 0;
  alignTime = 0;
  finishApproachStartTime = 0;
  finishLineEndStartTime = 0;
  lastRecoveryTime = 0;
  error = 0.0;
  lastError = 0.0;
  filteredError = 0.0;
  errorFilterReady = false;
  straightMode = false;
  setASLAutonomous();
  DEBUG_PRINTLN("RUN START");
}

void turnToward(int dir, int forwardSpeed, int reverseSpeed)
{
  if (dir > 0)
  {
    motorDrive(forwardSpeed, reverseSpeed);
  }
  else
  {
    motorDrive(reverseSpeed, forwardSpeed);
  }
}

void turnAwayFrom(int dir, int forwardSpeed, int reverseSpeed)
{
  turnToward(-dir, forwardSpeed, reverseSpeed);
}

void searchLine()
{
  if (state != SEARCHING)
  {
    state = SEARCHING;
    lostTime = millis();
    lastRecoveryTime = millis();
    resetFinishDetection();
    setASLAutonomous();
    DEBUG_PRINTLN("SEARCHING");
  }

  unsigned long elapsed = millis() - lostTime;

  if (elapsed > LOST_LIMIT)
  {
    stopAndLock();
    return;
  }

  unsigned long cycleTime = SEARCH_PRIMARY_TIME + SEARCH_REST_TIME + SEARCH_OPPOSITE_TIME + SEARCH_REST_TIME;
  unsigned long phase = elapsed % cycleTime;
  int searchDir = lastError >= 0 ? 1 : -1;

  if (phase < SEARCH_PRIMARY_TIME)
  {
    turnToward(searchDir, SEARCH_TURN_FORWARD, SEARCH_TURN_REVERSE);
    return;
  }

  phase -= SEARCH_PRIMARY_TIME;

  if (phase < SEARCH_REST_TIME)
  {
    motorBrake();
    return;
  }

  phase -= SEARCH_REST_TIME;

  if (phase < SEARCH_OPPOSITE_TIME)
  {
    turnAwayFrom(searchDir, SEARCH_TURN_FORWARD, SEARCH_TURN_REVERSE);
    return;
  }

  motorBrake();
}

void startAligning(int dir)
{
  state = ALIGNING;
  alignDir = dir;
  alignTime = millis();
  lastRecoveryTime = millis();
  resetFinishDetection();
  setASLAutonomous();
  DEBUG_PRINTLN("EDGE ALIGNING");
}

void alignToLine(int mask)
{
  unsigned long elapsed = millis() - alignTime;

  if (centerOnLine(mask))
  {
    state = TRACKING;
    lastRecoveryTime = millis();
    motorDrive(BASE_SPEED, BASE_SPEED);
    return;
  }

  if (elapsed > ALIGN_LIMIT)
  {
    stopAndLock();
    return;
  }

  turnToward(alignDir, ALIGN_TURN_FORWARD, ALIGN_TURN_REVERSE);
}

void trackLine(float newError)
{
  state = TRACKING;
  setASLAutonomous();

  if (!errorFilterReady)
  {
    filteredError = newError;
    errorFilterReady = true;
  }
  else
  {
    filteredError += ERROR_FILTER_ALPHA * (newError - filteredError);
  }

  error = filteredError;

  if (abs(error) <= STRAIGHT_ERROR_LIMIT)
  {
    straightMode = true;
  }
  else if (abs(error) >= STRAIGHT_RELEASE_LIMIT)
  {
    straightMode = false;
  }

  float derivative = error - lastError;
  float correction = KP * error + KD * derivative;
  correction = constrain(correction, -MAX_PD_CORRECTION, MAX_PD_CORRECTION);

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (straightMode)
  {
    error = 0.0;
    derivative = 0.0;
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED;
  }
  else
  {
    leftSpeed = BASE_SPEED + correction;
    rightSpeed = BASE_SPEED - correction;
  }

  motorDrive(leftSpeed, rightSpeed);
  lastError = error;

  DEBUG_PRINTF("E=%.2f D=%.2f L=%d R=%d\n", error, derivative, leftSpeed, rightSpeed);
}

void setup()
{
  DEBUG_BEGIN(115200);

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
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);

  motorBrake();
  setASLReady();
  DEBUG_PRINTLN("READY");
}

void loop()
{
  if (!runStarted)
  {
    motorBrake();
    setASLReady();

    if (startButtonPressed())
    {
      delay(30);
      if (startButtonPressed())
      {
        while (startButtonPressed())
        {
          motorBrake();
          setASLReady();
          delay(5);
        }

        startRun();
      }
    }

    delay(20);
    return;
  }

  if (state == STOPPED)
  {
    motorBrake();
    setASLSafe();
    delay(20);
    return;
  }

  float newError = 0.0;
  int sensorMask = 0;
  bool lineDetected = readLine(newError, sensorMask);

  if (!lineDetected)
  {
    if (handleFinishLineEnd())
    {
      delay(5);
      return;
    }

    resetFinishDetection();
    searchLine();
    delay(5);
    return;
  }

  updateFinishApproach(sensorMask, newError);

  if (state == ALIGNING)
  {
    alignToLine(sensorMask);
    delay(5);
    return;
  }

  if (needsBackAlign(sensorMask))
  {
    startAligning(edgeTurnDirection(sensorMask, newError));
    delay(5);
    return;
  }

  trackLine(newError);
  delay(5);
}
