///################################################//
///################################################//
//############ SETTINGS ###########################//
#include <Arduino.h>
#include <Servo.h>

//MOTOR TYPE SELECTION
// Select the motor type used by your robot:
// 1 → Encoder Motors (with Encoder + BNO086)
// 2 → Normal BO Motors (without Encoder + BNO086)
#define MOTOR_TYPE 2

#define WHEEL_DIAMETER 65.0  //6.5 cm

#define MOTOR_SPEED_FACTOR 1  //Select the value between 0 to 2
#define TURN_SPEED_FACTOR 2   //Select the value between 0 to 2


///################################################//
///################################################//
//############ ACTION PLAN ###########################//
/*
   ============================================================
   ROBOT MOVEMENT SEQUENCE
   ============================================================
   F = Forward (cm)
   B = Backward (cm)
   L = Left turn (degrees)
   R = Right turn (degrees)
   S = Servo 1 angle
   T = Servo 2 angle
   ============================================================
*/

struct RobotStep {
  char action;
  float value;
  unsigned long waitTime;
};

RobotStep steps[] = {
  {'S', 90, 1000},
  {'S', 0, 1000}
  /*{'F', 50,1000},
  {'B', 50, 1000},
  {'L', 80, 2000},
  {'F', 73, 1000},
  {'T', 80, 1500},
  {'S', 180, 1500},
  {'B', 74, 3000},
  {'L', 80, 2000},
  {'F', 12, 1000},
  {'R', 87, 2000},
  {'F', 78, 1000},
  {'B', 77, 2000},
  {'F', 10, 1000},
  {'L', 87, 2000},
  {'F', 25, 1000},
  {'R', 90, 1000},
  {'F', 57, 1000},
  {'R',  5,  500},
  {'B',  3, 1000},
  {'L',  5, 1000},
  {'B', 40, 1000},
  {'L', 90, 1000},
  {'F',  5, 1000},
  {'R', 90, 1000},
  {'F', 57, 0}    // Stop, wait 1 sec */
};

///############################################################//
///############################################################//
//################# MAIN CODE #################################//
//################# DONT TOUCH ################################//
//############ UNLESS YOU ARE AN EXPERT########################//




// ============================================================
// MOTOR TYPE SELECTION
// Select the motor type used by your robot:
//
// 1 → Encoder Motors (with Encoder + BNO086)
// 2 → Normal BO Motors (without Encoder + BNO086)
//
// Change MOTOR_TYPE below according to your hardware.
#define ENCODER_MOTOR 1
#define NORMAL_BO_MOTOR 2
// Select as per your requirement.
#define COUNTS_PER_REV 1214.0

// ============================================================




// MOTOR SPEED SETTINGS
// PWM range: 0–255
// Adjust these values according to battery voltage,
// motor performance, robot weight, and field conditions.
#define MOTOR_SPEED min(255, (int)(120 * MOTOR_SPEED_FACTOR))
#define TURN_SPEED min(255, (int)(80 * TURN_SPEED_FACTOR))  // Left/Right turning speed

#define MIN_MOVE_SPEED 35
#define MIN_TURN_SPEED 35

#define BO_MOVE_MS_PER_CM 7.4
#define BO_TURN_MS_PER_DEG 1.23529412

// ============================================================
// NORMAL BO MOTOR SETTINGS
// ============================================================
//
// These values MUST be calibrated for your BO motors.
//
// Example:
// If 1 cm requires approximately 25 ms:
//
// BO_MOVE_MS_PER_CM = 25
//
// If a 90 degree turn requires approximately 630 ms:
//
// BO_TURN_MS_PER_DEG = 7
//


#define MOVEMENT_TIMEOUT 30000UL
#define TURN_TIMEOUT 10000UL

#define HEADING_KP 7.0
#define HEADING_KD 0.0
#define MAX_HEADING_CORRECTION 20
#define HEADING_DEADBAND 0.3
#define IMU_ALPHA 0.15
#define IMU_STALE_TIMEOUT 500UL

#define ENCODER_SYNC_KP 0.04
#define MAX_SYNC_CORRECTION 10

#define M1_IN1 4
#define M1_IN2 7
#define M1_ENA 6

#define M2_IN3 13
#define M2_IN4 12
#define M2_ENB 11

#define SERVO1_PIN 9   // Servo1 Pin 9
#define SERVO2_PIN 10  // Servo2 pin 10

#if MOTOR_TYPE == ENCODER_MOTOR

#include <Wire.h>
#include <Adafruit_BNO08x.h>  // Need to install this from library Manager.

#define M1_ENC_A 3
#define M1_ENC_B 5
#define M2_ENC_A 19
#define M2_ENC_B 18

#define BNO086_ADDRESS 0x4B

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t sensorValue;

volatile long encoder1 = 0;
volatile long encoder2 = 0;

float yaw = 0;
float filteredYaw = 0;
float targetYaw = 0;

unsigned long lastIMU = 0;
unsigned long lastControl = 0;

#endif


Servo servo1;
Servo servo2;





const int STEP_COUNT = sizeof(steps) / sizeof(steps[0]);


/* ============================================================
   MOTOR CONTROL
   ============================================================ */

void motor(int m, int speed) {

  speed = constrain(speed, -255, 255);

  if (m == 1) {

    if (speed > 0) {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    } else if (speed < 0) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    } else {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, LOW);
    }

    analogWrite(M1_ENA, abs(speed));
  }

  else {

    if (speed > 0) {
      digitalWrite(M2_IN3, HIGH);
      digitalWrite(M2_IN4, LOW);
    } else if (speed < 0) {
      digitalWrite(M2_IN3, LOW);
      digitalWrite(M2_IN4, HIGH);
    } else {
      digitalWrite(M2_IN3, LOW);
      digitalWrite(M2_IN4, LOW);
    }

    analogWrite(M2_ENB, abs(speed));
  }
}


void stopMotors() {
  motor(1, 0);
  motor(2, 0);
}


/* ============================================================
   ENCODERS + IMU
   ============================================================ */

#if MOTOR_TYPE == ENCODER_MOTOR

void encoder1ISR() {
  if (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B))
    encoder1++;
  else
    encoder1--;
}


void encoder2ISR() {
  if (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B))
    encoder2++;
  else
    encoder2--;
}


void resetEncoders() {

  noInterrupts();
  encoder1 = 0;
  encoder2 = 0;
  interrupts();
}


long getEncoder1() {

  noInterrupts();
  long x = encoder1;
  interrupts();
  return x;
}


long getEncoder2() {

  noInterrupts();
  long x = encoder2;
  interrupts();
  return x;
}


float angleDifference(float target, float current) {

  float error = target - current;

  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  return error;
}


void quaternionToYaw(sh2_RotationVectorWAcc_t &r) {

  float ysqr = r.j * r.j;

  float t3 = 2.0 * (r.real * r.k + r.i * r.j);
  float t4 = 1.0 - 2.0 * (ysqr + r.k * r.k);

  yaw = atan2(t3, t4) * 180.0 / PI;

  while (yaw > 180) yaw -= 360;
  while (yaw < -180) yaw += 360;
}


void updateIMU() {

  if (!bno.wasReset()) {

    if (bno.getSensorEvent(&sensorValue)) {

      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

        quaternionToYaw(sensorValue.un.rotationVector);

        if (lastIMU == 0)
          filteredYaw = yaw;
        else
          filteredYaw =
            filteredYaw * (1.0 - IMU_ALPHA) + yaw * IMU_ALPHA;

        while (filteredYaw > 180) filteredYaw -= 360;
        while (filteredYaw < -180) filteredYaw += 360;

        lastIMU = millis();
      }
    }
  }
}


bool imuFresh() {
  return (millis() - lastIMU) < IMU_STALE_TIMEOUT;
}


float headingCorrection(float target) {

  float error = angleDifference(target, filteredYaw);

  if (abs(error) < HEADING_DEADBAND)
    return 0;

  float dt = (millis() - lastControl) / 1000.0;

  static float previousError = 0;

  float derivative = 0;

  if (dt > 0)
    derivative = (error - previousError) / dt;

  previousError = error;
  lastControl = millis();

  float correction =
    HEADING_KP * error + HEADING_KD * derivative;

  return constrain(
    correction,
    -MAX_HEADING_CORRECTION,
    MAX_HEADING_CORRECTION);
}


long distanceToCounts(float cm) {

  float wheelCircumference =
    PI * WHEEL_DIAMETER;

  return abs(
    (long)(cm / wheelCircumference * COUNTS_PER_REV));
}

#endif


/* ============================================================
   ENCODER MOTOR MOVEMENT
   ============================================================ */

#if MOTOR_TYPE == ENCODER_MOTOR

void moveEncoder(float cm, int direction) {

  resetEncoders();

  long target =
    distanceToCounts(cm);

  if (target <= 0)
    return;

  updateIMU();

  targetYaw = filteredYaw;

  unsigned long start = millis();

  while (true) {

    updateIMU();

    long c1 = abs(getEncoder1());
    long c2 = abs(getEncoder2());

    long average = (c1 + c2) / 2;

    if (average >= target)
      break;

    if (millis() - start > MOVEMENT_TIMEOUT)
      break;

    int speed = MOTOR_SPEED;

    long remaining = target - average;

    if (remaining < target * 0.25) {

      speed = map(
        remaining,
        0,
        target * 0.25,
        MIN_MOVE_SPEED,
        MOTOR_SPEED);

      speed = constrain(
        speed,
        MIN_MOVE_SPEED,
        MOTOR_SPEED);
    }

    int syncError = c1 - c2;

    int syncCorrection =
      constrain(
        (int)(syncError * ENCODER_SYNC_KP),
        -MAX_SYNC_CORRECTION,
        MAX_SYNC_CORRECTION);

    float heading =
      headingCorrection(targetYaw);

    int leftSpeed =
      speed - syncCorrection - heading;

    int rightSpeed =
      speed + syncCorrection + heading;

    leftSpeed = constrain(
      leftSpeed,
      MIN_MOVE_SPEED,
      255);

    rightSpeed = constrain(
      rightSpeed,
      MIN_MOVE_SPEED,
      255);

    motor(1, leftSpeed * direction);
    motor(2, rightSpeed * direction);

    delay(5);
  }

  stopMotors();
}


#endif


/* ============================================================
   NORMAL BO MOTOR MOVEMENT
   ============================================================ */

#if MOTOR_TYPE == NORMAL_BO_MOTOR

void moveBO(float cm, int direction) {

  unsigned long moveTime =
    (unsigned long)(abs(cm) * BO_MOVE_MS_PER_CM);

  moveTime =
    min(moveTime, MOVEMENT_TIMEOUT);

  // Motor 2 is mounted in the opposite physical direction.
  // Reverse its command so both wheels move forward/backward together.
  motor(1, MOTOR_SPEED * direction);
  motor(2, -MOTOR_SPEED * direction);

  delay(moveTime);

  stopMotors();
}

#endif


/* ============================================================
   DISTANCE MOVEMENT
   ============================================================ */

void moveDistance(float cm, int direction) {

#if MOTOR_TYPE == ENCODER_MOTOR

  moveEncoder(cm, direction);

#else

  moveBO(cm, direction);

#endif
}


/* ============================================================
   TURNING
   ============================================================ */

#if MOTOR_TYPE == ENCODER_MOTOR

void turnIMU(float degrees) {

  updateIMU();

  float startYaw = filteredYaw;

  targetYaw = startYaw + degrees;

  while (targetYaw > 180) targetYaw -= 360;
  while (targetYaw < -180) targetYaw += 360;

  unsigned long start = millis();

  while (true) {

    updateIMU();

    if (!imuFresh())
      break;

    float error =
      angleDifference(targetYaw, filteredYaw);

    if (abs(error) <= HEADING_DEADBAND)
      break;

    if (millis() - start > TURN_TIMEOUT)
      break;

    int speed =
      (int)(MIN_TURN_SPEED + abs(error) * 2.0);

    speed =
      constrain(
        speed,
        MIN_TURN_SPEED,
        TURN_SPEED);

    int direction =
      error > 0 ? 1 : -1;

    motor(1, direction * speed);
    motor(2, -direction * speed);

    delay(5);
  }

  stopMotors();
  delay(100);
}


#endif


#if MOTOR_TYPE == NORMAL_BO_MOTOR

void turnBO(float degrees) {

  unsigned long turnTime =
    (unsigned long)(abs(degrees) * BO_TURN_MS_PER_DEG);

  turnTime =
    min(turnTime, TURN_TIMEOUT);

  int direction =
    degrees > 0 ? 1 : -1;

  // With Motor 2 reversed, same logical direction makes the robot turn.
  motor(1, direction * TURN_SPEED);
  motor(2, direction * TURN_SPEED);

  delay(turnTime);

  stopMotors();
}

#endif


void turnByAngle(float degrees) {

#if MOTOR_TYPE == ENCODER_MOTOR

  turnIMU(degrees);

#else

  turnBO(degrees);

#endif
}


/* ============================================================
   EXECUTE ONE STEP
   ============================================================ */

void executeStep(RobotStep step) {

  switch (step.action) {

    case 'F':
      moveDistance(step.value, 1);
      delay(step.waitTime);
      break;

    case 'B':
      moveDistance(step.value, -1);
      delay(step.waitTime);
      break;

    case 'L':
      turnByAngle(abs(step.value));
      delay(step.waitTime);
      break;

    case 'R':
      turnByAngle(-abs(step.value));
      delay(step.waitTime);
      break;

    case 'S':
      servo1.write(constrain((int)step.value, 0, 180));
      delay(step.waitTime);
      break;

    case 'T':
      servo2.write(constrain((int)step.value, 0, 80));
      delay(step.waitTime);
      break;
  }
}


/* ============================================================
   IMU INITIALIZATION
   ============================================================ */

#if MOTOR_TYPE == ENCODER_MOTOR

void initializeIMU() {

  Wire.begin();

  if (!bno.begin_I2C(BNO086_ADDRESS)) {

    Serial.println("BNO086 NOT FOUND!");
    while (1) {
      stopMotors();
      delay(1000);
    }
  }

  bno.enableReport(
    SH2_ROTATION_VECTOR,
    10000);

  delay(100);

  unsigned long start = millis();

  while (millis() - start < 3000) {

    updateIMU();

    if (lastIMU != 0)
      break;

    delay(10);
  }

  if (lastIMU == 0) {

    Serial.println("IMU DATA ERROR!");

    while (1) {
      stopMotors();
      delay(1000);
    }
  }

  targetYaw = filteredYaw;
}

#endif


/* ============================================================
   SETUP
   ============================================================ */

void setup() {

  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_ENA, OUTPUT);

  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
  pinMode(M2_ENB, OUTPUT);

  stopMotors();

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  servo1.write(0);
  servo2.write(0);


#if MOTOR_TYPE == ENCODER_MOTOR

  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);

  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(M1_ENC_A),
    encoder1ISR,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(M2_ENC_A),
    encoder2ISR,
    CHANGE);

  initializeIMU();

  Serial.println("MODE: ENCODER + BNO086");

#else

  Serial.println("MODE: NORMAL BO MOTOR");

#endif

  delay(1000);

  Serial.println("STARTING SEQUENCE");

  for (int i = 0; i < STEP_COUNT; i++) {

    Serial.print("STEP ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(steps[i].action);

    executeStep(steps[i]);
  }

  stopMotors();

  Serial.println("SEQUENCE COMPLETE");
}


/* ============================================================
   LOOP
   ============================================================ */

void loop() {

  stopMotors();

  delay(1000);
}