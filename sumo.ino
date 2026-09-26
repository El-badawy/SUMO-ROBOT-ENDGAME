#include <Servo.h>

// ============================================================
// 1. HARDWARE PIN DEFINITIONS
// ============================================================

// ---- L298N Motor Driver Pins ----
#define ENA 3
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define ENB 11

// ---- Servo Motor Pin ----
#define SERVO_PIN 9

// ---- 4x HC-SR04 Ultrasonic Sensors Pins ----
#define TRIG_FRONT 2
#define ECHO_FRONT 8

#define TRIG_RIGHT 13
#define ECHO_RIGHT 12

#define TRIG_LEFT  A0
#define ECHO_LEFT  A1

#define TRIG_BACK  10
#define ECHO_BACK  A5

// ---- 3x IR Line Sensors Pins (MH-B Modules) ----
#define LINE_BACK  A2
#define LINE_LEFT  A3
#define LINE_RIGHT A4

// ============================================================
// 2. TUNABLE PARAMETERS & THRESHOLDS
// ============================================================
#define ATTACK_DISTANCE     40      // Distance threshold to trigger attack (cm)
#define MOTOR_SPEED         255     // Maximum motor speed (0 - 255 PWM)
#define SEARCH_SPEED        200     // Cruise speed during search phase
#define REVERSE_DURATION    200     // Time to escape when edge detected (ms)
#define TURN_DURATION       250     // Time to pivot away from edge (ms)
#define SEARCH_FORWARD_MS   600     // Forward duration in search cycle (ms)
#define SEARCH_TURN_MS      200     // Turn duration in search cycle (ms)
#define ATTACK_TURN_TIMEOUT 700     // Max time spinning toward a target before giving up (ms)

// The ring has no walls, so most reads hit the timeout — keep it tight.
#define SENSOR_TIMEOUT_US   3500UL  // ~60 cm range, enough for a 40 cm attack distance

// Line sensor polarity: MH-B modules read LOW on the white edge.
#define EDGE_ACTIVE         LOW

// Servo Blade Angles
#define SERVO_IDLE_ANGLE    90      // Default blade angle (idle)
#define SERVO_ATTACK_ANGLE  15      // Attack blade angle

// Startup delay (check the competition rules for the required value)
#define STARTUP_DELAY_MS    500

// ============================================================
// 3. FINITE STATE MACHINE (FSM) DEFINITIONS
// ============================================================
enum State {
  SEARCH,
  ATTACK_FRONT,
  ATTACK_LEFT,
  ATTACK_RIGHT,
  ATTACK_BACK,
  REVERSE,
  TURN_LEFT,
  TURN_RIGHT
};

State currentState = SEARCH;
State nextStateAfterTurn = SEARCH;

unsigned long stateStartTime = 0;
unsigned long stateDuration = 0;

// Direction used while escaping the edge: -1 = backward, +1 = forward
int escapeDir = -1;

// Search phase logic variables
bool searchGoingForward = true;
int searchTurnDirection = 1; // 1 = Right, -1 = Left

Servo bladeServo;

// ============================================================
// 4. FUNCTION PROTOTYPES
// ============================================================
float readDistanceCM(int trigPin, int echoPin);
bool  isTarget(float dist);
void  setMotors(int leftSpeed, int rightSpeed);
void  enterState(State newState);
void  runSearch();

// ============================================================
// 5. SETUP FUNCTION
// ============================================================
void setup() {
  // Motor Driver Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  setMotors(0, 0);

  // Ultrasonic Sensor Pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_BACK,  OUTPUT); pinMode(ECHO_BACK,  INPUT);

  // IR Line Sensor Pins
  pinMode(LINE_BACK,  INPUT);
  pinMode(LINE_LEFT,  INPUT);
  pinMode(LINE_RIGHT, INPUT);

  // Servo Setup
  bladeServo.attach(SERVO_PIN);
  bladeServo.write(SERVO_IDLE_ANGLE);

  // Random Seed
  // A5 is used by the Back Ultrasonic ECHO, so it is not floating.
  // micros() gives a reasonably unpredictable seed instead.
  randomSeed(micros());

  // Startup Safety Delay
  delay(STARTUP_DELAY_MS);

  // Start in SEARCH state
  enterState(SEARCH);
}

// ============================================================
// 6. MAIN LOOP (NON-BLOCKING STATE MACHINE)
// ============================================================
void loop() {

  // ----------------------------------------------------------
  // 1. HIGH PRIORITY: RING EDGE DETECTION
  // IR Sensor Active LOW: LOW = White Edge Detected
  // ----------------------------------------------------------
  bool backEdge  = (digitalRead(LINE_BACK)  == EDGE_ACTIVE);
  bool leftEdge  = (digitalRead(LINE_LEFT)  == EDGE_ACTIVE);
  bool rightEdge = (digitalRead(LINE_RIGHT) == EDGE_ACTIVE);

  if ((backEdge || leftEdge || rightEdge) && currentState != REVERSE) {

    if (backEdge && !leftEdge && !rightEdge) {
      // Rear edge only: drive FORWARD away from it, not backward into it
      escapeDir = +1;
      nextStateAfterTurn = TURN_LEFT;
    } else {
      // Front/side edge: back up as before
      escapeDir = -1;
      nextStateAfterTurn = (rightEdge && !leftEdge) ? TURN_LEFT : TURN_RIGHT;
    }

    enterState(REVERSE);
  }

  // ----------------------------------------------------------
  // 2. SENSOR ACQUISITION
  //    Front is read every loop; side/back sensors take turns
  //    to keep the loop fast.
  // ----------------------------------------------------------
  static float distRight = -1.0;
  static float distLeft  = -1.0;
  static float distBack  = -1.0;
  static uint8_t sensorTurn = 0;

  float distFront = readDistanceCM(TRIG_FRONT, ECHO_FRONT);

  switch (sensorTurn) {
    case 0: distRight = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT); break;
    case 1: distLeft  = readDistanceCM(TRIG_LEFT,  ECHO_LEFT);  break;
    case 2: distBack  = readDistanceCM(TRIG_BACK,  ECHO_BACK);  break;
  }
  sensorTurn = (sensorTurn + 1) % 3;

  // ----------------------------------------------------------
  // 3. FSM STATE PROCESSING
  // ----------------------------------------------------------
  switch (currentState) {

    // ========================================================
    case SEARCH:

      runSearch();

      if (isTarget(distFront)) {
        enterState(ATTACK_FRONT);
      } else if (isTarget(distRight)) {
        enterState(ATTACK_RIGHT);
      } else if (isTarget(distLeft)) {
        enterState(ATTACK_LEFT);
      } else if (isTarget(distBack)) {
        enterState(ATTACK_BACK);
      }

      break;

    // ========================================================
    case ATTACK_FRONT:

      // Full speed forward
      setMotors(MOTOR_SPEED, MOTOR_SPEED);

      // Opponent lost -> back to search (blade reset happens in enterState)
      if (!isTarget(distFront)) {
        enterState(SEARCH);
      }

      break;

    // ========================================================
    case ATTACK_RIGHT:

      // Rotate right toward target
      setMotors(MOTOR_SPEED, -MOTOR_SPEED);

      if (isTarget(distFront)) {
        enterState(ATTACK_FRONT);
      } else if (millis() - stateStartTime > ATTACK_TURN_TIMEOUT) {
        // Gave up finding the target -> go back to search
        enterState(SEARCH);
      }

      break;

    // ========================================================
    case ATTACK_LEFT:

      // Rotate left toward target
      setMotors(-MOTOR_SPEED, MOTOR_SPEED);

      if (isTarget(distFront)) {
        enterState(ATTACK_FRONT);
      } else if (millis() - stateStartTime > ATTACK_TURN_TIMEOUT) {
        enterState(SEARCH);
      }

      break;

    // ========================================================
    case ATTACK_BACK:

      // 180-degree spin to face rear threat
      setMotors(MOTOR_SPEED, -MOTOR_SPEED);

      if (isTarget(distFront)) {
        enterState(ATTACK_FRONT);
      } else if (millis() - stateStartTime > ATTACK_TURN_TIMEOUT) {
        enterState(SEARCH);
      }

      break;

    // ========================================================
    case REVERSE:

      // Escape direction depends on which edge triggered this state
      setMotors(escapeDir * MOTOR_SPEED, escapeDir * MOTOR_SPEED);

      if (millis() - stateStartTime >= stateDuration) {
        enterState(nextStateAfterTurn);
      }

      break;

    // ========================================================
    case TURN_LEFT:

      setMotors(-MOTOR_SPEED, MOTOR_SPEED);

      if (millis() - stateStartTime >= stateDuration) {
        enterState(SEARCH);
      }

      break;

    // ========================================================
    case TURN_RIGHT:

      setMotors(MOTOR_SPEED, -MOTOR_SPEED);

      if (millis() - stateStartTime >= stateDuration) {
        enterState(SEARCH);
      }

      break;
  }
}

// ============================================================
// 7. ULTRASONIC DISTANCE FUNCTION
// ============================================================
float readDistanceCM(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);

  if (duration == 0) {
    return -1.0; // No echo within range
  }

  return (duration * 0.0343) / 2.0;
}

// ============================================================
// 8. TARGET CHECK HELPER
// ============================================================
bool isTarget(float dist) {
  return (dist > 0 && dist < ATTACK_DISTANCE);
}

// ============================================================
// 9. MOTOR CONTROL FUNCTION
// ============================================================
void setMotors(int leftSpeed, int rightSpeed) {

  // LEFT MOTOR - MOTOR A
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(leftSpeed));

  // RIGHT MOTOR - MOTOR B
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(rightSpeed));
}

// ============================================================
// 10. STATE TRANSITION FUNCTION
// ============================================================
void enterState(State newState) {

  currentState = newState;
  stateStartTime = millis();

  // Blade is lowered only during a front attack, raised in every other state
  if (newState == ATTACK_FRONT) {
    bladeServo.write(SERVO_ATTACK_ANGLE);
  } else {
    bladeServo.write(SERVO_IDLE_ANGLE);
  }

  switch (newState) {

    case REVERSE:
      stateDuration = REVERSE_DURATION;
      break;

    case TURN_LEFT:
    case TURN_RIGHT:
      stateDuration = TURN_DURATION;
      break;

    case SEARCH:
      searchGoingForward = true;
      stateDuration = 0;
      break;

    default:
      stateDuration = 0;
      break;
  }
}

// ============================================================
// 11. SEARCH FUNCTION
// ============================================================
void runSearch() {

  unsigned long elapsed = millis() - stateStartTime;

  // SEARCH FORWARD
  if (searchGoingForward) {

    setMotors(SEARCH_SPEED, SEARCH_SPEED);

    if (elapsed >= SEARCH_FORWARD_MS) {
      searchGoingForward = false;
      stateStartTime = millis();
      searchTurnDirection = (random(0, 2) == 0) ? 1 : -1;
    }
  }

  // SEARCH TURN
  else {

    if (searchTurnDirection == 1) {
      setMotors(SEARCH_SPEED, -SEARCH_SPEED);  // Turn right
    } else {
      setMotors(-SEARCH_SPEED, SEARCH_SPEED);  // Turn left
    }

    if (elapsed >= SEARCH_TURN_MS) {
      searchGoingForward = true;
      stateStartTime = millis();
    }
  }
} 