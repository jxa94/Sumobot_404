#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right (M2)
#define M2_RPWM 10 // Verified: M2_LPWM is forward, M2_RPWM is backward
#define M2_LPWM 11

// Motor left (M1)
#define M1_RPWM 5 // Verified: M1_RPWM is forward, M1_LPWM is backward
#define M1_LPWM 6

// Starter switch: Digital input (KY-022 IR Receiver Signal Pin)
#define JSUMO_SWITCH 2

// Bump sensors
#define BUMP_LEFT 12
#define BUMP_RIGHT 4

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150;
const int SPEED_MAX = 200;

const float RIGHT_MOTOR_FACTOR = 1.0; // Adjust if one motor is faster

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999; // Use a large int value for "no detection" for comparisons
const int IR_MIN_DISTANCE = 10;
const int IR_DETECTION_THRESHOLD = 70;
const int IR_CLOSE_THRESHOLD = 30;

const int US_MAX_DISTANCE_VAL = 999; // Use a large int value for "no detection"
const int US_DETECTION_THRESHOLD_BACK = 40;
const int US_VERY_CLOSE_BACK = 15;

// Global variables for robot state
unsigned long initialScanStartTime = 0;
bool isPerformingInitialScan = false;

unsigned long lastScanTime = 0;
int scanDirection = 1; // 1 for right, -1 for left

int bumpSensorState = 0;

int preferredDirection = 0; // -1 for left, 0 for center, 1 for right
unsigned long lastOpponentDetectionTime = 0;
bool isOpponentVisible = false;

// JSUMO switch variables
bool robotActive = false;
bool jsumoSignalActiveLastFrame = false;
unsigned long jsumoDebounceTime = 0;
const unsigned long JSUMO_MIN_INTERVAL = 200;

// Search state variables
enum SearchBotState {
  SEARCH_STATE_SPINNING,
  SEARCH_STATE_INIT_FORWARD,
  SEARCH_STATE_MOVING_FORWARD,
  SEARCH_STATE_STOPPING_AFTER_FORWARD
};
SearchBotState currentSearchState = SEARCH_STATE_SPINNING;
unsigned long searchStateTimer = 0;
const unsigned long SEARCH_FORWARD_DURATION = 500; // ms
const unsigned long SEARCH_STOP_DURATION = 200;    // ms

// Special Turn State for US detection
enum SpecialTurnState {
  TURN_STATE_NONE,
  TURN_STATE_US_TRIGGERED_LEFT,
  TURN_STATE_US_TRIGGERED_RIGHT
};
SpecialTurnState currentSpecialTurn = TURN_STATE_NONE;
unsigned long specialTurnStartTime = 0;
const unsigned long MAX_SPECIAL_TURN_DURATION = 3000; // Max time for the 180-degree turn


// Function declarations
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int turnSpeed); // Pivot turn
void turnRight(int turnSpeed); // Pivot turn
void turnLeftOneWheel(int turnSpeed);
void turnRightOneWheel(int turnSpeed);
void stopMovement();
int getIRDistance(int sensorPin, bool actualMaxVal = false);
long getUltrasonicDistance(bool actualMaxVal = false);
void updateBumpSensorState();
void searchOpponent();
void attackWithBumpers();
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch();
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist);


void setup() {
  Serial.begin(9600);
  Serial.println("Robot Setup Starting...");

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);

  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  pinMode(ULTRASONIC_ECHO, INPUT);

  pinMode(BUMP_LEFT, INPUT_PULLUP);
  pinMode(BUMP_RIGHT, INPUT_PULLUP);

  pinMode(JSUMO_SWITCH, INPUT_PULLUP);

  stopMovement(); // Initial stop

  jsumoSignalActiveLastFrame = (digitalRead(JSUMO_SWITCH) == LOW);
  Serial.println("Setup Complete. Waiting for JSUMO Switch (IR Signal).");
}

void loop() {
  handleJsumoSwitch();

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovement(); // Keep robot stopped during initial scan phase (THIS IS INTENTIONAL)
      performInitialScan();

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE; 
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_SPINNING; 
      }
    } else {
      if (currentSpecialTurn != TURN_STATE_NONE) {
        int irLeft = getIRDistance(IR_REFLECT_LEFT);
        int irCenter = getIRDistance(IR_REFLECT_CENTER);
        int irRight = getIRDistance(IR_REFLECT_RIGHT);

        bool targetAcquiredByIR = (irLeft < IR_DETECTION_THRESHOLD ||
                                   irCenter < IR_DETECTION_THRESHOLD ||
                                   irRight < IR_DETECTION_THRESHOLD);

        if (targetAcquiredByIR) {
          Serial.println("Special Turn: Target ACQUIRED by IR. Stopping special turn.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = true; 
          lastOpponentDetectionTime = millis();
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Turn: MAX DURATION reached. Giving up special turn.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; 
        } else {
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_LEFT) {
            turnLeftOneWheel(SPEED_FAST); 
            Serial.println("Special Turn: Executing LEFT one-wheel turn.");
          } else { // TURN_STATE_US_TRIGGERED_RIGHT
            turnRightOneWheel(SPEED_FAST); 
            Serial.println("Special Turn: Executing RIGHT one-wheel turn.");
          }
          return; 
        }
      }
      updateBumpSensorState();
      executeStrategy();
    }
  } else {
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE; 
  }
}

void handleJsumoSwitch() {
  bool currentJsumoSignalActive = (digitalRead(JSUMO_SWITCH) == LOW);

  if (currentJsumoSignalActive && !jsumoSignalActiveLastFrame && (millis() - jsumoDebounceTime > JSUMO_MIN_INTERVAL)) {
    robotActive = !robotActive;
    jsumoDebounceTime = millis();

    if (robotActive) {
      isPerformingInitialScan = true;
      initialScanStartTime = millis();
      Serial.println("Robot Activated by IR. Starting 3s scan.");
      // Reset states
      isOpponentVisible = false;
      preferredDirection = 0;
      lastOpponentDetectionTime = 0;
      bumpSensorState = 0;
      currentSpecialTurn = TURN_STATE_NONE;
      currentSearchState = SEARCH_STATE_SPINNING;
      // stopMovement(); // MODIFICATION: Removed from here. Robot stops at start of loop if isPerformingInitialScan.
    } else {
      isPerformingInitialScan = false;
      currentSpecialTurn = TURN_STATE_NONE;
      stopMovement(); // Stop when deactivated
      Serial.println("Robot Deactivated by IR.");
    }
  }
  jsumoSignalActiveLastFrame = currentJsumoSignalActive;
}


void performInitialScan() {
  // This function is called while the robot is stopped (by loop() logic)
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  isOpponentVisible = false;

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  if (backDist < US_DETECTION_THRESHOLD_BACK && (backDist < minFrontIRDistance || !frontIRDetecting)) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1; 
    else if (irRight < irLeft && irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1; 
    else preferredDirection = (random(2) == 0) ? -1 : 1; 
    Serial.println("Initial Scan: US is primary target. Preferred turn for 180 set.");
  } else if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    if (irCenter == minFrontIRDistance && irCenter < IR_DETECTION_THRESHOLD) {
        preferredDirection = 0; Serial.println("Initial Scan: Opponent CENTER by IR.");
    } else if (irLeft == minFrontIRDistance && irLeft < IR_DETECTION_THRESHOLD) {
        preferredDirection = -1; Serial.println("Initial Scan: Opponent LEFT by IR.");
    } else if (irRight == minFrontIRDistance && irRight < IR_DETECTION_THRESHOLD) {
        preferredDirection = 1; Serial.println("Initial Scan: Opponent RIGHT by IR.");
    }
  }
}


void executeStrategy() {
  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; 
    return;
  }

  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  bool usDetecting = (backDist < US_DETECTION_THRESHOLD_BACK);

  if (usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && currentSpecialTurn == TURN_STATE_NONE) {
    Serial.println("Strategy: US is primary target. Initiating SPECIAL (one-wheel) TURN.");
    isOpponentVisible = true; 
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    int turnDecision = preferredDirection;
    if (turnDecision == 0) { 
        turnDecision = (random(2) == 0) ? -1 : 1; 
    }

    if (turnDecision <= 0) { 
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_LEFT; // Will use turnLeftOneWheel
    } else { 
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_RIGHT; // Will use turnRightOneWheel
    }
    return; 
  }

  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
  } else if (usDetecting) { 
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    Serial.println("Strategy: US sees opponent (not primary for special turn), front clear. Adjusting (likely one-wheel turn).");
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); 
  } else { 
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
      Serial.println("Strategy: Lost opponent briefly, reacquiring (one-wheel turn or fwd).");
      // MODIFICATION: Use one-wheel turns for reacquiring if not straight
      if (preferredDirection < 0) turnLeftOneWheel(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRightOneWheel(SPEED_MEDIUM);
      else moveForward(SPEED_MEDIUM, SPEED_MEDIUM); // If last saw center
    } else {
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching (pivot turns)...");
      searchOpponent(); // searchOpponent uses pivot turns for faster scanning
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK);
  bool frontClearEnough = (leftDist >= IR_DETECTION_THRESHOLD &&
                           centerDist >= IR_DETECTION_THRESHOLD &&
                           rightDist >= IR_DETECTION_THRESHOLD);

  // MODIFICATION: Fallback for US detection uses one-wheel turns
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontClearEnough) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust (Fallback): Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Turning FAST (one-wheel).");
      if (preferredDirection == 1) { // Prefers turning body to the right
          turnRightOneWheel(SPEED_FAST);
      } else if (preferredDirection == -1) { // Prefers turning body to the left
          turnLeftOneWheel(SPEED_FAST);
      } else { // preferredDirection was 0 or uninformative, default to a one-wheel turn (e.g., right)
          turnRightOneWheel(SPEED_FAST);
          preferredDirection = 1;  // Update preferredDirection based on the turn taken
      }
      return;
  }

  // Standard IR-based adjustment (uses differential steering which is fine)
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM,
                centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Turning/Moving.");
    moveForward(leftDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM, // Gentle turn
                leftDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Turning/Moving.");
    moveForward(rightDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST, // Gentle turn
                rightDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM);
    preferredDirection = 1;
  } else if (isOpponentVisible) {
      Serial.println("Adjust: Opponent was visible but IRs ambiguous now. Using last preferredDirection (one-wheel/fwd) or searching.");
      // MODIFICATION: Use one-wheel turns if preferredDirection suggests a turn
      if (preferredDirection < 0) turnLeftOneWheel(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRightOneWheel(SPEED_MEDIUM);
      else searchOpponent(); 
  } else {
    Serial.println("Adjust: No clear opponent path, searching (pivot turns).");
    searchOpponent(); // searchOpponent uses pivot turns
  }
}


int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);

  if (voltage < 0.42) { 
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1; 
  }
  
  float distance;
  if (voltage > 2.8) distance = 5;       
  else if (voltage > 1.5) distance = 10; 
  else if (voltage > 0.8) distance = 20; 
  else if (voltage > 0.42) distance = 35; 
  else distance = IR_MAX_DISTANCE_VAL;    


  if (distance > (actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD +1) ) {
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  }
  if (distance < IR_MIN_DISTANCE) return IR_MIN_DISTANCE;
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  unsigned long startTime;
  digitalWrite(ULTRASONIC_TRIG, LOW);
  startTime = micros();
  while (micros() - startTime < 2) { /* wait */ }

  digitalWrite(ULTRASONIC_TRIG, HIGH);
  startTime = micros();
  while (micros() - startTime < 10) { /* wait */ }
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); 
  long distance = duration * 0.034 / 2;

  if (distance == 0 || distance > (actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1) ) {
     return actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1;
  }
  return distance;
}

// Motor Control Functions - based on verified pin behavior
// M1 (Left): M1_RPWM = Forward, M1_LPWM = Backward
// M2 (Right): M2_LPWM = Forward, M2_RPWM = Backward

void moveForward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, constrain(leftSpeed, 0, SPEED_MAX)); // Left Fwd
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_LPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX)); // Right Fwd
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, constrain(leftSpeed, 0, SPEED_MAX)); // Left Bwd
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_RPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX)); // Right Bwd
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int turnSpeed) { // Pivot turn: Left motor backward, Right motor forward
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedRightSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, speed); // Left Bwd
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_LPWM, adjustedRightSpeed); // Right Fwd
  analogWrite(M2_RPWM, 0);
}

void turnRight(int turnSpeed) { // Pivot turn: Left motor forward, Right motor backward
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedRightSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, speed); // Left Fwd
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, adjustedRightSpeed); // Right Bwd
  analogWrite(M2_LPWM, 0);
}

void turnLeftOneWheel(int turnSpeed) { // Robot body turns CCW (left). Right motor forward, Left motor stop.
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedRightSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);

  analogWrite(M1_RPWM, 0); // Left Stop
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_LPWM, adjustedRightSpeed); // Right Fwd
  analogWrite(M2_RPWM, 0);
}

void turnRightOneWheel(int turnSpeed) { // Robot body turns CW (right). Left motor forward, Right motor stop.
  int speed = constrain(turnSpeed, 0, SPEED_MAX);

  analogWrite(M1_RPWM, speed); // Left Fwd
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0); // Right Stop
  analogWrite(M2_LPWM, 0);
}


void stopMovement() {
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);
}

void updateBumpSensorState() {
  bool leftBumped = (digitalRead(BUMP_LEFT) == LOW);
  bool rightBumped = (digitalRead(BUMP_RIGHT) == LOW);

  bumpSensorState = 0;
  if (leftBumped) bumpSensorState |= (1 << BUMP_LEFT_BIT);
  if (rightBumped) bumpSensorState |= (1 << BUMP_RIGHT_BIT);
}

void searchOpponent() { // Uses pivot turns for efficient scanning
  unsigned long currentTime = millis();

  if (currentSearchState == SEARCH_STATE_SPINNING && currentTime - lastScanTime > 2000) { 
    scanDirection = -scanDirection; 
    lastScanTime = currentTime;
    if (random(3) == 0) { 
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_SPINNING:
      if (scanDirection == 1) turnRight(SPEED_MEDIUM); 
      else turnLeft(SPEED_MEDIUM); 
      preferredDirection = scanDirection; 
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW, SPEED_SLOW);
      searchStateTimer = millis();
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      break;
    case SEARCH_STATE_MOVING_FORWARD:
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        stopMovement();
        searchStateTimer = millis();
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
      }
      break;
    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      if (millis() - searchStateTimer >= SEARCH_STOP_DURATION) {
        currentSearchState = SEARCH_STATE_SPINNING; 
        lastScanTime = millis(); 
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX, SPEED_MAX);
}