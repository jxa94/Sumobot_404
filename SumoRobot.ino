#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right (M2) - Assuming M2_LPWM is FORWARD due to mirrored mounting
#define M2_RPWM 10 // Typically Backward for M2 if M2_LPWM is Forward
#define M2_LPWM 11 // Typically Forward for M2 if mounted mirrored to M1

// Motor left (M1)
#define M1_RPWM 5 // Forward for M1
#define M1_LPWM 6 // Backward for M1

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
const int SPEED_TURN_PIVOT_INNER = 10; // Speed for the inner wheel during a pivot (can be 0 for true pivot)
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150; // Used for pivot outer wheel speed
const int SPEED_MAX = 200;

const float RIGHT_MOTOR_FACTOR = 1.0;

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999;
const int IR_MIN_DISTANCE = 10;
const int IR_DETECTION_THRESHOLD = 70;
const int IR_CLOSE_THRESHOLD = 30;

const int US_MAX_DISTANCE_VAL = 999;
const int US_DETECTION_THRESHOLD_BACK = 40;
const int US_VERY_CLOSE_BACK = 15;

// Global variables for robot state
unsigned long initialScanStartTime = 0;
bool isPerformingInitialScan = false;

unsigned long lastScanTime = 0;
int scanDirection = 1;

int bumpSensorState = 0;

int preferredDirection = 0;
unsigned long lastOpponentDetectionTime = 0;
bool isOpponentVisible = false;

// JSUMO switch variables
bool robotActive = false; // Will become true once and stay true
bool jsumoSwitchProcessed = false; // Flag to ensure switch is processed only once for activation
bool jsumoSignalActiveLastFrame = false;
unsigned long jsumoDebounceTime = 0;
const unsigned long JSUMO_MIN_INTERVAL = 200;

// Search state variables
enum SearchBotState {
  SEARCH_STATE_PIVOTING, // Changed from SPINNING
  SEARCH_STATE_INIT_FORWARD,
  SEARCH_STATE_MOVING_FORWARD,
  SEARCH_STATE_STOPPING_AFTER_FORWARD
};
SearchBotState currentSearchState = SEARCH_STATE_PIVOTING; // Initialize to pivoting
unsigned long searchStateTimer = 0;
const unsigned long SEARCH_FORWARD_DURATION = 500;
const unsigned long SEARCH_STOP_DURATION = 200;

// Special Turn State for US detection
enum SpecialTurnState {
  TURN_STATE_NONE,
  TURN_STATE_US_TRIGGERED_PIVOT_LEFT,  // Changed name
  TURN_STATE_US_TRIGGERED_PIVOT_RIGHT // Changed name
};
SpecialTurnState currentSpecialTurn = TURN_STATE_NONE;
unsigned long specialTurnStartTime = 0;
const unsigned long MAX_SPECIAL_TURN_DURATION = 4000; // Increased due to pivot potentially being slower

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0;
int currentRightMotorDirection = 0;
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 10; // Slightly increased for faster ramp with pivots
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 20;


// Function declarations
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd);
void updateMotorsWithRamping();
void moveForward(int speed);
void moveBackward(int speed);
void pivotLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);  // New pivot functions
void pivotRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER); // New pivot functions
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

  stopMovement();

  jsumoSignalActiveLastFrame = (digitalRead(JSUMO_SWITCH) == LOW);
  Serial.println("Setup Complete. Waiting for JSUMO Switch (IR Signal) for one-time activation.");
}

void loop() {
  handleJsumoSwitch(); // Will activate robotActive once

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovement();
      performInitialScan();

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE;
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_PIVOTING;
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
          Serial.println("Special Pivot: Target ACQUIRED by IR.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = true;
          lastOpponentDetectionTime = millis();
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Pivot: MAX DURATION reached.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false;
        } else {
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeft(SPEED_FAST); // Pivot left
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRight(SPEED_FAST); // Pivot right
          }
        }
      }
      
      if (currentSpecialTurn == TURN_STATE_NONE) { 
            updateBumpSensorState();
            executeStrategy();
      }
    }
  } else {
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE;
  }

  updateMotorsWithRamping();
}

// Modified handleJsumoSwitch for ONE-TIME activation
void handleJsumoSwitch() {
  if (jsumoSwitchProcessed || robotActive) { // If already processed or robot is active, do nothing.
    return;
  }

  bool pinIsCurrentlyLow = (digitalRead(JSUMO_SWITCH) == LOW);

  if (pinIsCurrentlyLow && !jsumoSignalActiveLastFrame) { // Detects LOW signal start
    if (millis() - jsumoDebounceTime > JSUMO_MIN_INTERVAL) {
      robotActive = true; // Activate the robot
      jsumoSwitchProcessed = true; // Mark switch as processed
      jsumoDebounceTime = millis();

      isPerformingInitialScan = true;
      initialScanStartTime = millis();
      Serial.println("Robot Activated by IR. Starting 3s scan. Will remain active.");

      isOpponentVisible = false;
      preferredDirection = 0;
      lastOpponentDetectionTime = 0;
      bumpSensorState = 0;
      currentSpecialTurn = TURN_STATE_NONE;
      currentSearchState = SEARCH_STATE_PIVOTING;
      stopMovement();
    }
  }
  jsumoSignalActiveLastFrame = pinIsCurrentlyLow;
}


void performInitialScan() {
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
    if (frontIRDetecting) {
        if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
        else if (irRight < irLeft && irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
        else preferredDirection = (random(2) == 0) ? -1 : 1;
    } else {
        preferredDirection = (random(2) == 0) ? -1 : 1;
    }
    Serial.println("Initial Scan: US is primary target. Preferred pivot direction set.");
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
    Serial.println("Strategy: US is primary target. Initiating SPECIAL PIVOT.");
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    int turnDecision = preferredDirection;
    if (turnDecision == 0) {
        turnDecision = (random(2) == 0) ? -1 : 1;
    }

    if (turnDecision <= 0) {
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_LEFT;
    } else {
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_RIGHT;
    }
    // Don't return here; let loop manage the pivot if state is set
  }


  if (currentSpecialTurn == TURN_STATE_NONE) { // Only proceed if not initiating/in a special turn
    if (frontIRDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else if (usDetecting) { // US detects, but wasn't "primary" for special turn
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      Serial.println("Strategy: US sees opponent (not primary for special pivot). Adjusting.");
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else { // No current detection
      if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
        Serial.println("Strategy: Lost opponent briefly. Reacquiring.");
        if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
        else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
        else moveForward(SPEED_MEDIUM);
      } else {
        isOpponentVisible = false;
        Serial.println("Strategy: Opponent lost or not found. Searching.");
        searchOpponent();
      }
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK);
  bool frontClearEnough = (leftDist >= IR_DETECTION_THRESHOLD &&
                           centerDist >= IR_DETECTION_THRESHOLD &&
                           rightDist >= IR_DETECTION_THRESHOLD);

  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontClearEnough) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust (Fallback): Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Pivoting FAST.");
      if (preferredDirection == 1) pivotRight(SPEED_FAST);
      else if (preferredDirection == -1) pivotLeft(SPEED_FAST);
      else pivotRight(SPEED_FAST); // Default pivot
      return;
  }

  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Pivoting/Moving.");
    // Pivot left: right wheel forward, left wheel slower forward or stopped
    setMotorTargets(1, leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW, // Left motor (inner)
                    1, SPEED_FAST);                                                       // Right motor (outer)
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Pivoting/Moving.");
    // Pivot right: left wheel forward, right wheel slower forward or stopped
    setMotorTargets(1, SPEED_FAST,                                                        // Left motor (outer)
                    1, rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW); // Right motor (inner)
    preferredDirection = 1;
  } else if (isOpponentVisible) {
      Serial.println("Adjust: Opponent was visible but IRs ambiguous. Pivoting or searching.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else searchOpponent();
  } else {
    Serial.println("Adjust: No clear opponent path, searching.");
    searchOpponent();
  }
}

// --- Helper function to set motor targets ---
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd) {
  targetLeftMotorDirection = leftDir;
  targetLeftSpeedMagnitude = constrain(abs(leftSpd), 0, 255);

  targetRightMotorDirection = rightDir;
  targetRightSpeedMagnitude = constrain(abs((int)(rightSpd * RIGHT_MOTOR_FACTOR)), 0, 255);
}

// --- Modified Movement Functions for Pivoting ---
void moveForward(int speed) {
  setMotorTargets(1, speed, 1, speed);
}

void moveBackward(int speed) {
  setMotorTargets(-1, speed, -1, speed);
}

void pivotLeft(int outerSpeed, int innerSpeed) { // Right motor (outer) forward, Left motor (inner) forward (slower) or stopped
  // To pivot left, M2 (right) is the outer wheel, M1 (left) is inner.
  // Both motors go "forward" in terms of their own directionality for this pivot.
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

void pivotRight(int outerSpeed, int innerSpeed) { // Left motor (outer) forward, Right motor (inner) forward (slower) or stopped
  // To pivot right, M1 (left) is outer, M2 (right) is inner.
  setMotorTargets(1, outerSpeed, 1, innerSpeed);
}

void stopMovement() {
  setMotorTargets(0, 0, 0, 0);
}


// --- Ramping Motor Control ---
void updateMotorsWithRamping() {
  unsigned long currentTime = millis();
  if (currentTime - lastMotorRampTime < MOTOR_RAMP_INTERVAL) {
    return;
  }
  lastMotorRampTime = currentTime;

  // Ramp Left Motor Speed
  if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
    currentActualLeftSpeed += ACCELERATION_STEP;
    if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed -= ACCELERATION_STEP;
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  }
  if (targetLeftSpeedMagnitude == 0 && currentActualLeftSpeed < ACCELERATION_STEP) currentActualLeftSpeed = 0;


  // Ramp Right Motor Speed
  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed += ACCELERATION_STEP;
    if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed -= ACCELERATION_STEP;
    if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  }
  if (targetRightSpeedMagnitude == 0 && currentActualRightSpeed < ACCELERATION_STEP) currentActualRightSpeed = 0;


  // Update Directions
  currentLeftMotorDirection = targetLeftMotorDirection;
  currentRightMotorDirection = targetRightMotorDirection;


  // --- Apply to M1 (Left Motor) ---
  if (currentLeftMotorDirection == 1) { // Forward for M1
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) { // Backward for M1
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else { // Stop M1
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
  }

  // --- Apply to M2 (Right Motor) ---
  // Assumes M2_LPWM makes M2 go FORWARD (typical for mirrored mount)
  // and M2_RPWM makes M2 go BACKWARD.
  // Target direction 1 means "forward contributing action for the robot".
  if (currentRightMotorDirection == 1) { // M2 contributes to robot's forward motion / outer wheel of left pivot
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // M2 contributes to robot's backward motion / outer wheel of right pivot (if spinning)
                                                // For pivotRight, M2 is inner wheel, so its "forward" is still with M2_LPWM.
                                                // This part of logic might need review based on exact pivot desired vs. spin.
                                                // For now, assuming -1 means M2 goes backward using M2_RPWM.
                                                // This means pivotRight will be M1 forward, M2 backward (a spin)
                                                // To make pivotRight M1 outer (fwd), M2 inner (fwd slow):
                                                // currentRightMotorDirection would be 1 for M2 (inner wheel moving forward)
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else { // Stop M2
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
  }
}


// --- Sensor Functions ---
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


void updateBumpSensorState() {
  bool leftBumped = (digitalRead(BUMP_LEFT) == LOW);
  bool rightBumped = (digitalRead(BUMP_RIGHT) == LOW);

  bumpSensorState = 0;
  if (leftBumped) bumpSensorState |= (1 << BUMP_LEFT_BIT);
  if (rightBumped) bumpSensorState |= (1 << BUMP_RIGHT_BIT);
}

// --- Strategy Sub-Functions ---
void searchOpponent() {
  unsigned long currentTime = millis();
  if (currentSearchState == SEARCH_STATE_PIVOTING && currentTime - lastScanTime > 2000) {
    scanDirection = -scanDirection;
    lastScanTime = currentTime;
    if (random(3) == 0) {
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_PIVOTING: // Changed from SPINNING
      if (scanDirection == 1) pivotRight(SPEED_MEDIUM); // Pivot right
      else pivotLeft(SPEED_MEDIUM);                     // Pivot left
      preferredDirection = scanDirection;
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW);
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
        currentSearchState = SEARCH_STATE_PIVOTING;
        lastScanTime = millis();
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX);
}