#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right (M2)
#define M2_RPWM 10
#define M2_LPWM 11

// Motor left (M1)
#define M1_RPWM 5
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

const float RIGHT_MOTOR_FACTOR = 1.0;

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
void turnLeft(int turnSpeed);
void turnRight(int turnSpeed);
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
  Serial.println("Setup Complete. Waiting for JSUMO Switch (IR Signal).");
}

void loop() {
  handleJsumoSwitch();

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovement(); // Keep robot stopped during initial scan phase
      performInitialScan();

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE; // Ensure special turn is reset after initial scan
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_SPINNING; // Default to spinning search
      }
    } else {
      if (currentSpecialTurn != TURN_STATE_NONE) {
        // Handle the special turn initiated by US sensor
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
          isOpponentVisible = true; // Opponent is now visible to front IR
          lastOpponentDetectionTime = millis();
          // Update preferred direction based on IR acquisition
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
          // Fall through to executeStrategy in the same loop iteration
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Turn: MAX DURATION reached. Giving up special turn.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; // Assume opponent lost if turn timed out
          // Fall through to executeStrategy
        } else {
          // Continue executing the special turn
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_LEFT) {
            turnLeft(SPEED_FAST);
          } else { // TURN_STATE_US_TRIGGERED_RIGHT
            turnRight(SPEED_FAST);
          }
          return; // IMPORTANT: Continue special turn, skip rest of loop() for this iteration
        }
      }
      // If not in a special turn, or special turn just ended, proceed.
      updateBumpSensorState();
      executeStrategy();
    }
  } else {
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE; // Ensure special turn is off if robot is deactivated
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
      stopMovement(); // Ensure robot is stopped at activation
    } else {
      isPerformingInitialScan = false;
      currentSpecialTurn = TURN_STATE_NONE;
      stopMovement();
      Serial.println("Robot Deactivated by IR.");
    }
  }
  jsumoSignalActiveLastFrame = currentJsumoSignalActive;
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
    // Determine preferred turn direction for the special turn (180-degree turn)
    // If front IRs give a hint, turn towards that side to face it after the 180.
    // Otherwise, pick a random direction for the 180.
    if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1; // Suggests opponent slightly to front-left
    else if (irRight < irLeft && irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1; // Suggests opponent slightly to front-right
    else preferredDirection = (random(2) == 0) ? -1 : 1; // No clear front IR hint, pick random for 180
    Serial.println("Initial Scan: US is primary target. Preferred turn for 180 set.");
    // No return here, let the initial scan time complete. executeStrategy will pick this up.
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
  // If nothing detected, isOpponentVisible remains false, preferredDirection might be from a previous faint hint or random.
}


void executeStrategy() {
  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Bumper hit overrides special turn
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

  // *** PRIORITY LOGIC: US is closer OR US detects and front doesn't ***
  if (usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && currentSpecialTurn == TURN_STATE_NONE) {
    Serial.println("Strategy: US is primary target. Initiating SPECIAL TURN.");
    // stopMovement(); // <<<< KEY CHANGE: DO NOT STOP HERE. Let the loop handle the turn.
    isOpponentVisible = true; // Opponent is considered visible (behind)
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    // Use preferredDirection (set in initial scan or by previous logic) to decide turn direction
    // If preferredDirection is 0 (e.g. from front-center IR), pick a default or random turn.
    // For US triggered turn, we need a clear left or right.
    int turnDecision = preferredDirection;
    if (turnDecision == 0) { // If preferred was center or undefined from IRs
        turnDecision = (random(2) == 0) ? -1 : 1; // Default to a random turn for the 180
    }

    if (turnDecision <= 0) { // Includes -1 and the randomized -1 from above
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_LEFT;
    } else { // Includes 1 and the randomized 1 from above
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_RIGHT;
    }
    return; // IMPORTANT: Exit executeStrategy, let main loop's special turn logic take over
  }

  // If not initiating special US turn, proceed with IR or other cases
  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
  } else if (usDetecting) { // US detects, but wasn't "primary" (e.g., front IR was closer but now lost, or special turn not active)
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    Serial.println("Strategy: US sees opponent (not primary for special turn), front clear. Adjusting.");
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // Will likely turn based on US fallback in adjustDir
  } else { // No current detection
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
      Serial.println("Strategy: Lost opponent briefly, trying to reacquire using last known direction.");
      if (preferredDirection < 0) turnLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRight(SPEED_MEDIUM);
      else moveForward(SPEED_MEDIUM, SPEED_MEDIUM); // If last saw center, cautiously move forward
    } else {
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching...");
      searchOpponent();
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK);
  bool frontClearEnough = (leftDist >= IR_DETECTION_THRESHOLD &&
                           centerDist >= IR_DETECTION_THRESHOLD &&
                           rightDist >= IR_DETECTION_THRESHOLD);

  // Fallback for US detection if special turn isn't active and US is very close or front is clear
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontClearEnough) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust (Fallback): Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Turning FAST.");
      // If preferredDirection is already set (e.g. from IR just before losing front contact), use it. Otherwise, pick one.
      if (preferredDirection == 1) {
          turnRight(SPEED_FAST);
          // preferredDirection = 1; // Already 1
      } else if (preferredDirection == -1) {
          turnLeft(SPEED_FAST);
          // preferredDirection = -1; // Already -1
      } else { // preferredDirection was 0 or uninformative, default to a turn (e.g., right)
          turnRight(SPEED_FAST);
          preferredDirection = 1;  // Update preferredDirection based on the turn taken
      }
      return;
  }

  // Standard IR-based adjustment
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM,
                centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Turning/Moving.");
    moveForward(leftDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM,
                leftDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Turning/Moving.");
    moveForward(rightDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST,
                rightDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM);
    preferredDirection = 1;
  } else if (isOpponentVisible) {
      Serial.println("Adjust: Opponent was visible but IRs ambiguous now. Using last preferredDirection or searching.");
      if (preferredDirection < 0) turnLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRight(SPEED_MEDIUM);
      else searchOpponent(); // If preferred was center and now lost, search
  } else {
    Serial.println("Adjust: No clear opponent path, searching.");
    searchOpponent();
  }
}


int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);

  // Sharp GP2Y0A41SK0F: 4cm to 30cm. Voltage increases as distance decreases.
  // Typical values: ~2.5-3V at 4cm, ~0.4V at 30cm. Below 0.4V is likely >30cm or no object.
  // Let's assume a simplified linear inverse model for distances in the detectable range,
  // or use a lookup table / polynomial fit if more accuracy is needed.
  // For this example, let's refine the previous model.
  // V = k/d + offset  =>  1/d = (V - offset)/k => d = k/(V-offset)
  // From datasheet graph: at 5cm, V ~ 2.25V. At 30cm, V ~ 0.4V.
  // If voltage < 0.4, distance is "max" (out of range)
  if (voltage < 0.42) { // Adjusted slightly based on common observations for "no detection"
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1; // Return value > threshold
  }
  // Simplified calculation (NEEDS CALIBRATION FOR YOUR SPECIFIC SENSOR AND SETUP)
  // This is a placeholder and likely needs significant adjustment.
  // Using a rough fit for common Sharp IR sensors like GP2Y0A21YK0F (10-80cm) or GP2Y0A41SK0F (4-30cm)
  // For GP2Y0A41SK0F (4-30cm) - More sensitive at close range
  // A common formula seen is distance = A / (reading + B) - C or similar based on ADC values
  // Or for voltage: distance = K / (Voltage - V0)
  // Let's try to stick to the formula structure: d = 1 / ((V - V_offset) / Factor)
  // (voltage - 0.3) is a common starting point for V_offset.
  // If at 5cm, V=2.25. If at 30cm, V=0.4.
  // Let's use a simple power curve fit like: distance = pow(Voltage / A, 1/B)
  // Or more commonly distance = X * pow(analogReadValue, Y)
  // For now, retaining the previous simplified inverse model structure:
  float distance = 13 * pow(voltage, -1.10); // Example from some online sources for GP2Y0A21
                                            // THIS IS A GUESS AND NEEDS CALIBRATION
                                            // For GP2Y0A41SK0F (4-30cm) it would be different.
                                            // Let's use a more generic conceptual inverse logic:
  if (voltage > 2.8) distance = 5;       // Very close
  else if (voltage > 1.5) distance = 10; // Close
  else if (voltage > 0.8) distance = 20; // Medium
  else if (voltage > 0.42) distance = 35; // Far
  else distance = IR_MAX_DISTANCE_VAL;    // Out of range


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

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); // 25ms timeout
  long distance = duration * 0.034 / 2;

  if (distance == 0 || distance > (actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1) ) {
     return actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1;
  }
  return distance;
}

void moveForward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, constrain(leftSpeed, 0, SPEED_MAX));
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_LPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX)); // M2_LPWM for forward on right motor
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, constrain(leftSpeed, 0, SPEED_MAX));
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_RPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX)); // M2_RPWM for backward on right motor
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int turnSpeed) { // Left motor backward, Right motor forward
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, speed); // Left backward
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_LPWM, adjustedSpeed); // Right forward
  analogWrite(M2_RPWM, 0);
}

void turnRight(int turnSpeed) { // Left motor forward, Right motor backward
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, speed); // Left forward
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, adjustedSpeed); // Right backward
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

void searchOpponent() {
  unsigned long currentTime = millis();

  // Change spin direction periodically or if stuck
  if (currentSearchState == SEARCH_STATE_SPINNING && currentTime - lastScanTime > 2000) { // Spin for 2s then change
    scanDirection = -scanDirection; // Reverse spin direction
    lastScanTime = currentTime;
    if (random(3) == 0) { // Occasionally try moving forward
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_SPINNING:
      if (scanDirection == 1) turnRight(SPEED_MEDIUM);
      else turnLeft(SPEED_MEDIUM);
      preferredDirection = scanDirection; // Keep preferredDirection aligned with spin
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
        currentSearchState = SEARCH_STATE_SPINNING; // Go back to spinning
        lastScanTime = millis(); // Reset scan timer for spinning
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX, SPEED_MAX);
}