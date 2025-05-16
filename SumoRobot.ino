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
int scanDirection = 1;

int bumpSensorState = 0;

int preferredDirection = 0;
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
SearchBotState currentSearchState = SEARCH_STATE_SPINNING; // Initialize to spinning
unsigned long searchStateTimer = 0;
const unsigned long SEARCH_FORWARD_DURATION = 300; // ms
const unsigned long SEARCH_STOP_DURATION = 100;    // ms, short pause after moving


// Special Turn State for US detection
enum SpecialTurnState {
  TURN_STATE_NONE,
  TURN_STATE_US_TRIGGERED_LEFT,
  TURN_STATE_US_TRIGGERED_RIGHT
};
SpecialTurnState currentSpecialTurn = TURN_STATE_NONE;
unsigned long specialTurnStartTime = 0;
const unsigned long MAX_SPECIAL_TURN_DURATION = 3000;


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
      stopMovement();
      performInitialScan();

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE;
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_SPINNING;
      }
    } else { // Not in initial scan
      if (currentSpecialTurn != TURN_STATE_NONE) {
        // Using IR_DETECTION_THRESHOLD as the target for IR to acquire during special turn
        int irLeft = getIRDistance(IR_REFLECT_LEFT); // Default to thresholded max
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
          // Fall through to executeStrategy
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Turn: MAX DURATION reached. Giving up special turn.");
          stopMovement();
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; // Assume lost
          // Fall through to executeStrategy
        } else { // Continue special turn
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_LEFT) turnLeft(SPEED_FAST);
          else turnRight(SPEED_FAST);
          return; // Don't execute normal strategy while in special turn
        }
      }
      // If not in a special turn, or special turn just completed/aborted
      updateBumpSensorState();
      executeStrategy();
    }
  } else { // Robot not active
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE; // Reset special turn if robot is deactivated
  }
}

/*void handleJsumoSwitch() {
  bool currentJsumoSignalActive = (digitalRead(JSUMO_SWITCH) == LOW);

  if (currentJsumoSignalActive && !jsumoSignalActiveLastFrame && (millis() - jsumoDebounceTime > JSUMO_MIN_INTERVAL)) {
    robotActive = !robotActive;
    jsumoDebounceTime = millis();

    if (robotActive) {
      isPerformingInitialScan = true;
      initialScanStartTime = millis();
      Serial.println("Robot Activated by IR. Starting 3s scan.");
      isOpponentVisible = false;
      preferredDirection = 0;
      lastOpponentDetectionTime = 0;
      bumpSensorState = 0;
      currentSpecialTurn = TURN_STATE_NONE; // Reset on activation
      stopMovement();
    } else {
      isPerformingInitialScan = false;
      currentSpecialTurn = TURN_STATE_NONE; // Reset on deactivation
      stopMovement();
      Serial.println("Robot Deactivated by IR.");
    }
  }
  jsumoSignalActiveLastFrame = currentJsumoSignalActive;
}*/
void handleJsumoSwitch() {
  // If robot is already active, do nothing further with the switch.
  if (robotActive) {
    return;
  }

  // --- Logic to activate the robot (only if not already active) ---
  bool currentJsumoSignalActive = (digitalRead(JSUMO_SWITCH) == LOW);

  // Detect a falling edge (signal just became active: HIGH -> LOW)
  // And ensure enough time has passed since the last (attempted) toggle for debounce
  if (currentJsumoSignalActive && !jsumoSignalActiveLastFrame && (millis() - jsumoDebounceTime > JSUMO_MIN_INTERVAL)) {
    // Since robotActive is false at this point (due to the check above),
    // this will always be an activation event.
    robotActive = true; // Activate the robot
    jsumoDebounceTime = millis(); // Record time of activation for debounce

    // Robot is now turning ON
    isPerformingInitialScan = true;
    initialScanStartTime = millis();
    Serial.println("Robot Activated by IR. Starting 3s scan. Will remain active.");

    // Reset strategy variables for a fresh start
    isOpponentVisible = false;
    preferredDirection = 0;
    lastOpponentDetectionTime = 0;
    bumpSensorState = 0;
    currentSpecialTurn = TURN_STATE_NONE; // Reset special turn state
    stopMovement(); // Ensure motors are stopped before initial scan
  }

  // Update the last known state for the next loop iteration
  jsumoSignalActiveLastFrame = currentJsumoSignalActive;
}


void performInitialScan() {
  // Get raw distances, using actual max value for comparison if no detection
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  isOpponentVisible = false; // Reset for this scan pass

  // Find minimum front IR distance if any are detecting
  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }


  // PRIORITY: Ultrasonic is closer OR ultrasonic detects and front doesn't
  if (backDist < US_DETECTION_THRESHOLD_BACK && (backDist < minFrontIRDistance || !frontIRDetecting)) {
    isOpponentVisible = true; // Target is behind
    lastOpponentDetectionTime = millis();
    // Set preferred turn direction for special turn
    // A slight hint from IRs can be useful if opponent was briefly seen on one side
    if (frontIRDetecting) { // Check if front IRs provide any hint for turn direction
        if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
        else if (irRight < irLeft && irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
        else preferredDirection = (random(2) == 0) ? -1 : 1; // If front IRs are ambiguous, random
    } else {
        preferredDirection = (random(2) == 0) ? -1 : 1; // If no front IR hint, random
    }
    Serial.println("Initial Scan: US is primary target. Will special turn.");
    return; // US detection takes precedence for initial direction setting
  }

  // If US is not primary, check front IRs
  if (frontIRDetecting) {
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
  if (currentSpecialTurn != TURN_STATE_NONE) {
      // This should ideally not be reached if main loop handles special turn correctly
      // but as a safeguard:
      Serial.println("WARN: executeStrategy called while in special turn. Returning.");
      return;
  }

  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Bumper hit cancels special turn
    return;
  }

  // Get raw distances, using actual max value for comparison
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  // Determine minimum valid front IR detection distance
  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  bool usDetecting = (backDist < US_DETECTION_THRESHOLD_BACK);

  // *** PRIORITY LOGIC: US is closer OR US detects and front doesn't ***
  if (usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && currentSpecialTurn == TURN_STATE_NONE) {
    Serial.println("Strategy: US is primary target. Initiating SPECIAL TURN.");
    stopMovement(); // Stop briefly
    isOpponentVisible = true; // Opponent is presumed visible
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    // Use preferredDirection (set in initial scan or last known good direction)
    // to decide which way to turn first.
    if (preferredDirection <= 0) { // If last seen left or center, or random chose left
        currentSpecialTurn = TURN_STATE_US_TRIGGERED_LEFT;
        // turnLeft(SPEED_FAST); // Actual turning handled by main loop now
    } else { // preferredDirection > 0, last seen right or random chose right
        currentSpecialTurn = TURN_STATE_US_TRIGGERED_RIGHT;
        // turnRight(SPEED_FAST); // Actual turning handled by main loop now
    }
    return; // Main loop will now handle the special turn's movement
  }

  // If not initiating special US turn, proceed with IR or other cases
  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    // Pass raw values which will be compared against IR_DETECTION_THRESHOLD inside
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
  } else if (usDetecting) { // US detects, but wasn't deemed "primary" (e.g., front IR was closer but now lost)
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    Serial.println("Strategy: US sees opponent, front clear, but not primary. Adjusting (likely turn).");
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // Will likely turn
  } else { // No current detection
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
      Serial.println("Strategy: Lost opponent, trying to reacquire...");
      if (preferredDirection < 0) turnLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRight(SPEED_MEDIUM);
      else moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching...");
      searchOpponent();
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  // This function is called when IR is the primary focus, or as a fallback.
  // Distances passed are raw sensor readings (could be IR_MAX_DISTANCE_VAL or US_MAX_DISTANCE_VAL if no detection).

  // Fallback for US detection if special turn isn't active and US is very close or front is clear
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK);
  bool frontClearEnough = (leftDist >= IR_DETECTION_THRESHOLD && // Check against threshold here
                           centerDist >= IR_DETECTION_THRESHOLD &&
                           rightDist >= IR_DETECTION_THRESHOLD);

  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontClearEnough) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust (Fallback): Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Turning FAST.");
      if (preferredDirection == 1) turnRight(SPEED_FAST);
      else if (preferredDirection == -1) turnLeft(SPEED_FAST);
      else turnRight(SPEED_FAST); // Default
      preferredDirection = (scanDirection > 0) ? 1 : -1; // Update preferred direction based on turn
      return;
  }

  // Standard IR-based adjustment, using actual detection thresholds
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm).");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM,
                centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm).");
    moveForward(leftDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM,
                leftDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm).");
    moveForward(rightDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_FAST,
                rightDist < IR_CLOSE_THRESHOLD ? SPEED_SLOW : SPEED_MEDIUM);
    preferredDirection = 1;
  } else if (isOpponentVisible) { // Opponent was visible, but current IR readings are not decisive
      Serial.println("Adjust: Opponent visible but IRs ambiguous. Using preferredDirection or searching.");
      if (preferredDirection < 0) turnLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) turnRight(SPEED_MEDIUM);
      else searchOpponent();
  } else {
    // This case should ideally be covered by executeStrategy's "search" path
    Serial.println("Adjust: No clear opponent path, searching.");
    searchOpponent();
  }
}


// --- Movement Functions ---
void moveForward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, constrain(leftSpeed, 0, SPEED_MAX));
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_LPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX));
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, constrain(leftSpeed, 0, SPEED_MAX));
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_RPWM, constrain(adjustedRightSpeed, 0, SPEED_MAX));
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int turnSpeed) {
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_LPWM, speed);
  analogWrite(M1_RPWM, 0);
  analogWrite(M2_LPWM, adjustedSpeed);
  analogWrite(M2_RPWM, 0);
}

void turnRight(int turnSpeed) {
  int speed = constrain(turnSpeed, 0, SPEED_MAX);
  int adjustedSpeed = (int)(speed * RIGHT_MOTOR_FACTOR);
  analogWrite(M1_RPWM, speed);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, adjustedSpeed);
  analogWrite(M2_LPWM, 0);
}

void stopMovement() {
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);
}

// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);

  // Consistent check for too low voltage (too far or no object)
  if (voltage < 0.42) return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD;

  float inverse_distance_factor = (voltage - 0.42) / 42.5;
  // Consistent check for non-positive inverse factor
  if (inverse_distance_factor <= 1e-9) return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD;

  float distance = 1.0 / inverse_distance_factor;

  // Use IR_DETECTION_THRESHOLD for non-actualMaxVal comparison to simplify logic elsewhere
  int comparisonMax = actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD;

  if (distance > comparisonMax) return comparisonMax;
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

  // Use US_DETECTION_THRESHOLD_BACK for non-actualMaxVal comparison
  long comparisonMax = actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK;

  if (distance == 0 || distance > comparisonMax ) {
     return comparisonMax;
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

  // Only change spin direction or decide to move forward if in spinning state
  if (currentSearchState == SEARCH_STATE_SPINNING && currentTime - lastScanTime > 2000) {
    scanDirection = -scanDirection; // Toggle scan direction
    lastScanTime = currentTime;
    if (random(3) == 0) { // 1 in 3 chance to initiate forward movement
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_SPINNING:
      if (scanDirection == 1) {
        turnRight(SPEED_MEDIUM);
      } else {
        turnLeft(SPEED_MEDIUM);
      }
      preferredDirection = scanDirection; // Update preferredDirection during search spin
      break;

    case SEARCH_STATE_INIT_FORWARD:
      // Serial.println("Search: Init Forward.");
      moveForward(SPEED_SLOW, SPEED_SLOW);
      searchStateTimer = millis();
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      break;

    case SEARCH_STATE_MOVING_FORWARD:
      // Continue moving forward (action already started in INIT_FORWARD)
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        // Serial.println("Search: Stopping Forward.");
        stopMovement();
        searchStateTimer = millis();
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
      }
      break;

    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      // Pausing after forward movement (motors are already stopped)
      if (millis() - searchStateTimer >= SEARCH_STOP_DURATION) {
        // Serial.println("Search: Back to Spinning.");
        currentSearchState = SEARCH_STATE_SPINNING; // Go back to spinning
        lastScanTime = millis(); // Reset scan timer to avoid immediate direction change
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX, SPEED_MAX);
}