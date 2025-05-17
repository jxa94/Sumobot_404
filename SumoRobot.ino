#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right (M2) - Assuming M2_LPWM is FORWARD due to mirrored mounting
#define M2_RPWM 10 // Typically Backward for M2 if M2_LPWM is Forward
#define M2_LPWM 11 // Typically Forward for M2 if mounted mirrored to M1

// Motor left (M1)
#define M1_RPWM 5 // Forward for M1
#define M1_LPWM 6 // Backward for M1

// Starter switch: Digital input (MicroStart Module Signal Pin)
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
const int SPEED_TURN_PIVOT_INNER = 10; // Speed for the inner wheel during a standard pivot
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150; // Used for pivot outer wheel speed
const int SPEED_MAX = 200;

const float RIGHT_MOTOR_FACTOR = 1.0;

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999;
const int IR_MIN_DISTANCE = 10; // Min reliable distance for IR
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

// JSUMO MicroStart switch variables
bool robotActive = false; 
bool jsumoSignalWasHighLastFrame = false; 

// Search state variables
enum SearchBotState {
  SEARCH_STATE_PIVOTING,
  SEARCH_STATE_INIT_FORWARD,
  SEARCH_STATE_MOVING_FORWARD,
  SEARCH_STATE_STOPPING_AFTER_FORWARD
};
SearchBotState currentSearchState = SEARCH_STATE_PIVOTING;
unsigned long searchStateTimer = 0;
const unsigned long SEARCH_FORWARD_DURATION = 500;
const unsigned long SEARCH_STOP_DURATION = 200;

// Special Turn State for US detection
enum SpecialTurnState {
  TURN_STATE_NONE,
  TURN_STATE_US_TRIGGERED_PIVOT_LEFT,
  TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
};
SpecialTurnState currentSpecialTurn = TURN_STATE_NONE;
unsigned long specialTurnStartTime = 0;
const unsigned long MAX_SPECIAL_TURN_DURATION = 4000;

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0; // 0=Stop, 1=Forward, -1=Backward
int currentRightMotorDirection = 0; // 0=Stop, 1=Forward, -1=Backward
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 10; 
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 20; // Milliseconds

bool bypassRampTimerOnce = false; // Flag to bypass ramp timer for one cycle for immediate actions

// Function declarations
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd);
void setMotorTargetsImmediately(int leftDir, int leftSpd, int rightDir, int rightSpd); // New
void updateMotorsWithRamping();
void moveForward(int speed);
void moveBackward(int speed);
void pivotLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void pivotRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void pivotLeftImmediate(int outerSpeed, int innerSpeed = 0); // New, defaults inner to 0
void pivotRightImmediate(int outerSpeed, int innerSpeed = 0); // New, defaults inner to 0
void stopMovement();
void stopMovementImmediately(); // New

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

  stopMovementImmediately(); // Start with motors definitely off, immediately

  jsumoSignalWasHighLastFrame = (digitalRead(JSUMO_SWITCH) == HIGH);
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
}

void loop() {
  handleJsumoSwitch(); 

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovementImmediately(); // Ensure motors are stopped during the initial scan phase
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
          stopMovementImmediately(); // Immediate stop
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = true;
          lastOpponentDetectionTime = millis();
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Pivot: MAX DURATION reached.");
          stopMovementImmediately(); // Immediate stop
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false;
        } else {
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeftImmediate(SPEED_FAST, 0); // Pivot left, inner wheel (left) speed 0
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRightImmediate(SPEED_FAST, 0); // Pivot right, inner wheel (right) speed 0
          }
        }
      }
      
      if (currentSpecialTurn == TURN_STATE_NONE) { 
            updateBumpSensorState();
            executeStrategy();
      }
    }
  } else { 
    stopMovementImmediately(); // Immediate stop
    currentSpecialTurn = TURN_STATE_NONE; 
    isPerformingInitialScan = false; 
  }

  updateMotorsWithRamping();
}

void handleJsumoSwitch() {
  bool signalIsCurrentlyHigh = (digitalRead(JSUMO_SWITCH) == HIGH);

  if (signalIsCurrentlyHigh && !jsumoSignalWasHighLastFrame) { 
    if (!robotActive) { 
      Serial.println("Robot Activated by MicroStart (Signal HIGH). Starting initial scan.");
      robotActive = true;
      isPerformingInitialScan = true; 
      initialScanStartTime = millis();
      isOpponentVisible = false;
      preferredDirection = 0;
      lastOpponentDetectionTime = 0;
      bumpSensorState = 0;
      currentSpecialTurn = TURN_STATE_NONE;
      currentSearchState = SEARCH_STATE_PIVOTING; 
      stopMovementImmediately(); 
    }
  } else if (!signalIsCurrentlyHigh && jsumoSignalWasHighLastFrame) { 
    if (robotActive) { 
      Serial.println("Robot Deactivated by MicroStart (Signal LOW).");
      robotActive = false;
      isPerformingInitialScan = false; 
      stopMovementImmediately(); 
    }
  }
  jsumoSignalWasHighLastFrame = signalIsCurrentlyHigh; 
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
    attackWithBumpers(); // This uses moveForward which ramps. Could be immediate if needed.
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Ensure special turn is cancelled
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
    // loop() will now handle the pivot action
    return; // Exit strategy to let special turn execute in loop()
  }


  if (currentSpecialTurn == TURN_STATE_NONE) { // Only proceed if not initiating/in a special turn
    if (frontIRDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else if (usDetecting) { // US detects, but wasn't "primary" for special turn (e.g., front IR was closer)
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      Serial.println("Strategy: US sees opponent (not primary for special pivot). Adjusting based on overall sensor view.");
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // Let adjustDirection decide
    } else { // No current detection by IR or US
      if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
        Serial.println("Strategy: Lost opponent briefly. Reacquiring based on last preferred direction.");
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
  bool frontSensorsClear = (leftDist >= IR_DETECTION_THRESHOLD &&
                            centerDist >= IR_DETECTION_THRESHOLD &&
                            rightDist >= IR_DETECTION_THRESHOLD);

  // Priority: If opponent is very close behind OR behind and front is clear, pivot to face them
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontSensorsClear) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust: Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Pivoting FAST to engage.");
      // Using immediate pivots to quickly face the opponent behind
      if (preferredDirection == 1) pivotRightImmediate(SPEED_FAST, 0);      
      else if (preferredDirection == -1) pivotLeftImmediate(SPEED_FAST, 0); 
      else { 
          // If no strong preference, or preferred was center, default to one direction
          // (e.g. if US detected directly behind, robot might not know which way to turn)
          // A fixed default like pivotRightImmediate or a random choice can be used.
          pivotRightImmediate(SPEED_FAST, 0); // Defaulting to pivot right to engage rear target
      }
      return; // Action taken
  }

  // Next, address front IR sensors
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Pivoting/Moving towards left.");
    // Standard pivot, inner wheel slower for turning towards opponent
    setMotorTargets(1, leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW, 1, SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Pivoting/Moving towards right.");
    // Standard pivot
    setMotorTargets(1, SPEED_FAST, 1, rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW);
    preferredDirection = 1;
  } else if (isOpponentVisible) { // IRs are ambiguous but opponent was recently seen
      Serial.println("Adjust: Opponent was visible but current IRs ambiguous. Pivoting based on last known direction or searching.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else searchOpponent(); // If last direction was center or unknown
  } else { // No clear opponent, and not recently visible
    Serial.println("Adjust: No clear opponent path, resorting to search.");
    searchOpponent();
  }
}

// --- Helper function to set motor targets (for ramping) ---
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd) {
  targetLeftMotorDirection = leftDir;
  targetLeftSpeedMagnitude = constrain(abs(leftSpd), 0, 255);

  targetRightMotorDirection = rightDir;
  targetRightSpeedMagnitude = constrain(abs((int)(rightSpd * RIGHT_MOTOR_FACTOR)), 0, 255);
}

// --- Helper function to set motor targets and current speeds immediately ---
void setMotorTargetsImmediately(int leftDir, int leftSpd, int rightDir, int rightSpd) {
  targetLeftMotorDirection = leftDir;
  targetLeftSpeedMagnitude = constrain(abs(leftSpd), 0, 255);
  targetRightMotorDirection = rightDir;
  targetRightSpeedMagnitude = constrain(abs((int)(rightSpd * RIGHT_MOTOR_FACTOR)), 0, 255);

  // Immediately set current actuals to targets
  currentActualLeftSpeed = targetLeftSpeedMagnitude;
  currentLeftMotorDirection = targetLeftMotorDirection;
  currentActualRightSpeed = targetRightSpeedMagnitude;
  currentRightMotorDirection = targetRightMotorDirection;
  
  bypassRampTimerOnce = true; // Signal updateMotorsWithRamping to run and apply these values now
}


// --- Movement Functions ---
void moveForward(int speed) {
  setMotorTargets(1, speed, 1, speed);
}

void moveBackward(int speed) {
  setMotorTargets(-1, speed, -1, speed);
}

// Standard pivot (ramped)
void pivotLeft(int outerSpeed, int innerSpeed) { // innerSpeed defaults in declaration
  setMotorTargets(1, innerSpeed, 1, outerSpeed); // M1 (Left) is inner, M2 (Right) is outer
}

void pivotRight(int outerSpeed, int innerSpeed) { // innerSpeed defaults in declaration
  setMotorTargets(1, outerSpeed, 1, innerSpeed); // M1 (Left) is outer, M2 (Right) is inner
}

// Immediate pivot (no ramp, inner wheel typically 0 for special turn)
void pivotLeftImmediate(int outerSpeed, int innerSpeed) { // innerSpeed defaults in declaration
  setMotorTargetsImmediately(1, innerSpeed, 1, outerSpeed);
}

void pivotRightImmediate(int outerSpeed, int innerSpeed) { // innerSpeed defaults in declaration
  setMotorTargetsImmediately(1, outerSpeed, 1, innerSpeed);
}

void stopMovement() {
  setMotorTargets(0, 0, 0, 0);
}

void stopMovementImmediately() {
  setMotorTargetsImmediately(0, 0, 0, 0);
}


// --- Ramping Motor Control ---
void updateMotorsWithRamping() {
  unsigned long currentTime = millis();
  
  if (!bypassRampTimerOnce && (currentTime - lastMotorRampTime < MOTOR_RAMP_INTERVAL)) {
    return; // Skip update if not enough time passed and no immediate request
  }
  lastMotorRampTime = currentTime; // An update is happening
  bypassRampTimerOnce = false;    // Reset the flag

  // If setMotorTargetsImmediately was called, currentActualSpeeds are already at targetSpeeds.
  // So, the ramping logic below will effectively be skipped for that command.

  // Left Motor Ramping
  if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
    currentActualLeftSpeed += ACCELERATION_STEP;
    if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed -= ACCELERATION_STEP;
    // Clamp to target, handles positive and zero targets correctly
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude; 
  }
  // If targetLeftSpeedMagnitude is 0, the above clamp ensures currentActualLeftSpeed becomes 0.

  // Right Motor Ramping
  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed += ACCELERATION_STEP;
    if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed -= ACCELERATION_STEP;
    // Clamp to target, handles positive and zero targets correctly
    if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  }
  // If targetRightSpeedMagnitude is 0, the above clamp ensures currentActualRightSpeed becomes 0.

  // Update motor directions (these are set by setMotorTargets or setMotorTargetsImmediately)
  currentLeftMotorDirection = targetLeftMotorDirection;
  currentRightMotorDirection = targetRightMotorDirection;

  // Apply to Motors
  if (currentLeftMotorDirection == 1) { // Forward
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) { // Backward
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else { // Stop
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
  }

  if (currentRightMotorDirection == 1) { // Forward (M2_LPWM is Forward for right motor)
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // Backward (M2_RPWM is Backward for right motor)
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else { // Stop
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
  }
}


// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);
  float distance;

  // This calibration is specific and might need tuning for actual sensors
  if (voltage < 0.42) { 
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  } else if (voltage > 2.8) { 
      distance = 5;
  } else if (voltage > 1.5) { 
      distance = 10;
  } else if (voltage > 0.8) { 
      distance = 20;
  } else if (voltage > 0.42) { 
      distance = 35; 
  } else { 
      distance = IR_MAX_DISTANCE_VAL;
  }

  if (distance > (actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1) ) {
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  }
  // Ensure distance does not go below a practical minimum if calibration is tricky at very close range
  if (distance < IR_MIN_DISTANCE && distance > 0) return IR_MIN_DISTANCE; 
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
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
      searchStateTimer = currentTime; 
      Serial.println("Search: Switching to short forward burst.");
      return; 
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_PIVOTING:
      if (scanDirection == 1) {
        pivotRight(SPEED_MEDIUM); // Ramped pivot for searching
        Serial.println("Search: Pivoting Right.");
      } else {
        pivotLeft(SPEED_MEDIUM); // Ramped pivot for searching
        Serial.println("Search: Pivoting Left.");
      }
      preferredDirection = scanDirection; 
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW); // Ramped forward
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      // searchStateTimer was set when transitioning to INIT_FORWARD
      break;
    case SEARCH_STATE_MOVING_FORWARD:
      moveForward(SPEED_SLOW); 
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        stopMovement(); // Ramped stop
        searchStateTimer = millis(); 
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
        Serial.println("Search: Finished forward burst, stopping briefly.");
      }
      break;
    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      stopMovement(); 
      if (millis() - searchStateTimer >= SEARCH_STOP_DURATION) {
        currentSearchState = SEARCH_STATE_PIVOTING; 
        lastScanTime = millis(); 
        Serial.println("Search: Resuming pivot.");
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  // For maximum aggression, you might want an immediate full forward:
  // setMotorTargetsImmediately(1, SPEED_MAX, 1, SPEED_MAX);
  moveForward(SPEED_MAX); // Uses standard ramping
}