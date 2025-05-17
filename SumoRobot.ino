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

// Constants
const int SPEED_TURN_PIVOT_INNER = 10; // Speed for the inner wheel during an ARC turn
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150; // Used for pivot outer wheel speed & zero-radius pivot speed
const int SPEED_MAX = 200;

const float RIGHT_MOTOR_FACTOR = 1.0;

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999;
const int IR_MIN_DISTANCE = 10; // Minimum reliable distance for IR sensors
const int IR_DETECTION_THRESHOLD = 70; // Beyond this, opponent is considered not detected by an IR sensor
const int IR_CLOSE_THRESHOLD = 30;    // Closer than this, opponent is very close

// Global variables for robot state
unsigned long initialScanStartTime = 0;
bool isPerformingInitialScan = false;

unsigned long lastScanTime = 0;
int scanDirection = 1; // 1 for right, -1 for left

int bumpSensorState = 0;

int preferredDirection = 0; // -1 for left, 0 for center, 1 for right
unsigned long lastOpponentDetectionTime = 0;
bool isOpponentVisible = false;

// JSUMO MicroStart switch variables
bool robotActive = false; // True if the robot is supposed to be running (MicroStart signal is HIGH)
bool jsumoSignalWasHighLastFrame = false; // Tracks the previous state of the JSUMO_SWITCH signal

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

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0; // 0=stop, 1=forward, -1=backward
int currentRightMotorDirection = 0; // 0=stop, 1=forward, -1=backward
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 10;
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 20;


// Function declarations
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd);
void updateMotorsWithRamping();
void moveForward(int speed);
void moveBackward(int speed);

void arcLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void arcRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void pivotLeft(int speed);
void pivotRight(int speed);

void stopMovement();

int getIRDistance(int sensorPin, bool actualMaxVal = false);
void updateBumpSensorState();
void searchOpponent();
void attackWithBumpers();
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch();
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist);


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

  pinMode(BUMP_LEFT, INPUT_PULLUP);
  pinMode(BUMP_RIGHT, INPUT_PULLUP);

  pinMode(JSUMO_SWITCH, INPUT_PULLUP);

  stopMovement();
  updateMotorsWithRamping(); // Initialize motor states

  jsumoSignalWasHighLastFrame = (digitalRead(JSUMO_SWITCH) == HIGH);
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
}

void loop() {
  handleJsumoSwitch();

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovement(); // Ensure motors are stopped during the initial scan phase
      performInitialScan(); // This function only gathers data, doesn't move

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_PIVOTING; // Start searching if nothing found
      }
    } else { // Main operational logic
      updateBumpSensorState();
      executeStrategy();
    }
  } else { // robotActive is false
    stopMovement();
    isPerformingInitialScan = false; // Ensure scan flag is also reset
  }

  updateMotorsWithRamping(); // Apply motor targets with ramping
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
      currentSearchState = SEARCH_STATE_PIVOTING;
      stopMovement(); // Ensure motors are reset at start
    }
  } else if (!signalIsCurrentlyHigh && jsumoSignalWasHighLastFrame) {
    if (robotActive) {
      Serial.println("Robot Deactivated by MicroStart (Signal LOW).");
      robotActive = false;
      isPerformingInitialScan = false;
      stopMovement();
    }
  }
  jsumoSignalWasHighLastFrame = signalIsCurrentlyHigh;
}


void performInitialScan() {
  // This function ONLY reads sensors and sets flags like isOpponentVisible and preferredDirection.
  // It does NOT command any motor movements. Motors are stopped by loop() during this phase.
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);

  isOpponentVisible = false; // Reset visibility at the start of scan

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

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
  } else {
    isOpponentVisible = false; // Explicitly set if no IR detection
    preferredDirection = (random(2) == 0) ? -1 : 1; // Set a random pivot direction for subsequent search
    Serial.println("Initial Scan: No opponent detected by IR. Random pivot direction set for search.");
  }
}


void executeStrategy() {
  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    return;
  }

  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    adjustDirectionToOpponent(irLeft, irCenter, irRight);
  } else { // No front IR detection
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) { // Lost opponent briefly
      Serial.println("Strategy: Lost opponent briefly. Reacquiring based on last preferred direction.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else moveForward(SPEED_MEDIUM); // If last seen centered, cautiously move forward
    } else {
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching.");
      searchOpponent(); // This will make the robot turn around
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist) {
  // If opponent is in front
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Arc turning left.");
    arcLeft(SPEED_FAST, (leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Arc turning right.");
    arcRight(SPEED_FAST, (rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = 1;
  } else if (isOpponentVisible) { // IRs are ambiguous but opponent was recently seen
      Serial.println("Adjust: Opponent was visible but current IRs ambiguous. Pivoting based on last known direction or searching.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else searchOpponent(); // If last known was center, and now ambiguous, search
  } else {
    // No clear opponent path from front IRs
    Serial.println("Adjust: No clear opponent path, resorting to search.");
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

// --- Movement Functions ---
void moveForward(int speed) {
  setMotorTargets(1, speed, 1, speed);
}

void moveBackward(int speed) {
  setMotorTargets(-1, speed, -1, speed);
}

void arcLeft(int outerSpeed, int innerSpeed) {
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

void arcRight(int outerSpeed, int innerSpeed) {
  setMotorTargets(1, outerSpeed, 1, innerSpeed);
}

void pivotLeft(int speed) {
  setMotorTargets(-1, speed, 1, speed);
}

void pivotRight(int speed) {
  setMotorTargets(1, speed, -1, speed);
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

  // Ramp Left Motor
  if (currentLeftMotorDirection != targetLeftMotorDirection && targetLeftSpeedMagnitude > 0) {
    if (currentActualLeftSpeed > 0) {
        currentActualLeftSpeed -= ACCELERATION_STEP * 2;
        if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
    } else {
        currentLeftMotorDirection = targetLeftMotorDirection;
    }
  } else if (currentLeftMotorDirection != targetLeftMotorDirection && targetLeftSpeedMagnitude == 0){
    currentLeftMotorDirection = targetLeftMotorDirection;
  }


  if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
    currentActualLeftSpeed += ACCELERATION_STEP;
    if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed -= ACCELERATION_STEP;
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  }
  if (targetLeftSpeedMagnitude == 0 && currentActualLeftSpeed < ACCELERATION_STEP / 2) { // Quick stop to 0
      currentActualLeftSpeed = 0;
  }
  currentActualLeftSpeed = constrain(currentActualLeftSpeed, 0, 255);


  // Ramp Right Motor
  if (currentRightMotorDirection != targetRightMotorDirection && targetRightSpeedMagnitude > 0) {
    if (currentActualRightSpeed > 0) {
        currentActualRightSpeed -= ACCELERATION_STEP * 2;
        if (currentActualRightSpeed < 0) currentActualRightSpeed = 0;
    } else {
        currentRightMotorDirection = targetRightMotorDirection;
    }
  } else if (currentRightMotorDirection != targetRightMotorDirection && targetRightSpeedMagnitude == 0){
      currentRightMotorDirection = targetRightMotorDirection;
  }


  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed += ACCELERATION_STEP;
    if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed -= ACCELERATION_STEP;
    if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  }
   if (targetRightSpeedMagnitude == 0 && currentActualRightSpeed < ACCELERATION_STEP / 2) { // Quick stop to 0
      currentActualRightSpeed = 0;
  }
  currentActualRightSpeed = constrain(currentActualRightSpeed, 0, 255);


  // Apply to Left Motor (M1)
  if (currentLeftMotorDirection == 1) { // Forward
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) { // Backward
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else { // Stop
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
    if (targetLeftSpeedMagnitude == 0) currentActualLeftSpeed = 0; // Ensure speed is zeroed if target is stop
  }

  // Apply to Right Motor (M2) - M2_LPWM is Forward
  if (currentRightMotorDirection == 1) { // Forward
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // Backward
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else { // Stop
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
    if (targetRightSpeedMagnitude == 0) currentActualRightSpeed = 0; // Ensure speed is zeroed if target is stop
  }
}


// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);
  float distance;

  // Calibration for Sharp GP2Y0A41SK0F (4-30cm) or similar
  // This is a rough piecewise linear approximation.
  // Higher voltage means closer.
  if (voltage < 0.42) { // Beyond ~30-35cm or no object
      distance = IR_MAX_DISTANCE_VAL;
  } else if (voltage > 2.8) { // Closer than ~5cm, treat as min reliable
      distance = IR_MIN_DISTANCE -1 ; // Ensure it's less than IR_MIN_DISTANCE to be clamped
  } else if (voltage > 1.5) { // Approx 5-10cm -> maps to voltage 2.8V - 1.5V
      distance = map(voltage * 100, 150, 280, 10, IR_MIN_DISTANCE); // map voltage (1.5-2.8) to distance (10-5)
  } else if (voltage > 0.8) { // Approx 10-20cm -> maps to voltage 1.5V - 0.8V
      distance = map(voltage * 100, 80, 150, 20, 10); // map voltage (0.8-1.5) to distance (20-10)
  } else { // Approx 20-35cm -> maps to voltage 0.8V - 0.42V
      distance = map(voltage * 100, 42, 80, 35, 20); // map voltage (0.42-0.8) to distance (35-20)
  }

  distance = constrain(distance, IR_MIN_DISTANCE, IR_MAX_DISTANCE_VAL);

  // If actualMaxVal is false, cap return at threshold+1 for non-detection
  // If true, return the actual high distance value.
  int returnThreshold = actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;

  if (distance >= returnThreshold) {
      return returnThreshold;
  }
  return (int)distance;
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
        pivotRight(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Right (zero-radius).");
      } else {
        pivotLeft(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Left (zero-radius).");
      }
      preferredDirection = scanDirection;
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW);
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      break;
    case SEARCH_STATE_MOVING_FORWARD:
      moveForward(SPEED_SLOW);
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        stopMovement();
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
  moveForward(SPEED_MAX);
}