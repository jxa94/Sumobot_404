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
const int SPEED_TURN_PIVOT_INNER = 10; // Speed for the inner wheel during an ARC turn
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150; // Used for pivot outer wheel speed & zero-radius pivot speed
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
int currentLeftMotorDirection = 0;
int currentRightMotorDirection = 0;
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

// MODIFIED/NEW Pivot function declarations
void arcLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER); // Renamed from pivotLeft, performs an arc turn
void arcRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER); // Renamed from pivotRight, performs an arc turn
void pivotLeft(int speed); // New: for zero-radius (point) turn
void pivotRight(int speed); // New: for zero-radius (point) turn

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

  pinMode(JSUMO_SWITCH, INPUT_PULLUP); // Or INPUT if module has strong pull-downs for LOW

  stopMovement();

  // Initialize JSUMO switch state
  jsumoSignalWasHighLastFrame = (digitalRead(JSUMO_SWITCH) == HIGH);
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
}

void loop() {
  handleJsumoSwitch(); // Manages robotActive state based on MicroStart signal

  if (robotActive) {
    if (isPerformingInitialScan) {
      stopMovement(); // Ensure motors are stopped during the initial scan phase
      performInitialScan();

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE; // Ensure special turn is reset after scan
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_PIVOTING; // Or your desired post-scan state
      }
    } else { // Main operational logic
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
          // These calls now use the new zero-radius pivot functions
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeft(SPEED_MEDIUM); // MODIFIED: Was SPEED_FAST
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRight(SPEED_MEDIUM); // MODIFIED: Was SPEED_FAST
          }
        }
      }
      
      if (currentSpecialTurn == TURN_STATE_NONE) { 
            updateBumpSensorState();
            executeStrategy();
      }
    }
  } else { // robotActive is false
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE; // Reset special turn state
    isPerformingInitialScan = false; // Ensure scan flag is also reset
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
      stopMovement();
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
    if (turnDecision == 0) { // If no strong preference, or centered, pick a random direction for pivot
        turnDecision = (random(2) == 0) ? -1 : 1;
    }

    if (turnDecision <= 0) { // Includes -1 and cases where preferredDirection was 0 and randomly became -1
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_LEFT;
    } else { // Includes 1 and cases where preferredDirection was 0 and randomly became 1
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_RIGHT;
    }
    // The actual pivot command is handled in loop() based on currentSpecialTurn
    return; // Return to allow the special turn to execute in the next loop iteration
  }


  if (currentSpecialTurn == TURN_STATE_NONE) { // Only proceed if not in a special US-triggered pivot
    if (frontIRDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else if (usDetecting) { // US detects, but not as primary for special pivot (e.g., front IRs also see something closer)
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      Serial.println("Strategy: US sees opponent (not primary for special pivot). Adjusting based on overall sensor view.");
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // Still let adjustDirection decide
    } else {
      // Opponent not visible by IR or US
      if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) { // Lost opponent briefly
        Serial.println("Strategy: Lost opponent briefly. Reacquiring based on last preferred direction.");
        // These calls now use the new zero-radius pivot functions
        if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
        else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
        else moveForward(SPEED_MEDIUM); // If last seen centered, cautiously move forward
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

  // Priority: If opponent is very close behind OR behind and front is clear, pivot to face them.
  // This check is only done if not already in a special US turn.
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontSensorsClear) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust: Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Pivoting to engage."); // MODIFIED: "FAST" removed
      // These calls now use the new zero-radius pivot functions
      if (preferredDirection == 1) pivotRight(SPEED_SLOW); // MODIFIED: Was SPEED_FAST
      else if (preferredDirection == -1) pivotLeft(SPEED_SLOW); // MODIFIED: Was SPEED_FAST
      else {
          // If preferredDirection is 0, default to one direction (e.g., right) or choose randomly
          pivotRight(SPEED_SLOW); // MODIFIED: Was SPEED_FAST
      }
      return; // Pivoting, so no further adjustment needed this cycle
  }

  // If opponent is in front
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Pivoting/Moving towards left.");
    // MODIFIED: Use arcLeft for an arc turn
    arcLeft(SPEED_FAST, (leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Pivoting/Moving towards right.");
    // MODIFIED: Use arcRight for an arc turn
    arcRight(SPEED_FAST, (rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = 1;
  } else if (isOpponentVisible) { // IRs are ambiguous but opponent was recently seen
      Serial.println("Adjust: Opponent was visible but current IRs ambiguous. Pivoting based on last known direction or searching.");
      // These calls now use the new zero-radius pivot functions
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else searchOpponent(); // If last known was center, and now ambiguous, search
  } else {
    // No clear opponent path from front IRs, and US didn't trigger the above pivot
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

// RENAMED from pivotLeft: Performs an arc turn to the left
// M1 (left) is inner, M2 (right) is outer. Both move forward.
// M1 speed = innerSpeed, M2 speed = outerSpeed.
void arcLeft(int outerSpeed, int innerSpeed) { // Default for innerSpeed is SPEED_TURN_PIVOT_INNER
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

// RENAMED from pivotRight: Performs an arc turn to the right
// M2 (right) is inner, M1 (left) is outer. Both move forward.
// M1 speed = outerSpeed, M2 speed = innerSpeed.
void arcRight(int outerSpeed, int innerSpeed) { // Default for innerSpeed is SPEED_TURN_PIVOT_INNER
  setMotorTargets(1, outerSpeed, 1, innerSpeed);
}

// NEW: Performs a zero-radius (point) turn to the left
// M1 (left) moves backward, M2 (right) moves forward at the same speed.
void pivotLeft(int speed) {
  setMotorTargets(-1, speed, 1, speed);
}

// NEW: Performs a zero-radius (point) turn to the right
// M1 (left) moves forward, M2 (right) moves backward at the same speed.
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
    // If changing direction while needing to move, decelerate to 0 first
    if (currentActualLeftSpeed > 0) {
        currentActualLeftSpeed -= ACCELERATION_STEP * 2; // Faster deceleration
        if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
    } else {
        currentLeftMotorDirection = targetLeftMotorDirection; // Switch direction once stopped
    }
  } else if (currentLeftMotorDirection != targetLeftMotorDirection && targetLeftSpeedMagnitude == 0){
    currentLeftMotorDirection = targetLeftMotorDirection; // Ok to switch direction if target speed is 0
  }


  if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
    currentActualLeftSpeed += ACCELERATION_STEP;
    if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed -= ACCELERATION_STEP;
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  }
  if (targetLeftSpeedMagnitude == 0 && currentActualLeftSpeed < ACCELERATION_STEP / 2) {
      currentActualLeftSpeed = 0;
  }
  currentActualLeftSpeed = constrain(currentActualLeftSpeed, 0, 255);


  // Ramp Right Motor
  if (currentRightMotorDirection != targetRightMotorDirection && targetRightSpeedMagnitude > 0) {
    // If changing direction while needing to move, decelerate to 0 first
    if (currentActualRightSpeed > 0) {
        currentActualRightSpeed -= ACCELERATION_STEP * 2; // Faster deceleration
        if (currentActualRightSpeed < 0) currentActualRightSpeed = 0;
    } else {
        currentRightMotorDirection = targetRightMotorDirection; // Switch direction once stopped
    }
  } else if (currentRightMotorDirection != targetRightMotorDirection && targetRightSpeedMagnitude == 0){
      currentRightMotorDirection = targetRightMotorDirection; // Ok to switch direction if target speed is 0
  }


  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed += ACCELERATION_STEP;
    if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed -= ACCELERATION_STEP;
    if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  }
   if (targetRightSpeedMagnitude == 0 && currentActualRightSpeed < ACCELERATION_STEP / 2) {
      currentActualRightSpeed = 0;
  }
  currentActualRightSpeed = constrain(currentActualRightSpeed, 0, 255);


  // Apply to Left Motor
  if (currentLeftMotorDirection == 1) { // Forward
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) { // Backward
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else { // Stop
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
    currentActualLeftSpeed = 0; // Ensure speed is zeroed if direction is 0
  }

  // Apply to Right Motor
  // M2_LPWM is Forward for M2, M2_RPWM is Backward
  if (currentRightMotorDirection == 1) { // Forward
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // Backward
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else { // Stop
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
    currentActualRightSpeed = 0; // Ensure speed is zeroed if direction is 0
  }
}


// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  // Simple mapping, assuming higher reading means closer
  // This needs calibration for your specific sensors (e.g., GP2Y0A41SK0F 4-30cm, GP2Y0A21YK0F 10-80cm)
  // For GP2Y0A41SK0F (4-30cm): Voltage increases as distance decreases.
  // ~3.0V @ 4cm, ~0.4V @ 30cm
  // Let's assume a somewhat linear inverse relationship for simplification, or use a lookup table/curve fit.
  // This is a very rough example, replace with proper calibration.
  // map(value, fromLow, fromHigh, toLow, toHigh)
  // If analogRead is 0-1023. High value = low voltage = far. Low value = high voltage = close. This is for IR Emitters/Detectors.
  // For distance sensors like Sharp, higher analogRead values often mean *lower* voltage, and for some sensors, higher voltage means closer.
  // The provided example seems to use a voltage-to-distance mapping.
  float voltage = reading * (5.0 / 1023.0);

  float distance;
  // This calibration is specific to Sharp GP2Y0A41SK0F (4-30cm) or similar
  // It's a rough piecewise linear approximation.
  if (voltage < 0.42) { // Beyond ~30-35cm or no object
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  } else if (voltage > 2.8) { // Closer than ~5cm
      distance = 5; // Min reliable distance
  } else if (voltage > 1.5) { // Approx 5-10cm
      // (V - 1.5) / (2.8 - 1.5) = (D - 10) / (5 - 10)
      // (V - 1.5) / 1.3 = (D - 10) / -5
      // D = 10 - 5 * (V - 1.5) / 1.3
      distance = 10 - 5 * (voltage - 1.5) / (2.8 - 1.5);
  } else if (voltage > 0.8) { // Approx 10-20cm
      // (V - 0.8) / (1.5 - 0.8) = (D - 20) / (10 - 20)
      // (V - 0.8) / 0.7 = (D - 20) / -10
      // D = 20 - 10 * (V - 0.8) / 0.7
      distance = 20 - 10 * (voltage - 0.8) / (1.5 - 0.8);
  } else if (voltage > 0.42) { // Approx 20-35cm
      // (V - 0.42) / (0.8 - 0.42) = (D - 35) / (20 - 35)
      // (V - 0.42) / 0.38 = (D - 35) / -15
      // D = 35 - 15 * (V - 0.42) / 0.38
      distance = 35 - 15 * (voltage - 0.42) / (0.8 - 0.42);
  } else { // Should be covered by first if
      distance = IR_MAX_DISTANCE_VAL;
  }

  distance = constrain(distance, IR_MIN_DISTANCE, IR_MAX_DISTANCE_VAL);

  if (distance > (actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1) ) {
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  }
  // if (distance < IR_MIN_DISTANCE && distance > 0) return IR_MIN_DISTANCE; // already handled by constrain
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); // Timeout 25ms (approx 4.25m, well beyond sumo range)
  long distance = duration * 0.034 / 2; // Speed of sound = 340 m/s or 0.034 cm/us

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
  // Change pivot direction or try moving forward periodically
  if (currentSearchState == SEARCH_STATE_PIVOTING && currentTime - lastScanTime > 2000) { // Every 2 seconds
    scanDirection = -scanDirection; // Reverse pivot direction
    lastScanTime = currentTime;
    if (random(3) == 0) { // 1 in 3 chance to try moving forward
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
      searchStateTimer = currentTime;
      Serial.println("Search: Switching to short forward burst.");
      return; // Exit to allow state change to take effect
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_PIVOTING:
      // These calls now use the new zero-radius pivot functions
      if (scanDirection == 1) {
        pivotRight(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Right (zero-radius).");
      } else {
        pivotLeft(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Left (zero-radius).");
      }
      preferredDirection = scanDirection; // Update preferred direction based on current search pivot
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW);
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      // searchStateTimer was already set when transitioning to INIT_FORWARD
      break;
    case SEARCH_STATE_MOVING_FORWARD:
      moveForward(SPEED_SLOW); // Continue moving forward
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        stopMovement();
        searchStateTimer = millis(); // Reset timer for stop duration
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
        Serial.println("Search: Finished forward burst, stopping briefly.");
      }
      break;
    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      stopMovement(); // Ensure stopped
      if (millis() - searchStateTimer >= SEARCH_STOP_DURATION) {
        currentSearchState = SEARCH_STATE_PIVOTING; // Go back to pivoting
        lastScanTime = millis(); // Reset scan timer to avoid immediate pivot direction change
        Serial.println("Search: Resuming pivot.");
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX);
}
