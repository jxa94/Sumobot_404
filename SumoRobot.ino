#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right (M2) - Assuming M2_LPWM is FORWARD due to mirrored mounting
#define M2_RPWM 10 // Typically Backward for M2 if M2_LPWM is Forward
#define M2_LPWM 11 // Typically Forward for M2 if mounted mirrored to M1

// Motor left (M1)
#define M1_RPWM 5 // Forward for M1
#define M1_LPWM 6 // Backward for M1

// Starter switch: MicroStart Signal Pin (Digital input)
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

// JSUMO switch variables / MicroStart State
bool robotActive = false; // True when MicroStart signal is HIGH, false when LOW or initially

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
void pivotLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void pivotRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void stopMovement();

int getIRDistance(int sensorPin, bool actualMaxVal = false);
long getUltrasonicDistance(bool actualMaxVal = false);
void updateBumpSensorState();
void searchOpponent();
void attackWithBumpers();
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch(); // Changed to handle MicroStart
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

  pinMode(JSUMO_SWITCH, INPUT); // Changed from INPUT_PULLUP for MicroStart

  stopMovement(); // Initialize motor targets to stop

  Serial.println("Setup Complete. Waiting for MicroStart 'Start' Signal (Signal Pin HIGH).");
}

void loop() {
  handleJsumoSwitch(); // Manages robotActive state based on MicroStart Signal

  if (robotActive) {
    if (isPerformingInitialScan) {
      // Motors should be stopped during the scan itself, or scan strategy defines movement
      // The performInitialScan function below mostly reads sensors, not moves.
      // If scan involves movement, ensure stopMovement() is called appropriately or
      // the scan logic itself sets motor targets.
      // For now, assuming performInitialScan is non-moving, so we can stop motors before it.
      stopMovement(); // Ensure motors are stopped before sensor readings for scan
      performInitialScan(); // This function reads sensors to set initial preferredDirection

      if (millis() - initialScanStartTime >= 3000) {
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE; // Reset special turn state
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_PIVOTING; // Start searching after scan
      }
    } else { // Normal operation after initial scan
      if (currentSpecialTurn != TURN_STATE_NONE) {
        int irLeft = getIRDistance(IR_REFLECT_LEFT);
        int irCenter = getIRDistance(IR_REFLECT_CENTER);
        int irRight = getIRDistance(IR_REFLECT_RIGHT);

        bool targetAcquiredByIR = (irLeft < IR_DETECTION_THRESHOLD ||
                                   irCenter < IR_DETECTION_THRESHOLD ||
                                   irRight < IR_DETECTION_THRESHOLD);

        if (targetAcquiredByIR) {
          Serial.println("Special Pivot: Target ACQUIRED by IR during special turn.");
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
          isOpponentVisible = false; // Lost target during special turn
        } else { // Continue special pivot
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeft(SPEED_FAST);
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRight(SPEED_FAST);
          }
        }
      }
      
      if (currentSpecialTurn == TURN_STATE_NONE) { // Only run main strategy if not in a special turn
        updateBumpSensorState();
        executeStrategy();
      }
    }
  } else { // robotActive is false
    stopMovement();
    isPerformingInitialScan = false; // Ensure scan doesn't run if robot becomes inactive
    currentSpecialTurn = TURN_STATE_NONE; // Ensure special turn is cancelled
  }

  updateMotorsWithRamping(); // Always update motor outputs based on current targets
}


void handleJsumoSwitch() {
    bool jsumoSignalIsHigh = (digitalRead(JSUMO_SWITCH) == HIGH);

    if (jsumoSignalIsHigh && !robotActive) {
        // --- Transition from Inactive to Active (MicroStart "Start" signal received) ---
        Serial.println("MicroStart: START Signal Detected (Signal HIGH). Robot Activating.");
        robotActive = true;

        // Reset and start the initial sequence every time we activate
        isPerformingInitialScan = true;
        initialScanStartTime = millis();

        // Reset other relevant robot states for a fresh start
        currentSpecialTurn = TURN_STATE_NONE;
        currentSearchState = SEARCH_STATE_PIVOTING;
        preferredDirection = 0;
        lastOpponentDetectionTime = 0;
        isOpponentVisible = false;
        bumpSensorState = 0;
        stopMovement(); // Set motor targets to stop, updateMotorsWithRamping will apply it

        Serial.println("MicroStart: Initiating 3s initial scan procedure.");

    } else if (!jsumoSignalIsHigh && robotActive) {
        // --- Transition from Active to Inactive (MicroStart "Stop" signal received) ---
        Serial.println("MicroStart: STOP Signal Detected (Signal LOW). Robot Deactivating.");
        robotActive = false;

        stopMovement(); // Set motor targets to stop
        isPerformingInitialScan = false; 
        currentSpecialTurn = TURN_STATE_NONE;
        Serial.println("MicroStart: Robot deactivated. Motors commanded to stop.");
    }
    // If jsumoSignalIsHigh && robotActive, do nothing (robot is already running correctly)
    // If !jsumoSignalIsHigh && !robotActive, do nothing (robot is already stopped correctly)
}


void performInitialScan() {
  // This function primarily reads sensors to determine an initial preferred direction
  // It does NOT command motor movements itself.
  // The loop() structure ensures motors are stopped before this is called during the initial scan phase.
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  isOpponentVisible = false; // Assume not visible until detected

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  if (backDist < US_DETECTION_THRESHOLD_BACK && (backDist < minFrontIRDistance || !frontIRDetecting)) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis(); // Record time of this initial detection
    if (frontIRDetecting) { // If front also sees something, decide which way to prefer turning
        if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1; // Prefer left
        else if (irRight < irLeft && irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1; // Prefer right
        else preferredDirection = (random(2) == 0) ? -1 : 1; // Random if front is ambiguous
    } else { // Only US sees something
        preferredDirection = (random(2) == 0) ? -1 : 1; // Randomly pick a direction to turn towards back
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
  } else {
    Serial.println("Initial Scan: No opponent detected.");
    preferredDirection = (random(2) == 0) ? -1 : 1; // Default preferred direction for search
  }
}


void executeStrategy() {
  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Cancel special turn if bumpers hit
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

  // Prioritize special turn if US is the clearest target and no special turn is active
  if (usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && currentSpecialTurn == TURN_STATE_NONE) {
    Serial.println("Strategy: US is primary target. Initiating SPECIAL PIVOT.");
    isOpponentVisible = true; // US sees opponent
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    int turnDecision = preferredDirection; // Use preferredDirection from scan or last sighting
    if (turnDecision == 0) { // If preferred was straight or unknown, pick a side
        turnDecision = (random(2) == 0) ? -1 : 1;
    }

    if (turnDecision <= 0) { // Turn left if preferred was left or straight (randomly chosen left)
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_LEFT;
    } else { // Turn right
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_RIGHT;
    }
    // Loop will handle the pivot execution in the next iteration
    return; // Important: Do not proceed further in executeStrategy this cycle
  }

  // If not initiating or in a special turn, proceed with normal strategy
  if (currentSpecialTurn == TURN_STATE_NONE) {
    if (frontIRDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else if (usDetecting) { // US detects, but wasn't "primary" enough for special turn, or special turn not triggered
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      Serial.println("Strategy: US sees opponent (not primary for special pivot). Adjusting.");
      // Fallback to adjustDirectionToOpponent, which might still decide to turn based on US
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else { // No current detection by any sensor
      if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) {
        // Lost opponent briefly, try to reacquire based on last known direction
        Serial.println("Strategy: Lost opponent briefly. Reacquiring.");
        if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
        else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
        else moveForward(SPEED_MEDIUM); // If last seen center, try moving forward
      } else {
        // Opponent truly lost or never seen
        isOpponentVisible = false;
        Serial.println("Strategy: Opponent lost or not found. Searching.");
        searchOpponent();
      }
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK);
  bool frontClearEnoughForUSTurn = (leftDist >= IR_DETECTION_THRESHOLD &&
                                    centerDist >= IR_DETECTION_THRESHOLD &&
                                    rightDist >= IR_DETECTION_THRESHOLD);

  // If opponent is definitely behind (US strong and front is clear OR US very close)
  // and we are not already in a special turn (though this check is also in executeStrategy)
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontClearEnoughForUSTurn) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust (US Dominant): Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Pivoting FAST to face.");
      if (preferredDirection == 1) pivotRight(SPEED_FAST); // If last known was right, pivot right to bring back to front
      else if (preferredDirection == -1) pivotLeft(SPEED_FAST); // If last known was left, pivot left
      else pivotRight(SPEED_FAST); // Default pivot if no strong preference
      // Potentially set currentSpecialTurn here if this is a committed turn-around
      // For now, let it be a single adjustment. If it persists, executeStrategy might trigger special turn.
      return;
  }

  // IR-based adjustment
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust (IR): Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust (IR): Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Pivoting/Moving.");
    setMotorTargets(1, leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW, 1, SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust (IR): Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Pivoting/Moving.");
    setMotorTargets(1, SPEED_FAST, 1, rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW);
    preferredDirection = 1;
  } else if (isOpponentVisible) { // Opponent was visible recently, but IRs are now ambiguous
      Serial.println("Adjust: Opponent was visible but IRs ambiguous. Pivoting based on preferredDirection or searching.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else searchOpponent(); // If no preference, search
  } else {
    // No clear opponent path from IR, and US wasn't dominant enough to take over yet
    Serial.println("Adjust: No clear opponent path (IR/US). Searching.");
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

void pivotLeft(int outerSpeed, int innerSpeed) {
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

void pivotRight(int outerSpeed, int innerSpeed) {
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

  if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
    currentActualLeftSpeed = min(currentActualLeftSpeed + ACCELERATION_STEP, targetLeftSpeedMagnitude);
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed = max(currentActualLeftSpeed - ACCELERATION_STEP, targetLeftSpeedMagnitude);
  }
   if (targetLeftSpeedMagnitude == 0 && currentActualLeftSpeed != 0 && currentActualLeftSpeed < ACCELERATION_STEP) currentActualLeftSpeed = 0;


  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed = min(currentActualRightSpeed + ACCELERATION_STEP, targetRightSpeedMagnitude);
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed = max(currentActualRightSpeed - ACCELERATION_STEP, targetRightSpeedMagnitude);
  }
  if (targetRightSpeedMagnitude == 0 && currentActualRightSpeed != 0 && currentActualRightSpeed < ACCELERATION_STEP) currentActualRightSpeed = 0;


  currentLeftMotorDirection = targetLeftMotorDirection;
  currentRightMotorDirection = targetRightMotorDirection;

  // Apply to M1 (Left Motor)
  if (currentLeftMotorDirection == 1) {
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) {
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else {
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
  }

  // Apply to M2 (Right Motor) - M2_LPWM is FORWARD for M2
  if (currentRightMotorDirection == 1) {
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) {
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else {
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
  }
}


// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  float voltage = reading * (5.0 / 1023.0);
  float distance;

  // Simplified mapping based on typical Sharp IR sensor behavior (adjust if needed)
  if (voltage < 0.4) distance = actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1; // Out of range low
  else if (voltage < 0.42) distance = 80; // Approximation, often means far
  else if (voltage < 0.5) distance = 60;
  else if (voltage < 0.6) distance = 50;
  else if (voltage < 0.8) distance = 35;
  else if (voltage < 1.0) distance = 25;
  else if (voltage < 1.5) distance = 15;
  else if (voltage < 2.2) distance = 10;
  else if (voltage < 2.8) distance = 5;   // Very close
  else distance = IR_MIN_DISTANCE;        // Likely too close, effectively min distance

  if (distance > (actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD +1) ) {
      return actualMaxVal ? IR_MAX_DISTANCE_VAL : IR_DETECTION_THRESHOLD + 1;
  }
  if (distance < IR_MIN_DISTANCE) return IR_MIN_DISTANCE;
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); // 25ms timeout
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

  // Change search pattern occasionally
  if (currentTime - lastScanTime > 3000) { // Increased duration for pivoting search
    scanDirection = -scanDirection; // Reverse pivot direction
    lastScanTime = currentTime;
    // Occasionally add a forward movement to the search
    if (random(4) == 0) { // 1 in 4 chance
      currentSearchState = SEARCH_STATE_INIT_FORWARD;
      searchStateTimer = currentTime;
      Serial.println("Search: Adding forward movement.");
      return; // Exit to execute forward movement
    }
  }

  switch (currentSearchState) {
    case SEARCH_STATE_PIVOTING:
      if (scanDirection == 1) pivotRight(SPEED_MEDIUM);
      else pivotLeft(SPEED_MEDIUM);
      preferredDirection = scanDirection; // Update preferred direction based on search pivot
      break;
    case SEARCH_STATE_INIT_FORWARD:
      moveForward(SPEED_SLOW);
      currentSearchState = SEARCH_STATE_MOVING_FORWARD;
      // searchStateTimer already set when transitioning to INIT_FORWARD
      break;
    case SEARCH_STATE_MOVING_FORWARD:
      moveForward(SPEED_SLOW); // Continue moving forward
      if (millis() - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        stopMovement();
        searchStateTimer = millis();
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
        Serial.println("Search: Stopping after forward movement.");
      }
      break;
    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      stopMovement(); // Ensure stopped
      if (millis() - searchStateTimer >= SEARCH_STOP_DURATION) {
        currentSearchState = SEARCH_STATE_PIVOTING; // Return to pivoting
        lastScanTime = millis(); // Reset scan timer for pivoting
        Serial.println("Search: Returning to pivoting.");
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX);
  // No preferredDirection change here, just pure attack.
}