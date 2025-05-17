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
const int SPEED_TURN_PIVOT_INNER = 10;
const int SPEED_SLOW = 50;
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150;
const int SPEED_MAX = 200;
const int SPEED_SPIN = 130; // Speed for one-wheel spins, can be adjusted

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

bool robotActive = false;
bool jsumoSignalWasHighLastFrame = false;

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

enum SpecialTurnState {
  TURN_STATE_NONE,
  TURN_STATE_US_TRIGGERED_PIVOT_LEFT,
  TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
};
SpecialTurnState currentSpecialTurn = TURN_STATE_NONE;
unsigned long specialTurnStartTime = 0;
const unsigned long MAX_SPECIAL_TURN_DURATION = 3000; // Reduced special turn duration a bit

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
void spinLeftOneWheel(int speed);    // <<< NEW
void spinRightOneWheel(int speed);   // <<< NEW
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

  jsumoSignalWasHighLastFrame = (digitalRead(JSUMO_SWITCH) == HIGH);
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
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
          Serial.println("Special Pivot: Target ACQUIRED by IR during spin.");
          stopMovement(); // Stop the spin
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = true;
          lastOpponentDetectionTime = millis();
          // Let adjustDirectionToOpponent handle the IR target on the next cycle
          // Update preferred direction based on IR
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
          executeStrategy(); // Immediately try to act on the new IR data
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Pivot: MAX DURATION reached for spin.");
          stopMovement(); // Stop the spin
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; // Assume lost if timed out
        } else {
          // Continue the special one-wheel spin
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            spinLeftOneWheel(SPEED_SPIN); // <<< EDITED to use new spin function and speed
            // Serial.println("Special Pivot: Spinning Left (one wheel)"); // Redundant if printed once at start
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            spinRightOneWheel(SPEED_SPIN); // <<< EDITED to use new spin function and speed
            // Serial.println("Special Pivot: Spinning Right (one wheel)"); // Redundant
          }
        }
      }
      
      if (currentSpecialTurn == TURN_STATE_NONE) { // Only run main strategy if not in a special spin
            updateBumpSensorState();
            executeStrategy();
      }
    }
  } else {
    stopMovement();
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
    Serial.println("Initial Scan: US is primary target. Preferred spin direction set.");
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
    currentSpecialTurn = TURN_STATE_NONE; // Ensure any special turn is cancelled
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
    Serial.println("Strategy: US is primary target. Initiating SPECIAL ONE-WHEEL SPIN.");
    isOpponentVisible = true; // Mark as visible due to US
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    int turnDecision = preferredDirection;
    if (turnDecision == 0) {
        turnDecision = (random(2) == 0) ? -1 : 1;
    }

    if (turnDecision <= 0) {
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_LEFT; // Will spin left
         Serial.println("Special Spin Triggered: Left");
    } else {
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_RIGHT; // Will spin right
         Serial.println("Special Spin Triggered: Right");
    }
    // The loop() will now handle the spin because currentSpecialTurn is set
    // No motor commands here, loop() handles the spin action itself.
    return; // Return to let the loop handle the special turn immediately
  }

  // This part only runs if not initiating or in a special turn for US
  if (currentSpecialTurn == TURN_STATE_NONE) {
    if (frontIRDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    } else if (usDetecting) {
      isOpponentVisible = true;
      lastOpponentDetectionTime = millis();
      Serial.println("Strategy: US sees opponent (not primary for special spin). Adjusting.");
      adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // Let adjustDirection decide (might spin if back is close)
    } else {
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
  bool frontSensorsClear = (leftDist >= IR_DETECTION_THRESHOLD &&
                            centerDist >= IR_DETECTION_THRESHOLD &&
                            rightDist >= IR_DETECTION_THRESHOLD);

  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontSensorsClear) && currentSpecialTurn == TURN_STATE_NONE) {
      Serial.print("Adjust: Opponent BEHIND (US: "); Serial.print(backDist); Serial.println("cm). Spinning FAST to engage (one wheel).");
      // Prefer spinning towards the side that was last "preferred" if known, otherwise default.
      // If preferredDirection was for a front target, it might not be ideal, so consider a default spin.
      // For simplicity, let's use preferredDirection if it's non-zero, or a default.
      if (preferredDirection == 1) {      // If preferred was right, spin right to bring front to where back was
          spinRightOneWheel(SPEED_SPIN);   // <<< EDITED
      } else if (preferredDirection == -1) { // If preferred was left, spin left
          spinLeftOneWheel(SPEED_SPIN);    // <<< EDITED
      } else {                             // No strong preference or preferred was center, default spin
          spinRightOneWheel(SPEED_SPIN);   // <<< EDITED (defaulting to spin right)
      }
      // isOpponentVisible = true; // Already true if US detected
      lastOpponentDetectionTime = millis();
      return; // Committed to a spin for this cycle
  }

  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking.");
    moveForward(centerDist < IR_CLOSE_THRESHOLD ? SPEED_FAST : SPEED_MEDIUM);
    preferredDirection = 0;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Pivoting/Moving towards left.");
    setMotorTargets(1, leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW, 1, SPEED_FAST);
    preferredDirection = -1;
  } else if (rightDist < IR_DETECTION_THRESHOLD) {
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Pivoting/Moving towards right.");
    setMotorTargets(1, SPEED_FAST, 1, rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW);
    preferredDirection = 1;
  } else if (isOpponentVisible) {
      Serial.println("Adjust: Opponent was visible but IRs ambiguous. Pivoting based on last known or searching.");
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

// --- Movement Functions ---
void moveForward(int speed) {
  setMotorTargets(1, speed, 1, speed);
}

void moveBackward(int speed) {
  setMotorTargets(-1, speed, -1, speed);
}

void pivotLeft(int outerSpeed, int innerSpeed) { // Standard pivot: both wheels forward, right (outer) faster
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

void pivotRight(int outerSpeed, int innerSpeed) { // Standard pivot: both wheels forward, left (outer) faster
  setMotorTargets(1, outerSpeed, 1, innerSpeed);
}

// <<< NEW FUNCTIONS for one-wheel spin >>>
void spinLeftOneWheel(int speed) { // Turn body left: Right wheel forward, Left wheel stop
  setMotorTargets(0, 0, 1, speed); // M1 (Left) Stop, M2 (Right) Forward
}

void spinRightOneWheel(int speed) { // Turn body right: Left wheel forward, Right wheel stop
  setMotorTargets(1, speed, 0, 0);  // M1 (Left) Forward, M2 (Right) Stop
}
// <<< END NEW FUNCTIONS >>>

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
    currentActualLeftSpeed += ACCELERATION_STEP;
    if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
    currentActualLeftSpeed -= ACCELERATION_STEP;
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
  }
  if (targetLeftSpeedMagnitude == 0 && currentActualLeftSpeed < ACCELERATION_STEP / 2) currentActualLeftSpeed = 0;


  if (currentActualRightSpeed < targetRightSpeedMagnitude) {
    currentActualRightSpeed += ACCELERATION_STEP;
    if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
    currentActualRightSpeed -= ACCELERATION_STEP;
    if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
  }
  if (targetRightSpeedMagnitude == 0 && currentActualRightSpeed < ACCELERATION_STEP / 2) currentActualRightSpeed = 0;


  currentLeftMotorDirection = targetLeftMotorDirection;
  currentRightMotorDirection = targetRightMotorDirection;

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
  if (distance < IR_MIN_DISTANCE && distance > 0) return IR_MIN_DISTANCE;
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  // These short delays are necessary for the HC-SR04 sensor to trigger correctly.
  // They are in microseconds and do not significantly block the loop.
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2); // Ensure TRIG pin is low for a short period
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10); // Send a 10 microsecond pulse to TRIG
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // pulseIn() will wait for the echo or timeout.
  // Timeout is 25ms. If no echo, code pauses here for up to 25ms.
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
    case SEARCH_STATE_PIVOTING: // Uses standard two-wheel pivot
      if (scanDirection == 1) {
        pivotRight(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Right (standard).");
      } else {
        pivotLeft(SPEED_MEDIUM);
        Serial.println("Search: Pivoting Left (standard).");
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
        Serial.println("Search: Resuming pivot (standard).");
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX);
}