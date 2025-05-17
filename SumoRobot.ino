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
const int SPEED_TURN_PIVOT_INNER = 10; // Speed for the inner wheel during an ARC turn (very close)
const int SPEED_SLOW = 80;             // General slow speed, also used for inner wheel of arcs (not very close)
const int SPEED_MEDIUM = 100;          // General medium speed (e.g., for forward movement)
const int SPEED_FAST = 150;            // General fast speed (e.g., for forward attack)
const int SPEED_MAX = 200;             // Maximum speed for bumper attacks
const int SPEED_RETREAT = SPEED_MEDIUM; 

// NEW Turning Speeds
const int SPEED_PIVOT_DEFAULT = 60;    // Slower speed for general pivots (Ensure motors don't stall at this speed)
const int SPEED_ARC_OUTER_DEFAULT = 100; // Slower speed for the outer wheel in arc turns

const float RIGHT_MOTOR_FACTOR = 1.0;

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999; 
const int IR_MIN_DISTANCE = 10;  // Adjust based on your IR sensor's true minimum reliable distance
const int IR_DETECTION_THRESHOLD = 70; 
const int IR_CLOSE_THRESHOLD = 30; 

const int US_MAX_DISTANCE_VAL = 999; 
const int US_DETECTION_THRESHOLD_BACK = 40; 
const int US_VERY_CLOSE_BACK = 15; // Critical close distance for US

// Global variables for robot state
unsigned long initialScanStartTime = 0;
bool isPerformingInitialScan = false;

int bumpSensorState = 0;
int previousBumpSensorState = 0;
bool isRetreatingAfterBump = false;
unsigned long retreatStartTime = 0;
const unsigned long BUMPER_RETREAT_DURATION = 500; 

int preferredDirection = 0; 
unsigned long lastOpponentDetectionTime = 0;
bool isOpponentVisible = false;

// JSUMO MicroStart switch variables
bool robotActive = false;
bool jsumoSignalWasHighLastFrame = false;

// Search state variables
enum SearchBotState {
  SEARCH_STATE_INIT,
  SEARCH_STATE_PIVOTING_LEFT,
  SEARCH_STATE_PIVOTING_RIGHT,
  SEARCH_STATE_MOVING_FORWARD,
  SEARCH_STATE_STOPPING_AFTER_FORWARD
};
SearchBotState currentSearchState = SEARCH_STATE_INIT;
unsigned long searchStateTimer = 0;
const unsigned long SEARCH_PIVOT_DURATION = 1500; 
const unsigned long SEARCH_FORWARD_DURATION = 1000;
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

// Cooldown for US-triggered special turns
unsigned long usSpecialTurnCooldownUntil = 0;
const unsigned long US_SPECIAL_TURN_COOLDOWN_DURATION = 3000; 

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
void arcLeft(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void arcRight(int outerSpeed, int innerSpeed = SPEED_TURN_PIVOT_INNER);
void pivotLeft(int speed);
void pivotRight(int speed);
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
  isRetreatingAfterBump = false;
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
        currentSearchState = SEARCH_STATE_INIT; 
        searchStateTimer = millis();
        usSpecialTurnCooldownUntil = 0; 
      }
    } else { 
      // Handle ongoing special US-triggered pivot
      if (currentSpecialTurn != TURN_STATE_NONE) {
        // Get current IR readings. US reading from pivot initiation is implicitly used.
        int irLeftDuringPivot = getIRDistance(IR_REFLECT_LEFT, true); 
        int irCenterDuringPivot = getIRDistance(IR_REFLECT_CENTER, true);
        int irRightDuringPivot = getIRDistance(IR_REFLECT_RIGHT, true);

        bool targetAcquiredByIR = (irLeftDuringPivot < IR_DETECTION_THRESHOLD ||
                                   irCenterDuringPivot < IR_DETECTION_THRESHOLD ||
                                   irRightDuringPivot < IR_DETECTION_THRESHOLD);

        // ***** MODIFICATION 1: Handle IR acquisition during special US pivot *****
        if (targetAcquiredByIR) {
          Serial.println("Special Pivot: Target ACQUIRED by IR. Transitioning. Applying cooldown.");
          // stopMovement(); // OLD: This was a potential "stop" point.
          isOpponentVisible = true;
          lastOpponentDetectionTime = millis();
          
          // Update preferred direction based on which IR sensor acquired the target
          if (irCenterDuringPivot < irLeftDuringPivot && irCenterDuringPivot < irRightDuringPivot && irCenterDuringPivot < IR_DETECTION_THRESHOLD) {
            preferredDirection = 0;
          } else if (irLeftDuringPivot < irRightDuringPivot && irLeftDuringPivot < IR_DETECTION_THRESHOLD) {
            preferredDirection = -1;
          } else if (irRightDuringPivot < IR_DETECTION_THRESHOLD) {
            preferredDirection = 1;
          }
          
          currentSpecialTurn = TURN_STATE_NONE; // End the special turn
          usSpecialTurnCooldownUntil = millis() + US_SPECIAL_TURN_COOLDOWN_DURATION; // Apply cooldown

          // Get fresh sensor readings (including US) for adjustDirectionToOpponent
          int currentIrLeft = getIRDistance(IR_REFLECT_LEFT, true);
          int currentIrCenter = getIRDistance(IR_REFLECT_CENTER, true);
          int currentIrRight = getIRDistance(IR_REFLECT_RIGHT, true);
          long currentBackDist = getUltrasonicDistance(true); 

          adjustDirectionToOpponent(currentIrLeft, currentIrCenter, currentIrRight, currentBackDist);
          // The main loop structure will then proceed. updateMotorsWithRamping() will be called at the end.
          // The `if (currentSpecialTurn == TURN_STATE_NONE)` block below will execute next in this same loop iteration.
        } 
        // ***** END OF MODIFICATION 1 *****
        else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) { // Pivot timeout
          Serial.println("Special Pivot: MAX DURATION reached without IR lock.");
          stopMovement(); 
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; 
          usSpecialTurnCooldownUntil = millis() + US_SPECIAL_TURN_COOLDOWN_DURATION; 
        } else { // Continue pivoting
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeft(SPEED_PIVOT_DEFAULT); 
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRight(SPEED_PIVOT_DEFAULT); 
          }
        }
      }
      
      // If not in a special US-triggered pivot (or just exited one), run main strategy
      if (currentSpecialTurn == TURN_STATE_NONE) { 
            updateBumpSensorState(); 
            executeStrategy(); // This will call adjustDirectionToOpponent if needed
      }
    }
  } else { 
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE;
    isPerformingInitialScan = false;
    isRetreatingAfterBump = false;
    usSpecialTurnCooldownUntil = 0;
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
      previousBumpSensorState = 0;
      isRetreatingAfterBump = false;
      currentSpecialTurn = TURN_STATE_NONE;
      currentSearchState = SEARCH_STATE_INIT;
      searchStateTimer = millis();
      usSpecialTurnCooldownUntil = 0;
      stopMovement(); 
    }
  } else if (!signalIsCurrentlyHigh && jsumoSignalWasHighLastFrame) {
    if (robotActive) {
      Serial.println("Robot Deactivated by MicroStart (Signal LOW).");
      robotActive = false;
      isPerformingInitialScan = false;
      isRetreatingAfterBump = false;
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
    preferredDirection = (random(2) == 0) ? -1 : 1; 
    Serial.println("Initial Scan: US is primary target. Preferred pivot (random) set.");
  } else if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    if (irCenter == minFrontIRDistance && irCenter < IR_DETECTION_THRESHOLD) {
        preferredDirection = 0; Serial.println("Initial Scan: Opponent CENTER by IR.");
    } else if (irLeft == minFrontIRDistance && irLeft < IR_DETECTION_THRESHOLD) {
        preferredDirection = -1; Serial.println("Initial Scan: Opponent LEFT by IR.");
    } else if (irRight == minFrontIRDistance && irRight < IR_DETECTION_THRESHOLD) {
        preferredDirection = 1; Serial.println("Initial Scan: Opponent RIGHT by IR.");
    } else { 
        preferredDirection = 0; 
        Serial.println("Initial Scan: Front IR ambiguous. Preferred direction neutral.");
    }
  } else {
      preferredDirection = 0; 
      Serial.println("Initial Scan: No opponent detected.");
  }
}


void executeStrategy() {
  // Bumper logic has highest priority
  if (isRetreatingAfterBump) {
    if (bumpSensorState > 0) { 
      Serial.println("Retreat: Interrupted by new bumper press. Attacking.");
      isRetreatingAfterBump = false;
      attackWithBumpers(); 
      lastOpponentDetectionTime = millis();
      isOpponentVisible = true; 
    } else if (millis() - retreatStartTime >= BUMPER_RETREAT_DURATION) {
      Serial.println("Retreat: Duration complete. Resuming normal ops.");
      isRetreatingAfterBump = false;
      stopMovement(); 
    } else {
      moveBackward(SPEED_RETREAT); 
    }
    return; 
  }

  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Cancel any special turn
    usSpecialTurnCooldownUntil = 0; // Reset cooldown
    return; 
  }

  if (previousBumpSensorState > 0 && bumpSensorState == 0) {
    Serial.println("Bumper released. Initiating retreat.");
    isRetreatingAfterBump = true;
    retreatStartTime = millis();
    moveBackward(SPEED_RETREAT); 
    return; 
  }

  // If no bumper activity, proceed with sensor-based strategy
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

  // Condition for initiating a special US-triggered pivot:
  if (millis() >= usSpecialTurnCooldownUntil &&
      usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && 
      currentSpecialTurn == TURN_STATE_NONE) { // Ensure not already in a special turn
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
    // The actual pivot movement will be handled in the loop() by the currentSpecialTurn block.
    return; // Exit executeStrategy for this iteration; special pivot will take over.
  }

  // If not initiating/in a special turn, proceed with normal sensor evaluation
  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    // If IR takes over, we might want to reset US cooldown, but it's currently only set by US events.
    // usSpecialTurnCooldownUntil = 0; // Consider if IR detection should always break US cooldown.
  } else if (usDetecting) { 
    // US is detecting, but not as primary for a special turn (e.g., cooldown active, or front IR also saw something)
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    Serial.println("Strategy: US sees opponent (not primary for special pivot OR cooldown active). Adjusting via US logic.");
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); 
  } else {
    // No front IR, no US detection
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) { 
      Serial.println("Strategy: Lost opponent briefly. Reacquiring based on last preferred direction.");
      if (preferredDirection < 0) pivotLeft(SPEED_PIVOT_DEFAULT); 
      else if (preferredDirection > 0) pivotRight(SPEED_PIVOT_DEFAULT); 
      else moveForward(SPEED_MEDIUM); 
    } else { 
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching.");
      searchOpponent();
    }
  }
}

void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK); 
  bool frontSensorsClear = (leftDist >= IR_DETECTION_THRESHOLD &&
                            centerDist >= IR_DETECTION_THRESHOLD &&
                            rightDist >= IR_DETECTION_THRESHOLD);

  // Priority 1: Strong frontal IR detection for attack
  if (centerDist < IR_CLOSE_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER & VERY CLOSE (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking FAST.");
    moveForward(SPEED_FAST);
    preferredDirection = 0;
    return;
  }
  if (centerDist < IR_DETECTION_THRESHOLD && centerDist <= leftDist && centerDist <= rightDist) {
    Serial.print("Adjust: Opponent CENTER (IR: "); Serial.print(centerDist); Serial.println("cm). Attacking MEDIUM.");
    moveForward(SPEED_MEDIUM);
    preferredDirection = 0;
    return;
  }

  // Priority 2: Side IR detection for arcing
  if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    Serial.print("Adjust: Opponent LEFT (IR: "); Serial.print(leftDist); Serial.println("cm). Arc turning left.");
    arcLeft(SPEED_ARC_OUTER_DEFAULT, (leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW)); 
    preferredDirection = -1;
    return;
  }
  if (rightDist < IR_DETECTION_THRESHOLD) { 
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Arc turning right.");
    arcRight(SPEED_ARC_OUTER_DEFAULT, (rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW)); 
    preferredDirection = 1;
    return;
  }

  // Priority 3: US detection if front IRs are not decisive OR US indicates critical close threat
  // This block is reached if the IR conditions above were NOT met.
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontSensorsClear)) {
      Serial.print("Adjust: Opponent BEHIND or US primary (IRs not decisive). US: "); Serial.print(backDist); Serial.println("cm. Pivoting by US.");
      int pivotDir = preferredDirection; 
      if (pivotDir == 0) pivotDir = (random(2) == 0) ? -1 : 1; 

      if (pivotDir <= 0) pivotLeft(SPEED_PIVOT_DEFAULT); 
      else pivotRight(SPEED_PIVOT_DEFAULT);  
      preferredDirection = (pivotDir <=0) ? -1 : 1; 
      return;
  }

  // Priority 4: Fallback if sensor data is ambiguous but opponent was recently visible
  if (isOpponentVisible) { 
      Serial.println("Adjust: Opponent was visible but current sensors ambiguous. Pivoting based on last known direction.");
      if (preferredDirection < 0) pivotLeft(SPEED_PIVOT_DEFAULT); 
      else if (preferredDirection > 0) pivotRight(SPEED_PIVOT_DEFAULT); 
      else searchOpponent(); // If no preference, or was center, search
      return;
  }
  
  // Priority 5: No clear opponent, resort to search
  Serial.println("Adjust: No clear opponent path, resorting to search.");
  searchOpponent();
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

  // --- Left Motor Ramping ---
  if (currentLeftMotorDirection != 0 && targetLeftMotorDirection != currentLeftMotorDirection && currentActualLeftSpeed > 0) {
    currentActualLeftSpeed -= ACCELERATION_STEP * 2; 
    if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
  }
  else if (targetLeftMotorDirection == 0 || (targetLeftMotorDirection != currentLeftMotorDirection && targetLeftSpeedMagnitude == 0)) {
    if (currentActualLeftSpeed > 0) {
        currentActualLeftSpeed -= ACCELERATION_STEP;
        if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
    }
  }
  else {
    if (currentActualLeftSpeed < targetLeftSpeedMagnitude) {
        currentActualLeftSpeed += ACCELERATION_STEP;
        if (currentActualLeftSpeed > targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
    } else if (currentActualLeftSpeed > targetLeftSpeedMagnitude) {
        currentActualLeftSpeed -= ACCELERATION_STEP;
        if (currentActualLeftSpeed < targetLeftSpeedMagnitude) currentActualLeftSpeed = targetLeftSpeedMagnitude;
    }
  }
  currentActualLeftSpeed = constrain(currentActualLeftSpeed, 0, 255);

  if (currentActualLeftSpeed == 0) {
    currentLeftMotorDirection = targetLeftMotorDirection;
  } else if (currentLeftMotorDirection != targetLeftMotorDirection && targetLeftMotorDirection != 0) {
    if (targetLeftSpeedMagnitude > 0) currentLeftMotorDirection = targetLeftMotorDirection;
  }
   if (targetLeftMotorDirection != 0 && currentLeftMotorDirection == 0 && currentActualLeftSpeed >0){
      currentLeftMotorDirection = targetLeftMotorDirection; 
  }


  // --- Right Motor Ramping (similar logic to left) ---
  if (currentRightMotorDirection != 0 && targetRightMotorDirection != currentRightMotorDirection && currentActualRightSpeed > 0) {
    currentActualRightSpeed -= ACCELERATION_STEP * 2;
    if (currentActualRightSpeed < 0) currentActualRightSpeed = 0;
  }
  else if (targetRightMotorDirection == 0 || (targetRightMotorDirection != currentRightMotorDirection && targetRightSpeedMagnitude == 0)) {
    if (currentActualRightSpeed > 0) {
        currentActualRightSpeed -= ACCELERATION_STEP;
        if (currentActualRightSpeed < 0) currentActualRightSpeed = 0;
    }
  }
  else {
    if (currentActualRightSpeed < targetRightSpeedMagnitude) {
        currentActualRightSpeed += ACCELERATION_STEP;
        if (currentActualRightSpeed > targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
    } else if (currentActualRightSpeed > targetRightSpeedMagnitude) {
        currentActualRightSpeed -= ACCELERATION_STEP;
        if (currentActualRightSpeed < targetRightSpeedMagnitude) currentActualRightSpeed = targetRightSpeedMagnitude;
    }
  }
  currentActualRightSpeed = constrain(currentActualRightSpeed, 0, 255);

  if (currentActualRightSpeed == 0) {
    currentRightMotorDirection = targetRightMotorDirection;
  } else if (currentRightMotorDirection != targetRightMotorDirection && currentRightMotorDirection != 0){
     if (targetRightSpeedMagnitude > 0) currentRightMotorDirection = targetRightMotorDirection;
  }
  if (targetRightMotorDirection != 0 && currentRightMotorDirection == 0 && currentActualRightSpeed >0){
      currentRightMotorDirection = targetRightMotorDirection;
  }


  // Apply to Left Motor (M1)
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

  // Apply to Right Motor (M2) 
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

  // Example for Sharp GP2Y0A41SK0F (4-30cm) like sensor.
  // Voltage INCREASES as distance DECREASES.
  // Calibrate these values for YOUR specific IR sensors.
  const float VOLTAGE_AT_MIN_DIST = 2.8; // Approx. voltage at IR_MIN_DISTANCE (e.g., 4cm or 10cm)
  const float VOLTAGE_AT_MAX_DETECT_DIST = 0.42; // Approx. voltage at IR_DETECTION_THRESHOLD or max reliable range
  const float VOLTAGE_AT_10CM = 1.5; // Example intermediate point if needed for piecewise
  const float VOLTAGE_AT_20CM = 0.8; // Example intermediate point

  if (voltage < VOLTAGE_AT_MAX_DETECT_DIST) { 
      distance = IR_MAX_DISTANCE_VAL;
  } else if (voltage > VOLTAGE_AT_MIN_DIST) { 
      distance = IR_MIN_DISTANCE; 
  } else {
      // Piecewise linear interpolation (example, adjust to your sensor's curve)
      if (IR_MIN_DISTANCE <= 4 && voltage > VOLTAGE_AT_10CM) { // e.g. 4-10cm range for GP2Y0A41
         distance = IR_MIN_DISTANCE + (10 - IR_MIN_DISTANCE) * (VOLTAGE_AT_MIN_DIST - voltage) / (VOLTAGE_AT_MIN_DIST - VOLTAGE_AT_10CM);
      } else if (voltage > VOLTAGE_AT_20CM) { // e.g. 10-20cm range
         distance = 10 + (20 - 10) * (VOLTAGE_AT_10CM - voltage) / (VOLTAGE_AT_10CM - VOLTAGE_AT_20CM);
      } else { // e.g. 20-30/70cm range
         distance = 20 + (IR_DETECTION_THRESHOLD - 20) * (VOLTAGE_AT_20CM - voltage) / (VOLTAGE_AT_20CM - VOLTAGE_AT_MAX_DETECT_DIST);
      }
  }

  distance = constrain(distance, IR_MIN_DISTANCE, IR_MAX_DISTANCE_VAL);

  if (!actualMaxVal && distance > IR_DETECTION_THRESHOLD) {
      return IR_DETECTION_THRESHOLD + 1; 
  }
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); // Timeout for HC-SR04
  long distance = duration * 0.034 / 2; 

  if (distance == 0 || distance < 2) { // HC-SR04 min range is ~2cm. 0 often means timeout or too close.
     // If too close or timeout, treat as max distance unless needing actual for very close logic
     if (actualMaxVal && distance < 2 && distance !=0) return distance; // Allow very close actual if non-zero
     return actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1;
  }
  if (distance > US_MAX_DISTANCE_VAL) {
     return actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1;
  }

  if (!actualMaxVal && distance > US_DETECTION_THRESHOLD_BACK) {
      return US_DETECTION_THRESHOLD_BACK + 1; 
  }
  return distance;
}


void updateBumpSensorState() {
  previousBumpSensorState = bumpSensorState; 
  
  bool leftBumped = (digitalRead(BUMP_LEFT) == LOW); 
  bool rightBumped = (digitalRead(BUMP_RIGHT) == LOW);

  bumpSensorState = 0; 
  if (leftBumped) bumpSensorState |= (1 << BUMP_LEFT_BIT);
  if (rightBumped) bumpSensorState |= (1 << BUMP_RIGHT_BIT);
}

// --- Strategy Sub-Functions ---
void searchOpponent() {
  unsigned long currentTime = millis();

  switch (currentSearchState) {
    case SEARCH_STATE_INIT:
      stopMovement(); 
      if (preferredDirection >= 0) { 
          currentSearchState = SEARCH_STATE_PIVOTING_LEFT;
      } else { 
          currentSearchState = SEARCH_STATE_PIVOTING_RIGHT;
      }
      searchStateTimer = currentTime; 
      Serial.print("Search: Init -> "); Serial.println(currentSearchState == SEARCH_STATE_PIVOTING_LEFT ? "Pivoting Left" : "Pivoting Right");
      break;

    case SEARCH_STATE_PIVOTING_LEFT:
      pivotLeft(SPEED_PIVOT_DEFAULT); 
      if (currentTime - searchStateTimer >= SEARCH_PIVOT_DURATION) {
        currentSearchState = SEARCH_STATE_MOVING_FORWARD;
        searchStateTimer = currentTime;
        preferredDirection = -1; 
        Serial.println("Search: Left Pivot Done -> Moving Forward.");
      }
      break;

    case SEARCH_STATE_PIVOTING_RIGHT:
      pivotRight(SPEED_PIVOT_DEFAULT); 
      if (currentTime - searchStateTimer >= SEARCH_PIVOT_DURATION) {
        currentSearchState = SEARCH_STATE_MOVING_FORWARD;
        searchStateTimer = currentTime;
        preferredDirection = 1; 
        Serial.println("Search: Right Pivot Done -> Moving Forward.");
      }
      break;

    case SEARCH_STATE_MOVING_FORWARD:
      moveForward(SPEED_SLOW); 
      if (currentTime - searchStateTimer >= SEARCH_FORWARD_DURATION) {
        currentSearchState = SEARCH_STATE_STOPPING_AFTER_FORWARD;
        searchStateTimer = currentTime;
        Serial.println("Search: Forward Done -> Stopping.");
      }
      break;

    case SEARCH_STATE_STOPPING_AFTER_FORWARD:
      stopMovement();
      if (currentTime - searchStateTimer >= SEARCH_STOP_DURATION) {
        if (preferredDirection == 1) { 
            currentSearchState = SEARCH_STATE_PIVOTING_LEFT; 
        } else { 
            currentSearchState = SEARCH_STATE_PIVOTING_RIGHT; 
        }
        searchStateTimer = currentTime;
        Serial.print("Search: Stop Done -> "); Serial.println(currentSearchState == SEARCH_STATE_PIVOTING_LEFT ? "Pivoting Left" : "Pivoting Right");
      }
      break;
  }
}

void attackWithBumpers() {
  Serial.println("Attack: BUMPER HIT! Full speed ahead!");
  moveForward(SPEED_MAX);
}