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
const int SPEED_SLOW = 80; // MODIFIED: Increased from 50. Very low speeds might appear as a stop or stall motors.
const int SPEED_MEDIUM = 100;
const int SPEED_FAST = 150; // Used for pivot outer wheel speed & zero-radius pivot speed
const int SPEED_MAX = 200;
const int SPEED_RETREAT = SPEED_MEDIUM; // NEW: Speed for bumper retreat

const float RIGHT_MOTOR_FACTOR = 1.0;

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

const int IR_MAX_DISTANCE_VAL = 999; // Represents out of range for IR
const int IR_MIN_DISTANCE = 10; // Min reliable distance for IR Sharp sensors (e.g., GP2Y0A41SK0F is 4cm, GP2Y0A21YK0F is 10cm, adjust if different)
const int IR_DETECTION_THRESHOLD = 70; // Objects further than this are "not detected" by default by IR
const int IR_CLOSE_THRESHOLD = 30; // Objects closer than this might warrant faster approach or specific tactics

const int US_MAX_DISTANCE_VAL = 999; // Represents out of range for US
const int US_DETECTION_THRESHOLD_BACK = 40; // Detection threshold for US (e.g., 40cm)
const int US_VERY_CLOSE_BACK = 15; // Very close threshold for US (e.g., 15cm)

// Global variables for robot state
unsigned long initialScanStartTime = 0;
bool isPerformingInitialScan = false;

int bumpSensorState = 0;
int previousBumpSensorState = 0;
bool isRetreatingAfterBump = false;
unsigned long retreatStartTime = 0;
const unsigned long BUMPER_RETREAT_DURATION = 500; // 0.5 seconds retreat time

int preferredDirection = 0; // -1 for left, 0 for center/none, 1 for right
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
const unsigned long MAX_SPECIAL_TURN_DURATION = 4000; // 4 seconds

// Cooldown for US-triggered special turns
unsigned long usSpecialTurnCooldownUntil = 0;
const unsigned long US_SPECIAL_TURN_COOLDOWN_DURATION = 3000; // 3 seconds cooldown

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0; // 0=stop, 1=fwd, -1=bwd
int currentRightMotorDirection = 0;// 0=stop, 1=fwd, -1=bwd
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 10;
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 20; // ms


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

  stopMovement(); // Initialize motors to stop
  jsumoSignalWasHighLastFrame = (digitalRead(JSUMO_SWITCH) == HIGH);
  isRetreatingAfterBump = false;
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
}

void loop() {
  handleJsumoSwitch();

  if (robotActive) {
    if (isPerformingInitialScan) {
      // During initial scan, robot should ideally be stationary or perform a predefined scan pattern.
      // Current logic keeps it stopped and updates preferredDirection based on sensors.
      stopMovement(); // Ensure robot is stopped during this phase
      performInitialScan(); // Reads sensors, updates preferredDirection

      if (millis() - initialScanStartTime >= 3000) { // Initial scan duration
        isPerformingInitialScan = false;
        currentSpecialTurn = TURN_STATE_NONE; // Reset special turn state
        Serial.println("Initial 3s scan complete. Starting main strategy.");
        currentSearchState = SEARCH_STATE_INIT; // Initialize search state
        searchStateTimer = millis();
        usSpecialTurnCooldownUntil = 0; // Reset US turn cooldown
      }
    } else { // Main operational logic after initial scan
      // Handle ongoing special US-triggered pivot
      if (currentSpecialTurn != TURN_STATE_NONE) {
        int irLeft = getIRDistance(IR_REFLECT_LEFT, true); // Use true to get actual values for decision
        int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
        int irRight = getIRDistance(IR_REFLECT_RIGHT, true);

        bool targetAcquiredByIR = (irLeft < IR_DETECTION_THRESHOLD ||
                                   irCenter < IR_DETECTION_THRESHOLD ||
                                   irRight < IR_DETECTION_THRESHOLD);

        if (targetAcquiredByIR) {
          Serial.println("Special Pivot: Target ACQUIRED by IR.");
          stopMovement(); // Stop to re-evaluate based on new IR lock
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = true;
          lastOpponentDetectionTime = millis();
          // Update preferred direction based on IR that acquired target
          if (irCenter < irLeft && irCenter < irRight && irCenter < IR_DETECTION_THRESHOLD) preferredDirection = 0;
          else if (irLeft < irRight && irLeft < IR_DETECTION_THRESHOLD) preferredDirection = -1;
          else if (irRight < IR_DETECTION_THRESHOLD) preferredDirection = 1;
          // Cooldown is NOT set here; let immediate strategy take over.
        } else if (millis() - specialTurnStartTime > MAX_SPECIAL_TURN_DURATION) {
          Serial.println("Special Pivot: MAX DURATION reached without IR lock.");
          stopMovement(); // Stop if pivot times out
          currentSpecialTurn = TURN_STATE_NONE;
          isOpponentVisible = false; // Opponent not found during pivot
          usSpecialTurnCooldownUntil = millis() + US_SPECIAL_TURN_COOLDOWN_DURATION; // Cooldown after timeout
        } else {
          // Continue pivoting
          if (currentSpecialTurn == TURN_STATE_US_TRIGGERED_PIVOT_LEFT) {
            pivotLeft(SPEED_MEDIUM);
          } else { // TURN_STATE_US_TRIGGERED_PIVOT_RIGHT
            pivotRight(SPEED_MEDIUM);
          }
        }
      }
      
      // If not in a special US-triggered pivot, run main strategy
      if (currentSpecialTurn == TURN_STATE_NONE) { 
            updateBumpSensorState(); // Update here so executeStrategy has current and previous
            executeStrategy();
      }
    }
  } else { // Robot not active
    stopMovement();
    currentSpecialTurn = TURN_STATE_NONE;
    isPerformingInitialScan = false;
    isRetreatingAfterBump = false;
    usSpecialTurnCooldownUntil = 0;
  }

  updateMotorsWithRamping(); // Update motor speeds based on targets and ramping
}

void handleJsumoSwitch() {
  bool signalIsCurrentlyHigh = (digitalRead(JSUMO_SWITCH) == HIGH);

  if (signalIsCurrentlyHigh && !jsumoSignalWasHighLastFrame) {
    if (!robotActive) {
      Serial.println("Robot Activated by MicroStart (Signal HIGH). Starting initial scan.");
      robotActive = true;
      isPerformingInitialScan = true;
      initialScanStartTime = millis();
      // Reset states
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
      stopMovement(); // Ensure motors are stopped at activation
    }
  } else if (!signalIsCurrentlyHigh && jsumoSignalWasHighLastFrame) {
    if (robotActive) {
      Serial.println("Robot Deactivated by MicroStart (Signal LOW).");
      robotActive = false;
      isPerformingInitialScan = false;
      isRetreatingAfterBump = false;
      stopMovement(); // Ensure motors are stopped at deactivation
    }
  }
  jsumoSignalWasHighLastFrame = signalIsCurrentlyHigh;
}


void performInitialScan() {
  // Reads all sensors to get an initial idea of the opponent's location.
  // The robot should be stationary during this scan.
  int irLeft = getIRDistance(IR_REFLECT_LEFT, true);
  int irCenter = getIRDistance(IR_REFLECT_CENTER, true);
  int irRight = getIRDistance(IR_REFLECT_RIGHT, true);
  long backDist = getUltrasonicDistance(true);

  isOpponentVisible = false; // Assume not visible until confirmed

  int minFrontIRDistance = IR_MAX_DISTANCE_VAL;
  bool frontIRDetecting = false;
  if (irLeft < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irLeft); frontIRDetecting = true; }
  if (irCenter < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irCenter); frontIRDetecting = true; }
  if (irRight < IR_DETECTION_THRESHOLD) { minFrontIRDistance = min(minFrontIRDistance, irRight); frontIRDetecting = true; }

  // Prioritize US if it's detecting and front IRs are not, or if US is closer than front IRs
  if (backDist < US_DETECTION_THRESHOLD_BACK && (backDist < minFrontIRDistance || !frontIRDetecting)) {
    isOpponentVisible = true; 
    lastOpponentDetectionTime = millis(); 
    // If US is primary, set a random pivot direction as US doesn't give L/R info
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
    } else { // Ambiguous front IR, but detected
        preferredDirection = 0; // Neutral, or could be random
        Serial.println("Initial Scan: Front IR ambiguous. Preferred direction neutral.");
    }
  } else {
      // No opponent detected by any sensor
      preferredDirection = 0; 
      Serial.println("Initial Scan: No opponent detected.");
  }
}


void executeStrategy() {
  // Bumper logic has highest priority
  if (isRetreatingAfterBump) {
    if (bumpSensorState > 0) { // Bumper hit again during retreat
      Serial.println("Retreat: Interrupted by new bumper press. Attacking.");
      isRetreatingAfterBump = false;
      attackWithBumpers(); 
      lastOpponentDetectionTime = millis();
      isOpponentVisible = true; // We've made contact
    } else if (millis() - retreatStartTime >= BUMPER_RETREAT_DURATION) {
      Serial.println("Retreat: Duration complete. Resuming normal ops.");
      isRetreatingAfterBump = false;
      stopMovement(); // Stop briefly before deciding next action
    } else {
      moveBackward(SPEED_RETREAT); // Continue retreating
    }
    return; // Exit strategy if retreating
  }

  // Check for new bumper press (if not currently retreating)
  if (bumpSensorState > 0) {
    attackWithBumpers();
    lastOpponentDetectionTime = millis();
    isOpponentVisible = true;
    currentSpecialTurn = TURN_STATE_NONE; // Cancel any special turn
    usSpecialTurnCooldownUntil = 0; // Reset cooldown
    return; // Exit strategy after initiating attack
  }

  // Check for bumper release (if not retreating and was previously pressed)
  if (previousBumpSensorState > 0 && bumpSensorState == 0) {
    Serial.println("Bumper released. Initiating retreat.");
    isRetreatingAfterBump = true;
    retreatStartTime = millis();
    moveBackward(SPEED_RETREAT); 
    return; // Exit strategy after initiating retreat
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
  // - US turn not on cooldown
  // - US is detecting an object
  // - US is the primary detection (closer than any front IR, or front IRs not detecting)
  // - Not already in a special turn
  if (millis() >= usSpecialTurnCooldownUntil &&
      usDetecting && (backDist < minFrontIRDistance || !frontIRDetecting) && 
      currentSpecialTurn == TURN_STATE_NONE) {
    Serial.println("Strategy: US is primary target. Initiating SPECIAL PIVOT.");
    isOpponentVisible = true; // Assume US sees an opponent
    lastOpponentDetectionTime = millis();
    specialTurnStartTime = millis();

    int turnDecision = preferredDirection; // Use last known direction if available
    if (turnDecision == 0) { // If no preference, or centered, choose randomly for US
        turnDecision = (random(2) == 0) ? -1 : 1;
    }

    if (turnDecision <= 0) { // Includes -1 (left) and 0 (randomly chose left)
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_LEFT;
    } else { // 1 (right or randomly chose right)
         currentSpecialTurn = TURN_STATE_US_TRIGGERED_PIVOT_RIGHT;
    }
    // Action will be handled by the currentSpecialTurn block in loop()
    return; 
  }

  // This part runs if not initiating/in a special turn
  // Note: currentSpecialTurn should be TURN_STATE_NONE if we reach here due to loop structure
  // and the return statement above.
  if (frontIRDetecting) {
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist);
    usSpecialTurnCooldownUntil = 0; // IR has taken over, reset US turn cooldown
  } else if (usDetecting) { 
    // US is detecting, but not as primary for a special turn (e.g., cooldown active, or front IR also saw something but not strongly)
    isOpponentVisible = true;
    lastOpponentDetectionTime = millis();
    Serial.println("Strategy: US sees opponent (not primary for special pivot OR cooldown active). Adjusting via US logic.");
    adjustDirectionToOpponent(irLeft, irCenter, irRight, backDist); // adjustDirection will handle US if front IRs are clear
  } else {
    // No front IR, no US detection
    if (millis() - lastOpponentDetectionTime < 1000 && isOpponentVisible) { // Lost opponent briefly
      Serial.println("Strategy: Lost opponent briefly. Reacquiring based on last preferred direction.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
      else moveForward(SPEED_MEDIUM); // If last was center, try moving forward
    } else { // Opponent truly lost or never found
      isOpponentVisible = false;
      Serial.println("Strategy: Opponent lost or not found. Searching.");
      searchOpponent();
    }
  }
}

// MODIFIED adjustDirectionToOpponent
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  bool usVeryClose = (backDist < US_VERY_CLOSE_BACK); // e.g., < 15cm
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
    arcLeft(SPEED_FAST, (leftDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = -1;
    return;
  }
  // No need to check "rightDist < leftDist" as it's implied if the "leftDist < rightDist" above wasn't met
  if (rightDist < IR_DETECTION_THRESHOLD) { 
    Serial.print("Adjust: Opponent RIGHT (IR: "); Serial.print(rightDist); Serial.println("cm). Arc turning right.");
    arcRight(SPEED_FAST, (rightDist < IR_CLOSE_THRESHOLD ? SPEED_TURN_PIVOT_INNER : SPEED_SLOW));
    preferredDirection = 1;
    return;
  }

  // Priority 3: US detection if front IRs are not decisive OR US indicates critical close threat
  // This block is reached if the IR conditions above were NOT met.
  // `currentSpecialTurn == TURN_STATE_NONE` check is implicitly handled as this function is called when it's NONE.
  if (backDist < US_DETECTION_THRESHOLD_BACK && (usVeryClose || frontSensorsClear)) {
      Serial.print("Adjust: Opponent BEHIND or US primary (IRs not decisive). US: "); Serial.print(backDist); Serial.println("cm. Pivoting by US.");
      int pivotDir = preferredDirection; 
      if (pivotDir == 0) pivotDir = (random(2) == 0) ? -1 : 1; // Default pivot if no strong preference

      if (pivotDir <= 0) pivotLeft(SPEED_SLOW); 
      else pivotRight(SPEED_SLOW); 
      // Set preferredDirection based on pivot, helps if US lock is lost next cycle
      preferredDirection = (pivotDir <=0) ? -1 : 1; 
      return;
  }

  // Priority 4: Fallback if sensor data is ambiguous but opponent was recently visible
  if (isOpponentVisible) { 
      Serial.println("Adjust: Opponent was visible but current sensors ambiguous. Pivoting based on last known direction.");
      if (preferredDirection < 0) pivotLeft(SPEED_MEDIUM);
      else if (preferredDirection > 0) pivotRight(SPEED_MEDIUM);
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

void arcLeft(int outerSpeed, int innerSpeed) { // Left motor is inner, Right motor is outer
  setMotorTargets(1, innerSpeed, 1, outerSpeed);
}

void arcRight(int outerSpeed, int innerSpeed) { // Right motor is inner, Left motor is outer
  setMotorTargets(1, outerSpeed, 1, innerSpeed);
}

void pivotLeft(int speed) { // Left backward, Right forward
  setMotorTargets(-1, speed, 1, speed);
}

void pivotRight(int speed) { // Left forward, Right backward
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
  // If direction changes and motor was moving, decelerate faster
  if (currentLeftMotorDirection != 0 && targetLeftMotorDirection != currentLeftMotorDirection && currentActualLeftSpeed > 0) {
    currentActualLeftSpeed -= ACCELERATION_STEP * 2; // Faster deceleration for direction change
    if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
  }
  // If target is stop, or direction changes to stop
  else if (targetLeftMotorDirection == 0 || (targetLeftMotorDirection != currentLeftMotorDirection && targetLeftSpeedMagnitude == 0)) {
    if (currentActualLeftSpeed > 0) {
        currentActualLeftSpeed -= ACCELERATION_STEP;
        if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
    }
  }
  // Accelerate or maintain speed towards target
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

  // Update direction after speed adjustment, especially if speed reached 0
  if (currentActualLeftSpeed == 0) {
    currentLeftMotorDirection = targetLeftMotorDirection;
  } else if (currentLeftMotorDirection != targetLeftMotorDirection && targetLeftMotorDirection != 0) {
    // If changing direction while not stopped (e.g. from fwd to bwd directly)
    // This case is tricky with simple ramping; ideally, it should ramp to 0 first.
    // The above logic handles deceleration first. If still >0, it will keep old dir.
    // For simplicity, if target dir is non-zero and current speed becomes non-zero, adopt target dir.
    if (targetLeftSpeedMagnitude > 0) currentLeftMotorDirection = targetLeftMotorDirection;
  }
   if (targetLeftMotorDirection != 0 && currentLeftMotorDirection == 0 && currentActualLeftSpeed >0){
      currentLeftMotorDirection = targetLeftMotorDirection; // If starting from stop
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
  } else if (currentRightMotorDirection != targetRightMotorDirection && targetRightMotorDirection != 0){
     if (targetRightSpeedMagnitude > 0) currentRightMotorDirection = targetRightMotorDirection;
  }
  if (targetRightMotorDirection != 0 && currentRightMotorDirection == 0 && currentActualRightSpeed >0){
      currentRightMotorDirection = targetRightMotorDirection;
  }


  // Apply to Left Motor (M1)
  if (currentLeftMotorDirection == 1) { // Forward
    analogWrite(M1_RPWM, currentActualLeftSpeed);
    analogWrite(M1_LPWM, 0);
  } else if (currentLeftMotorDirection == -1) { // Backward
    analogWrite(M1_LPWM, currentActualLeftSpeed);
    analogWrite(M1_RPWM, 0);
  } else { // Stop (currentLeftMotorDirection == 0)
    analogWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, 0);
    // currentActualLeftSpeed should be 0 if direction is 0
  }

  // Apply to Right Motor (M2) - Note M2_LPWM is assumed FORWARD
  if (currentRightMotorDirection == 1) { // Forward
    analogWrite(M2_LPWM, currentActualRightSpeed); // M2_LPWM for forward
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // Backward
    analogWrite(M2_RPWM, currentActualRightSpeed); // M2_RPWM for backward
    analogWrite(M2_LPWM, 0);
  } else { // Stop (currentRightMotorDirection == 0)
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
    // currentActualRightSpeed should be 0 if direction is 0
  }
}


// --- Sensor Functions ---
int getIRDistance(int sensorPin, bool actualMaxVal) {
  int reading = analogRead(sensorPin);
  // This is a simplified conversion. For accurate distances, you need to calibrate
  // each sensor and use a lookup table or a fitted curve equation.
  // The Sharp GP2Y0Axx sensors have a non-linear response.
  // Example for a generic IR sensor (replace with your sensor's characteristics)
  float voltage = reading * (5.0 / 1023.0);
  float distance;

  // Rough conversion for a common type of Sharp IR sensor (e.g., GP2Y0A21YK0F like)
  // Adjust these values based on your specific sensor model and calibration!
  // This example assumes a sensor that reads higher voltage for closer objects.
  if (voltage < 0.4) { // Below minimum reliable voltage reading (object too far or no object)
      distance = IR_MAX_DISTANCE_VAL;
  } else if (voltage > 3.0) { // Above maximum reliable voltage (object too close)
      distance = IR_MIN_DISTANCE; 
  } else {
      // Simplified inverse power law approximation: distance = k / (reading - offset)
      // Or a piece-wise linear approximation.
      // This is a placeholder. You MUST calibrate your sensors.
      // Example (very rough, adjust based on sensor):
      // distance = 27.728 * pow(voltage, -1.2045); // Common for GP2Y0A21
      // For this example, using the provided piecewise logic:
      if (voltage > 2.8) { distance = 4; } // Closest range, e.g. for GP2Y0A41 (4-30cm)
      else if (voltage > 1.5) { distance = 4 + (10-4) * (2.8 - voltage) / (2.8 - 1.5); } 
      else if (voltage > 0.8) { distance = 10 + (20-10) * (1.5 - voltage) / (1.5 - 0.8); } 
      else if (voltage > 0.42) { distance = 20 + (30-20) * (0.8 - voltage) / (0.8 - 0.42); } 
      else { distance = IR_MAX_DISTANCE_VAL; } // Should be caught by first 'if'
  }

  distance = constrain(distance, IR_MIN_DISTANCE, IR_MAX_DISTANCE_VAL);

  if (!actualMaxVal && distance > IR_DETECTION_THRESHOLD) {
      return IR_DETECTION_THRESHOLD + 1; // Return a value just over threshold
  }
  return (int)distance;
}

long getUltrasonicDistance(bool actualMaxVal) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // Timeout for pulseIn: Max range ~3.5m for 20ms, ~4.2m for 25ms
  // HC-SR04 typical max range is 4m. A 25000us (25ms) timeout is reasonable.
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000); 
  long distance = duration * 0.034 / 2; // Speed of sound = 0.034 cm/us

  if (distance == 0 || distance > US_MAX_DISTANCE_VAL) { // Timeout or out of practical range
     return actualMaxVal ? US_MAX_DISTANCE_VAL : US_DETECTION_THRESHOLD_BACK + 1;
  }
  if (!actualMaxVal && distance > US_DETECTION_THRESHOLD_BACK) {
      return US_DETECTION_THRESHOLD_BACK + 1; // Cap at threshold if not needing actual max
  }
  return distance;
}


void updateBumpSensorState() {
  previousBumpSensorState = bumpSensorState; 
  
  bool leftBumped = (digitalRead(BUMP_LEFT) == LOW); // Assuming LOW means pressed for pull-up
  bool rightBumped = (digitalRead(BUMP_RIGHT) == LOW);

  bumpSensorState = 0; // Reset current state
  if (leftBumped) bumpSensorState |= (1 << BUMP_LEFT_BIT);
  if (rightBumped) bumpSensorState |= (1 << BUMP_RIGHT_BIT);
}

// --- Strategy Sub-Functions ---
void searchOpponent() {
  unsigned long currentTime = millis();

  switch (currentSearchState) {
    case SEARCH_STATE_INIT:
      stopMovement(); 
      // Pivot away from the last known direction, or default to left pivot start
      if (preferredDirection >= 0) { // If last saw right, or center/none, pivot left first
          currentSearchState = SEARCH_STATE_PIVOTING_LEFT;
      } else { // If last saw left, pivot right first
          currentSearchState = SEARCH_STATE_PIVOTING_RIGHT;
      }
      searchStateTimer = currentTime; 
      Serial.print("Search: Init -> "); Serial.println(currentSearchState == SEARCH_STATE_PIVOTING_LEFT ? "Pivoting Left" : "Pivoting Right");
      break;

    case SEARCH_STATE_PIVOTING_LEFT:
      pivotLeft(SPEED_MEDIUM);
      // preferredDirection = -1; // Update preferred direction while pivoting
      if (currentTime - searchStateTimer >= SEARCH_PIVOT_DURATION) {
        currentSearchState = SEARCH_STATE_MOVING_FORWARD;
        searchStateTimer = currentTime;
        preferredDirection = -1; // Set based on the pivot completed
        Serial.println("Search: Left Pivot Done -> Moving Forward.");
      }
      break;

    case SEARCH_STATE_PIVOTING_RIGHT:
      pivotRight(SPEED_MEDIUM);
      // preferredDirection = 1;
      if (currentTime - searchStateTimer >= SEARCH_PIVOT_DURATION) {
        currentSearchState = SEARCH_STATE_MOVING_FORWARD;
        searchStateTimer = currentTime;
        preferredDirection = 1; // Set based on the pivot completed
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
        // After moving forward, alternate pivot direction from the last pivot
        if (preferredDirection == 1) { // If last moved forward after pivoting right
            currentSearchState = SEARCH_STATE_PIVOTING_LEFT; // Now pivot left
        } else { // If last moved forward after pivoting left (or init default)
            currentSearchState = SEARCH_STATE_PIVOTING_RIGHT; // Now pivot right
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