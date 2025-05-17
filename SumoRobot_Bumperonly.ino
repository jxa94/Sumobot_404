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

// Constants
const int SPEED_MAX = 220;             // Maximum speed for bumper attacks AND initial charge
const int SPEED_RETREAT = 100;         // Speed for retreating after bump
const int SPEED_PIVOT_DEFAULT = 80;    // Speed for the continuous pivot search

const float RIGHT_MOTOR_FACTOR = 1.0; // Adjust if one motor is consistently faster/slower

const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

// Global variables for robot state
int bumpSensorState = 0;
int previousBumpSensorState = 0;
bool isRetreatingAfterBump = false;
unsigned long retreatStartTime = 0;
const unsigned long BUMPER_RETREAT_DURATION = 300; // How long to retreat

// JSUMO MicroStart switch variables
bool robotActive = false;                 // Is the robot's main logic allowed to run?
bool jsumoSignalWasHighLastFrame = false; // To detect rising edge of JSumo signal

// Initial Forward Charge State
bool isPerformingInitialCharge = false;
unsigned long initialChargeStartTime = 0;
const unsigned long INITIAL_CHARGE_DURATION = 1000; // 1000 ms = 1 second

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0;  // 0: Stop, 1: Forward, -1: Backward
int currentRightMotorDirection = 0; // 0: Stop, 1: Forward, -1: Backward
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 15; 
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 15;

// Function declarations
void setMotorTargets(int leftDir, int leftSpd, int rightDir, int rightSpd);
void updateMotorsWithRamping();
void moveForward(int speed);
void moveBackward(int speed);
void pivotLeft(int speed);
void pivotRight(int speed);
void stopMovement();
void updateBumpSensorState();
void attackWithBumpers();
void executeStrategy();
void handleJsumoSwitch();

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Setup (1s Fwd Charge on Start, then Stop) Starting...");

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);

  pinMode(BUMP_LEFT, INPUT_PULLUP);
  pinMode(BUMP_RIGHT, INPUT_PULLUP);

  pinMode(JSUMO_SWITCH, INPUT_PULLUP);

  stopMovement();
  isRetreatingAfterBump = false;
  Serial.println("Setup Complete. Waiting for MicroStart Signal.");
}

void loop() {
  handleJsumoSwitch();

  if (isPerformingInitialCharge) {
    // The moveForward(SPEED_MAX) command was given in handleJsumoSwitch.
    // updateMotorsWithRamping will continue to execute it.
    if (millis() - initialChargeStartTime >= INITIAL_CHARGE_DURATION) {
      Serial.println("Initial 1s forward charge complete. Stopping, then transitioning to main strategy.");
      isPerformingInitialCharge = false;
      stopMovement(); // <<< EDITED: Re-added stopMovement() here
    }
    // Note: If a bumper is hit *during* the initial charge, 
    // executeStrategy will handle it once the charge duration is over and robot is stopped.
  } else if (robotActive) { // Not in initial charge, but robot is active
    updateBumpSensorState();
    executeStrategy();
  } else { // Robot is not active (and not in initial charge)
    stopMovement();
    isRetreatingAfterBump = false;
  }

  updateMotorsWithRamping();
}

void handleJsumoSwitch() {
  bool signalIsCurrentlyHigh = (digitalRead(JSUMO_SWITCH) == HIGH);

  if (signalIsCurrentlyHigh && !jsumoSignalWasHighLastFrame) { // Rising edge detected
    if (!robotActive && !isPerformingInitialCharge) { // Only activate if not already running or charging
      Serial.println("Robot Activated by MicroStart. Starting 1s forward charge!");
      isPerformingInitialCharge = true;
      initialChargeStartTime = millis();
      robotActive = true; // Robot is active, but in a special initial phase
      
      // Reset other relevant states at activation
      bumpSensorState = 0;
      previousBumpSensorState = 0;
      isRetreatingAfterBump = false;
      
      moveForward(SPEED_RETREAT); // Start the initial forward charge
    }
  }
  jsumoSignalWasHighLastFrame = signalIsCurrentlyHigh;
}

void executeStrategy() {
  // This function is now called AFTER the initial 1s charge and subsequent stopMovement()
  // or if the robot was already active and not in the charge phase.

  // Priority 1: Bumper is currently pressed
  if (bumpSensorState > 0) {
    attackWithBumpers();
    isRetreatingAfterBump = false;
    return;
  }

  // Priority 2: Bumper was just released
  if (previousBumpSensorState > 0 && bumpSensorState == 0) {
    Serial.println("Strategy: Bumper released. Initiating brief retreat.");
    isRetreatingAfterBump = true;
    retreatStartTime = millis();
    moveBackward(SPEED_RETREAT);
    return;
  }

  // Priority 3: Currently retreating
  if (isRetreatingAfterBump) {
    if (millis() - retreatStartTime >= BUMPER_RETREAT_DURATION) {
      Serial.println("Strategy: Retreat complete. Resuming pivot search.");
      isRetreatingAfterBump = false;
      stopMovement(); 
    }
    return;
  }

  // Priority 4: No bump, not retreating -> Pivot
  // This will be called after the robot has stopped (due to stopMovement() in loop or after retreat)
  pivotLeft(SPEED_PIVOT_DEFAULT);
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
  if ((currentLeftMotorDirection != 0 && targetLeftMotorDirection != currentLeftMotorDirection && currentActualLeftSpeed > 0) ||
      (targetLeftMotorDirection == 0 && currentActualLeftSpeed > 0)) {
    currentActualLeftSpeed -= ACCELERATION_STEP * 2; 
    if (currentActualLeftSpeed < 0) currentActualLeftSpeed = 0;
  } else { 
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
  } else if (targetLeftMotorDirection != 0) { 
    currentLeftMotorDirection = targetLeftMotorDirection;
  }


  // --- Right Motor Ramping (similar logic to left) ---
  if ((currentRightMotorDirection != 0 && targetRightMotorDirection != currentRightMotorDirection && currentActualRightSpeed > 0) ||
      (targetRightMotorDirection == 0 && currentActualRightSpeed > 0)) {
    currentActualRightSpeed -= ACCELERATION_STEP * 2;
    if (currentActualRightSpeed < 0) currentActualRightSpeed = 0;
  } else {
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
  } else if (targetRightMotorDirection != 0) {
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
  }

  // Apply to Right Motor (M2) - Mirrored if M2_LPWM is Forward
  if (currentRightMotorDirection == 1) { // Forward
    analogWrite(M2_LPWM, currentActualRightSpeed);
    analogWrite(M2_RPWM, 0);
  } else if (currentRightMotorDirection == -1) { // Backward
    analogWrite(M2_RPWM, currentActualRightSpeed);
    analogWrite(M2_LPWM, 0);
  } else { // Stop (currentRightMotorDirection == 0)
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 0);
  }
}

void updateBumpSensorState() {
  previousBumpSensorState = bumpSensorState;

  bool leftBumped = (digitalRead(BUMP_LEFT) == LOW);
  bool rightBumped = (digitalRead(BUMP_RIGHT) == LOW);

  bumpSensorState = 0;
  if (leftBumped) bumpSensorState |= (1 << BUMP_LEFT_BIT);
  if (rightBumped) bumpSensorState |= (1 << BUMP_RIGHT_BIT);
}

void attackWithBumpers() {
  moveForward(SPEED_MAX);
}