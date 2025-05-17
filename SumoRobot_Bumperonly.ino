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
const int SPEED_MAX = 200;             // Maximum speed for bumper attacks
const int SPEED_RETREAT = 100;         // Speed for retreating after bump
const int SPEED_PIVOT_DEFAULT = 70;    // Speed for the continuous pivot search

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

// --- Motor Ramping Variables ---
int currentActualLeftSpeed = 0;
int currentActualRightSpeed = 0;
int targetLeftSpeedMagnitude = 0;
int targetRightSpeedMagnitude = 0;
int currentLeftMotorDirection = 0;  // 0: Stop, 1: Forward, -1: Backward
int currentRightMotorDirection = 0; // 0: Stop, 1: Forward, -1: Backward
int targetLeftMotorDirection = 0;
int targetRightMotorDirection = 0;

const int ACCELERATION_STEP = 15; // How much to change speed per ramp interval (tune for responsiveness)
unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 15; // ms, how often to update motor speeds (tune for responsiveness)

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
  Serial.println("Simplified Robot Setup (No Delay) Starting...");

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

  if (robotActive) {
    updateBumpSensorState(); // Read bumpers
    executeStrategy();       // Decide what to do
  } else {
    // Robot is not active (e.g., before first start signal or if explicitly deactivated later)
    stopMovement();
    isRetreatingAfterBump = false; // Reset retreat state if deactivated
  }

  updateMotorsWithRamping(); // Apply speeds to motors
}

void handleJsumoSwitch() {
  bool signalIsCurrentlyHigh = (digitalRead(JSUMO_SWITCH) == HIGH);

  if (signalIsCurrentlyHigh && !jsumoSignalWasHighLastFrame) { // Rising edge detected
    if (!robotActive) { // Only activate if not already running
      Serial.println("Robot Activated by MicroStart. Engaging Strategy Immediately!");
      robotActive = true; // Robot is now fully active
      
      // Reset relevant states at activation
      stopMovement(); // Ensure motors are stopped before strategy begins
      bumpSensorState = 0;
      previousBumpSensorState = 0;
      isRetreatingAfterBump = false;
      // Ensure motor targets are also reset (stopMovement handles this)
    } else {
      // Serial.println("MicroStart Signal HIGH, but robot already active. No change.");
    }
  }
  // If signal goes low, robot remains active due to robotActive flag
  jsumoSignalWasHighLastFrame = signalIsCurrentlyHigh; // Update for next cycle
}

void executeStrategy() {
  // Priority 1: Bumper is currently pressed
  if (bumpSensorState > 0) {
    attackWithBumpers();
    isRetreatingAfterBump = false; // Stop any retreat if new bump occurs
    // Serial.println("Strategy: BUMPER ACTIVE! ATTACKING!");
    return;
  }

  // Priority 2: Bumper was just released (transition from pressed to not pressed)
  if (previousBumpSensorState > 0 && bumpSensorState == 0) {
    Serial.println("Strategy: Bumper released. Initiating brief retreat.");
    isRetreatingAfterBump = true;
    retreatStartTime = millis();
    moveBackward(SPEED_RETREAT);
    return;
  }

  // Priority 3: Currently retreating after a bumper release
  if (isRetreatingAfterBump) {
    if (millis() - retreatStartTime >= BUMPER_RETREAT_DURATION) {
      Serial.println("Strategy: Retreat complete. Resuming pivot search.");
      isRetreatingAfterBump = false;
      stopMovement(); // Stop briefly before pivoting again
    } else {
      // Continue retreating (moveBackward already called when retreat started)
      // Serial.println("Strategy: Retreating...");
    }
    return;
  }

  // Priority 4: No bump active, not retreating -> Default behavior: Turn continuously.
  // Serial.println("Strategy: No opponent contact. Pivoting...");
  pivotLeft(SPEED_PIVOT_DEFAULT); // Continuously pivot left (or right, or alternate)
  // Example for alternating pivot:
  // static unsigned long lastPivotSwitchTime = 0;
  // static bool pivotDirectionLeft = true;
  // const unsigned long PIVOT_SWITCH_INTERVAL = 3000; // Pivot for 3s in one direction

  // if (millis() - lastPivotSwitchTime > PIVOT_SWITCH_INTERVAL) {
  //   pivotDirectionLeft = !pivotDirectionLeft;
  //   lastPivotSwitchTime = millis();
  //   stopMovement(); // Brief stop before changing pivot direction for smoother transition
  //   Serial.print("Strategy: Switching pivot direction to ");
  //   Serial.println(pivotDirectionLeft ? "LEFT" : "RIGHT");
  // }

  // // Ensure stopMovement has a chance to ramp down before new pivot command if switch just happened
  // if (millis() - lastPivotSwitchTime < MOTOR_RAMP_INTERVAL * 5 && millis() != lastPivotSwitchTime) { // Wait a few ramp cycles
  //     // Motors are stopping or have just stopped
  // } else {
  //    if (pivotDirectionLeft) {
  //      pivotLeft(SPEED_PIVOT_DEFAULT);
  //    } else {
  //      pivotRight(SPEED_PIVOT_DEFAULT);
  //    }
  // }
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
  setMotorTargets(-1, speed, 1, speed); // Left motor backward, Right motor forward
}

void pivotRight(int speed) {
  setMotorTargets(1, speed, -1, speed); // Left motor forward, Right motor backward
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