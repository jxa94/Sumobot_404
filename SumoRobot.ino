#include <Arduino.h>

// BTS7960 Motor control pins
// Motor 1 (Left)
#define M1_RPWM 11  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1_LPWM 10  // Left/Backward PWM pin - connect to LPWM on BTS7960
// EN pins are connected to 5V directly

// Motor 2 (Right)
#define M2_RPWM 6   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2_LPWM 5   // Left/Backward PWM pin - connect to LPWM on BTS7960
// EN pins are connected to 5V directly

// Starter switch: Digital input
#define JSUMO_SWITCH 7

// Bump sensors - moved from RX/TX pins
#define BUMP_LEFT 8   // Moved from RX pin (D0) to D8
#define BUMP_RIGHT 9  // Moved from TX pin (D1) to D9

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int SPEEDS[] = {40, 80, 120, 160, 200}; 
const int ONstate = 1;
const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

// IR sensor thresholds and constants
const int IR_MAX_DISTANCE = 150; // cm - maximum reliable distance
const int IR_MIN_DISTANCE = 20;  // cm - minimum reliable distance
const int IR_DETECTION_THRESHOLD = 80; // cm - consider opponent detected below this value

// Global variables
unsigned long startTime = 0;
bool isInitialScanComplete = false;
unsigned long lastScanTime = 0;
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int bumpSensorState = 0;
int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side

// Initial scan results
bool opponentDetectedFront = false;
bool opponentDetectedBack = false;
int opponentDirection = 0; // 0: none, 1: left, 2: center, 3: right, 4: back

// JSUMO switch state tracking
bool previousSwitchState = false;
bool currentSwitchState = false;
unsigned long lastSwitchChangeTime = 0;
const unsigned long DEBOUNCE_TIME = 50; // Debounce time in milliseconds
bool robotActive = false; // Flag to track if the robot is active

// Function declarations
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopMovement();
int getIRDistance(int sensorPin);
long getUltrasonicDistance();
void updateSensorStates();
void searchOpponent();
void attackOpponent();
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch();

void setup() {
  // Serial communication can now be enabled since we're not using RX/TX pins
  Serial.begin(9600);  // Initialize serial communication for debugging

  // Motor 1 pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  
  // Motor 2 pins
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  
  // Note: EN pins are now hardwired to 5V so no need to set them

  // IR sensors
  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);

  // Ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Bump sensors
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);

  // Starter switch
  pinMode(JSUMO_SWITCH, INPUT);
  
  // Initialize switch state
  currentSwitchState = digitalRead(JSUMO_SWITCH);
  previousSwitchState = currentSwitchState;
  
  // Ensure motors are stopped
  stopMovement();
}

void loop() {
  // Handle the JSUMO switch state changes
  handleJsumoSwitch();
  
  // Only run robot logic if it's active
  if (robotActive) {
    // Initial 3-second scan without moving
    if (!isInitialScanComplete) {
      if (startTime == 0) {
        startTime = millis();
        // Reset opponent detection flags
        opponentDetectedFront = false;
        opponentDetectedBack = false;
        opponentDirection = 0;
      }
      
      // During the 3-second period, only perform scanning
      if (millis() - startTime < 3000) {
        stopMovement(); // Ensure we're not moving
        performInitialScan(); // Only scan, don't move
        return;
      } else {
        isInitialScanComplete = true;
      }
    }
    
    // Update sensor readings
    updateSensorStates();
    
    // Execute strategy based on sensors and initial scan
    executeStrategy();
  } else {
    // Robot is inactive, ensure motors are stopped
    stopMovement();
    isInitialScanComplete = false;
    startTime = 0;
  }
}

// Handle JSUMO switch state changes with debouncing
void handleJsumoSwitch() {
  // Read the current state of the switch
  bool newSwitchState = digitalRead(JSUMO_SWITCH);
  
  // Check if the switch state has changed
  if (newSwitchState != previousSwitchState) {
    // Update last switch change time
    lastSwitchChangeTime = millis();
  }
  
  // If the state has been stable for the debounce period
  if ((millis() - lastSwitchChangeTime) > DEBOUNCE_TIME) {
    // If the current state is different from the last stable state
    if (newSwitchState != currentSwitchState) {
      currentSwitchState = newSwitchState;
      
      // If the switch is pressed (HIGH)
      if (currentSwitchState == HIGH) {
        // Toggle robot active state
        robotActive = !robotActive;
        
        // Reset scan state if turning robot on
        if (robotActive) {
          isInitialScanComplete = false;
          startTime = 0;
        }
      }
    }
  }
  
  // Save the current reading for next comparison
  previousSwitchState = newSwitchState;
}

// Perform initial scan using only sensors (no movement)
void performInitialScan() {
  // Read IR sensors
  int leftDist = getIRDistance(IR_REFLECT_LEFT);
  int centerDist = getIRDistance(IR_REFLECT_CENTER);
  int rightDist = getIRDistance(IR_REFLECT_RIGHT);
  
  // Read ultrasonic sensor (behind the robot)
  long backDist = getUltrasonicDistance();
  
  // Check front direction (IR sensors)
  if (centerDist < IR_DETECTION_THRESHOLD || 
      leftDist < IR_DETECTION_THRESHOLD || 
      rightDist < IR_DETECTION_THRESHOLD) {
    
    opponentDetectedFront = true;
    
    // Determine most likely direction
    if (centerDist < leftDist && centerDist < rightDist) {
      opponentDirection = 2; // center
    } else if (leftDist < rightDist) {
      opponentDirection = 1; // left
    } else {
      opponentDirection = 3; // right
    }
  }
  
  // Check back direction (ultrasonic)
  if (backDist > 0 && backDist < 30) {
    opponentDetectedBack = true;
    opponentDirection = 4; // back
  }
}

// Execute strategy based on sensors and initial scan
void executeStrategy() {
  // First check bump sensors as they indicate direct contact
  if (bumpSensorState > 0) {
    attackMode = 1;
    attackOpponent();
    return;
  }
  
  // Read current sensor data
  int leftDist = getIRDistance(IR_REFLECT_LEFT);
  int centerDist = getIRDistance(IR_REFLECT_CENTER);
  int rightDist = getIRDistance(IR_REFLECT_RIGHT);
  long backDist = getUltrasonicDistance();
  
  // If opponent is directly detected now, override initial scan
  if (centerDist < IR_DETECTION_THRESHOLD) {
    attackMode = 1;
    moveForward(SPEEDS[3], SPEEDS[3]); // Medium speed ahead
    return;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    attackMode = 2;
    moveForward(SPEEDS[2], SPEEDS[3]); // Turn left while moving forward
    return;
  } else if (rightDist < IR_DETECTION_THRESHOLD && rightDist < leftDist) {
    attackMode = 2;
    moveForward(SPEEDS[3], SPEEDS[2]); // Turn right while moving forward
    return;
  } else if (backDist < 30 && backDist > 0) {
    // Opponent is behind
    attackMode = 2;
    // Turn around to face opponent - use non-blocking approach
    if (scanDirection == 1) {
      turnRight(SPEEDS[4], SPEEDS[4]);
    } else {
      turnLeft(SPEEDS[4], SPEEDS[4]);
    }
    return;
  }
  
  // If nothing directly detected, use initial scan results
  if (opponentDirection > 0) {
    switch (opponentDirection) {
      case 1: // left
        turnLeft(SPEEDS[3], SPEEDS[3]);
        break;
      case 2: // center
        moveForward(SPEEDS[3], SPEEDS[3]); // Medium speed
        break;
      case 3: // right
        turnRight(SPEEDS[3], SPEEDS[3]);
        break;
      case 4: // back
        // Continue turning to face opponent
        if (scanDirection == 1) {
          turnRight(SPEEDS[4], SPEEDS[4]);
        } else {
          turnLeft(SPEEDS[4], SPEEDS[4]);
        }
        break;
    }
  } else {
    // No opponent detected, search mode
    attackMode = 0;
    searchOpponent();
  }
}

// Movement Functions for BTS7960 - MODIFIED for parallel motors
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor forward (one direction)
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  // Right motor forward (opposite direction due to parallel setup)
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Left motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  // Right motor backward (opposite direction due to parallel setup)
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  // Left motor backward, Right motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  // Left motor forward, Right motor forward
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void stopMovement() {
  // Stop all motors
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);
}

// IR Distance conversion (adjust based on your specific sensor model)
int getIRDistance(int sensorPin) {
  int reading = analogRead(sensorPin);
  
  // Convert analog reading (0-1023) to voltage (0-5V)
  float voltage = reading * (5.0 / 1023.0);
  
  // Check if voltage is too low (beyond maximum distance)
  if (voltage < 0.5) return IR_MAX_DISTANCE;
  
  // Convert voltage to distance using the inverse relationship
  // The formula is derived from the second graph showing 1/distance vs voltage
  float inverse_distance = (voltage - 0.42) / 42.5;
  float distance = 1.0 / inverse_distance;
  
  // Apply limits
  if (distance > IR_MAX_DISTANCE) return IR_MAX_DISTANCE;
  if (distance < IR_MIN_DISTANCE) return IR_MIN_DISTANCE;
  
  return (int)distance;
}

// Ultrasonic Distance measurement
long getUltrasonicDistance() {
  // Clear the trigger pin
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin high for 10 microseconds
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // Timeout after 30ms
  
  // Calculate distance in centimeters
  long distance = duration * 0.034 / 2;
  
  return distance;
}

// Update sensor states
void updateSensorStates() {
  // Bump sensors
  bumpSensorState = (digitalRead(BUMP_LEFT) << BUMP_LEFT_BIT) |
                    (digitalRead(BUMP_RIGHT) << BUMP_RIGHT_BIT);
}

// Search pattern to find opponent
void searchOpponent() {
  // Change scan direction every 3 seconds without using delay
  if (millis() - lastScanTime > 3000) {
    scanDirection = -scanDirection;
    lastScanTime = millis();
  }
  
  if (scanDirection == 1) {
    // Clockwise spin
    turnRight(SPEEDS[3], SPEEDS[3]);
  } else {
    // Counter-clockwise spin
    turnLeft(SPEEDS[3], SPEEDS[3]);
  }
}

// Attack opponent based on sensor readings
void attackOpponent() {
  // If both bump sensors are triggered, go full power
  if ((bumpSensorState & ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) == 
      ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) {
    moveForward(SPEEDS[4], SPEEDS[4]); // Maximum speed
  }
  // If left bump sensor is triggered, push harder on left side
  else if (bumpSensorState & (1 << BUMP_LEFT_BIT)) {
    moveForward(SPEEDS[4], SPEEDS[3]);
  }
  // If right bump sensor is triggered, push harder on right side
  else if (bumpSensorState & (1 << BUMP_RIGHT_BIT)) {
    moveForward(SPEEDS[3], SPEEDS[4]);
  }
}