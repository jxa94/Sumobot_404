#include <Arduino.h>

// BTS7960 Motor control pins
// Motor 1 (Left)
#define M1_RPWM 11  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1_LPWM 10  // Left/Backward PWM pin - connect to LPWM on BTS7960
#define M1_R_EN 9   // Right/Forward enable pin - connect to R_EN on BTS7960
#define M1_L_EN 8   // Left/Backward enable pin - connect to L_EN on BTS7960

// Motor 2 (Right)
#define M2_RPWM 6   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2_LPWM 5   // Left/Backward PWM pin - connect to LPWM on BTS7960
#define M2_R_EN 4   // Right/Forward enable pin - connect to R_EN on BTS7960
#define M2_L_EN 3   // Left/Backward enable pin - connect to L_EN on BTS7960

// Starter switch: Digital input
#define JSUMO_SWITCH 7

// Line sensors: Digital input
#define LINE_SENSOR_FL 2  // Front-left
#define LINE_SENSOR_FR 12  // Front-right
#define LINE_SENSOR_BL 13 // Back-left
#define LINE_SENSOR_BR A5 // Back-right (using analog pin as digital)

// Bump sensors
#define BUMP_LEFT 0   // Using RX pin (D0)
#define BUMP_RIGHT 1  // Using TX pin (D1)

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
const int LINE_FL_BIT = 0;
const int LINE_FR_BIT = 1;
const int LINE_BL_BIT = 2;
const int LINE_BR_BIT = 3;
const int BUMP_LEFT_BIT = 4;
const int BUMP_RIGHT_BIT = 5;

// IR sensor thresholds and constants
const int IR_MAX_DISTANCE = 150; // cm - maximum reliable distance
const int IR_MIN_DISTANCE = 20;  // cm - minimum reliable distance
const int IR_DETECTION_THRESHOLD = 80; // cm - consider opponent detected below this value

// Field boundary
const int FIELD_BOUNDARY = 1; // Line sensor reads HIGH for white boundary

// Global variables
unsigned long startTime = 0;
bool isInitialScanComplete = false;
unsigned long lastScanTime = 0;
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int lineSensorState = 0;
int bumpSensorState = 0;
int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side, 3: evading boundary

// Initial scan results
bool opponentDetectedFront = false;
bool opponentDetectedBack = false;
int opponentDirection = 0; // 0: none, 1: left, 2: center, 3: right, 4: back

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
void evadeBoundary();
void performInitialScan();
void executeStrategy();

void setup() {
  // No Serial communication during operation since RX/TX are used for sensors
  // Serial.begin(9600);  // Commented out as we're using pins 0,1 for sensors

  // Motor 1 pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);
  
  // Motor 2 pins
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);
  
  // Enable the BTS7960 drivers
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);

  // Line sensors
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BR, INPUT_PULLUP);

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
  
  // Ensure motors are stopped
  stopMovement();
}

void loop() {
  // Check if start switch is activated
  if (digitalRead(JSUMO_SWITCH)) {
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
    
    // Priority 1: Evade boundary if detected
    if (lineSensorState > 0) {
      attackMode = 3;
      evadeBoundary();
      return;
    }
    
    // Execute strategy based on sensors and initial scan
    executeStrategy();
    
  } else {
    // Robot is off
    stopMovement();
    isInitialScanComplete = false;
    startTime = 0;
  }
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

// Movement Functions for BTS7960
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor forward
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  // Right motor forward
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Left motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  // Right motor backward
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  // Left motor backward, Right motor forward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  // Left motor forward, Right motor backward
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
  // Line sensors (HIGH on white boundary)
  lineSensorState = (digitalRead(LINE_SENSOR_FL) << LINE_FL_BIT) |
                    (digitalRead(LINE_SENSOR_FR) << LINE_FR_BIT) |
                    (digitalRead(LINE_SENSOR_BL) << LINE_BL_BIT) |
                    (digitalRead(LINE_SENSOR_BR) << LINE_BR_BIT);
  
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

// Handle boundary detection and evasion - Non-blocking version
void evadeBoundary() {
  static unsigned long boundaryDetectedTime = 0;
  static int evasionState = 0;
  
  // State machine for boundary evasion without delays
  if (evasionState == 0) {
    // Initial detection - record time and start backing away
    boundaryDetectedTime = millis();
    evasionState = 1;
  }
  
  // Determine which sensors detected the boundary and back away accordingly
  if (evasionState == 1) {
    // Front sensors detected boundary
    if ((lineSensorState & ((1 << LINE_FL_BIT) | (1 << LINE_FR_BIT))) > 0) {
      moveBackward(SPEEDS[4], SPEEDS[4]);
    }
    // Back sensors detected boundary
    else if ((lineSensorState & ((1 << LINE_BL_BIT) | (1 << LINE_BR_BIT))) > 0) {
      moveForward(SPEEDS[4], SPEEDS[4]);
    }
    
    // After backing up for a short time, turn
    if (millis() - boundaryDetectedTime > 300) {
      evasionState = 2;
      boundaryDetectedTime = millis();
    }
  }
  
  // Turn away from the boundary
  if (evasionState == 2) {
    if (lineSensorState & ((1 << LINE_FL_BIT) | (1 << LINE_BL_BIT))) {
      turnRight(SPEEDS[4], SPEEDS[4]);
    } else {
      turnLeft(SPEEDS[4], SPEEDS[4]);
    }
    
    // After turning for a short time, go back to normal
    if (millis() - boundaryDetectedTime > 200) {
      evasionState = 0;
      
      // Reset opponent direction based on sensors
      int leftDist = getIRDistance(IR_REFLECT_LEFT);
      int centerDist = getIRDistance(IR_REFLECT_CENTER);
      int rightDist = getIRDistance(IR_REFLECT_RIGHT);
      
      if (centerDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 2;
      } else if (leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 1;
      } else if (rightDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 3;
      } else if (getUltrasonicDistance() < 30) {
        opponentDirection = 4;
      } else {
        opponentDirection = 0;
      }
    }
  }
}