#include <Arduino.h>

// Motor pins: Digital output
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

// Starter switch: Digital input
#define JSUMO_SWITCH 4

// Line sensors: Digital input
#define LINE_SENSOR_FL 2  // Front-left
#define LINE_SENSOR_FR 3  // Front-right
#define LINE_SENSOR_BL 12 // Back-left
#define LINE_SENSOR_BR 13 // Back-right

// Bump sensors
#define BUMP_LEFT 7
#define BUMP_RIGHT 8

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int SPEEDS[] = {50, 100, 150, 200, 240}; 
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
unsigned long lastDebugTime = 0;  // For periodic debug output
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int lineSensorState = 0;
int bumpSensorState = 0;
int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side, 3: evading boundary
String currentState = "OFF"; // Current robot state for debugging

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
void printDebugInfo();

void setup() {
  Serial.begin(9600);
  Serial.println("Sumo Robot Initialized");

  // Motor pins
  pinMode(M2R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);

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
  
  Serial.println("All pins initialized.");
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
        Serial.println("Starting initial 3-second scan...");
      }
      
      // During the 3-second period, only perform scanning
      if (millis() - startTime < 3000) {
        stopMovement(); // Ensure we're not moving
        currentState = "INITIAL_SCAN";
        performInitialScan(); // Only scan, don't move
        
        // Print debug info every 500ms during scan
        if (millis() - lastDebugTime > 500) {
          printDebugInfo();
          lastDebugTime = millis();
        }
        return;
      } else {
        isInitialScanComplete = true;
        Serial.println("Initial scan complete. Opponent direction: " + String(opponentDirection));
        Serial.println("Opponent front: " + String(opponentDetectedFront) + ", Opponent back: " + String(opponentDetectedBack));
      }
    }
    
    // Update sensor readings
    updateSensorStates();
    
    // Print debug info every 500ms
    if (millis() - lastDebugTime > 500) {
      printDebugInfo();
      lastDebugTime = millis();
    }
    
    // Priority 1: Evade boundary if detected
    if (lineSensorState > 0) {
      attackMode = 3;
      currentState = "EVADING_BOUNDARY";
      Serial.println("BOUNDARY DETECTED! Line sensor state: " + String(lineSensorState, BIN));
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
    currentState = "OFF";
    
    // Only print OFF state once when changing to it
    static bool wasOn = false;
    if (wasOn) {
      Serial.println("Robot OFF");
      wasOn = false;
    }
    return;
  }
  
  // Set wasOn to true when robot is on
  static bool wasOn = false;
  wasOn = true;
}

// Print debug information to serial monitor
void printDebugInfo() {
  Serial.println("\n----- DEBUG INFO -----");
  Serial.println("Current state: " + currentState);
  Serial.println("Attack mode: " + String(attackMode));
  
  // Sensor readings
  int leftDist = getIRDistance(IR_REFLECT_LEFT);
  int centerDist = getIRDistance(IR_REFLECT_CENTER);
  int rightDist = getIRDistance(IR_REFLECT_RIGHT);
  long backDist = getUltrasonicDistance();
  
  Serial.println("IR Left: " + String(leftDist) + " cm");
  Serial.println("IR Center: " + String(centerDist) + " cm");
  Serial.println("IR Right: " + String(rightDist) + " cm");
  Serial.println("Ultrasonic: " + String(backDist) + " cm");
  
  // Line sensors
  Serial.print("Line sensors (FLBR): ");
  Serial.print(digitalRead(LINE_SENSOR_FL));
  Serial.print(digitalRead(LINE_SENSOR_FR));
  Serial.print(digitalRead(LINE_SENSOR_BL));
  Serial.println(digitalRead(LINE_SENSOR_BR));
  
  // Bump sensors
  Serial.print("Bump sensors (L/R): ");
  Serial.print(digitalRead(BUMP_LEFT));
  Serial.println(digitalRead(BUMP_RIGHT));
  
  // Robot strategy info
  Serial.println("Opponent direction: " + String(opponentDirection));
  Serial.println("Scan direction: " + String(scanDirection));
  Serial.println("Runtime: " + String((millis() - startTime) / 1000) + " seconds");
  Serial.println("---------------------\n");
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
      Serial.println("Initial scan: Opponent detected in CENTER");
    } else if (leftDist < rightDist) {
      opponentDirection = 1; // left
      Serial.println("Initial scan: Opponent detected on LEFT");
    } else {
      opponentDirection = 3; // right
      Serial.println("Initial scan: Opponent detected on RIGHT");
    }
  }
  
  // Check back direction (ultrasonic)
  if (backDist > 0 && backDist < 30) {
    opponentDetectedBack = true;
    opponentDirection = 4; // back
    Serial.println("Initial scan: Opponent detected at BACK");
  }
}

// Execute strategy based on sensors and initial scan
void executeStrategy() {
  // First check bump sensors as they indicate direct contact
  if (bumpSensorState > 0) {
    attackMode = 1;
    currentState = "ATTACKING_CONTACT";
    Serial.println("CONTACT DETECTED! Pushing at max speed");
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
    currentState = "ATTACKING_FRONT";
    moveForward(SPEEDS[3], SPEEDS[3]); // Medium speed ahead
    return;
  } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
    attackMode = 2;
    currentState = "ATTACKING_LEFT";
    moveForward(SPEEDS[2], SPEEDS[3]); // Turn left while moving forward
    return;
  } else if (rightDist < IR_DETECTION_THRESHOLD && rightDist < leftDist) {
    attackMode = 2;
    currentState = "ATTACKING_RIGHT";
    moveForward(SPEEDS[3], SPEEDS[2]); // Turn right while moving forward
    return;
  } else if (backDist < 30 && backDist > 0) {
    // Opponent is behind
    attackMode = 2;
    currentState = "TURNING_TO_BACK";
    Serial.println("Opponent detected behind robot, turning to face");
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
        currentState = "MOVING_TO_LEFT";
        Serial.println("Moving toward left based on initial scan");
        turnLeft(SPEEDS[3], SPEEDS[3]);
        break;
      case 2: // center
        currentState = "MOVING_FORWARD";
        Serial.println("Moving forward based on initial scan");
        moveForward(SPEEDS[3], SPEEDS[3]); // Medium speed
        break;
      case 3: // right
        currentState = "MOVING_TO_RIGHT";
        Serial.println("Moving toward right based on initial scan");
        turnRight(SPEEDS[3], SPEEDS[3]);
        break;
      case 4: // back
        currentState = "TURNING_AROUND";
        Serial.println("Turning around based on initial scan");
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
    currentState = "SEARCHING";
    searchOpponent();
  }
}

// Movement Functions
void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2L, LOW);
}

void turnRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);
}

void stopMovement() {
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
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
    Serial.println("Changing scan direction to: " + String(scanDirection == 1 ? "clockwise" : "counter-clockwise"));
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
    Serial.println("Both bumpers activated! Full power push");
    moveForward(SPEEDS[4], SPEEDS[4]); // Maximum speed
  }
  // If left bump sensor is triggered, push harder on left side
  else if (bumpSensorState & (1 << BUMP_LEFT_BIT)) {
    Serial.println("Left bumper activated! Pushing with left emphasis");
    moveForward(SPEEDS[4], SPEEDS[3]);
  }
  // If right bump sensor is triggered, push harder on right side
  else if (bumpSensorState & (1 << BUMP_RIGHT_BIT)) {
    Serial.println("Right bumper activated! Pushing with right emphasis");
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
    Serial.println("Boundary evasion: Starting to back away");
  }
  
  // Determine which sensors detected the boundary and back away accordingly
  if (evasionState == 1) {
    // Front sensors detected boundary
    if ((lineSensorState & ((1 << LINE_FL_BIT) | (1 << LINE_FR_BIT))) > 0) {
      Serial.println("Boundary detected at front, backing up");
      moveBackward(SPEEDS[4], SPEEDS[4]);
    }
    // Back sensors detected boundary
    else if ((lineSensorState & ((1 << LINE_BL_BIT) | (1 << LINE_BR_BIT))) > 0) {
      Serial.println("Boundary detected at back, moving forward");
      moveForward(SPEEDS[4], SPEEDS[4]);
    }
    
    // After backing up for a short time, turn
    if (millis() - boundaryDetectedTime > 300) {
      evasionState = 2;
      boundaryDetectedTime = millis();
      Serial.println("Boundary evasion: Changing direction");
    }
  }
  
  // Turn away from the boundary
  if (evasionState == 2) {
    if (lineSensorState & ((1 << LINE_FL_BIT) | (1 << LINE_BL_BIT))) {
      Serial.println("Turning right to avoid boundary");
      turnRight(SPEEDS[4], SPEEDS[4]);
    } else {
      Serial.println("Turning left to avoid boundary");
      turnLeft(SPEEDS[4], SPEEDS[4]);
    }
    
    // After turning for a short time, go back to normal
    if (millis() - boundaryDetectedTime > 200) {
      evasionState = 0;
      Serial.println("Boundary evasion complete");
      
      // Reset opponent direction based on sensors
      int leftDist = getIRDistance(IR_REFLECT_LEFT);
      int centerDist = getIRDistance(IR_REFLECT_CENTER);
      int rightDist = getIRDistance(IR_REFLECT_RIGHT);
      
      if (centerDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 2;
        Serial.println("Opponent now detected in center");
      } else if (leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 1;
        Serial.println("Opponent now detected on left");
      } else if (rightDist < IR_DETECTION_THRESHOLD) {
        opponentDirection = 3;
        Serial.println("Opponent now detected on right");
      } else if (getUltrasonicDistance() < 30) {
        opponentDirection = 4;
        Serial.println("Opponent now detected at back");
      } else {
        opponentDirection = 0;
        Serial.println("No opponent detected after boundary evasion");
      }
    }
  }
}