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
bool isStartDelayCompleted = false;
unsigned long lastScanTime = 0;
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int lineSensorState = 0;
int bumpSensorState = 0;
int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side, 3: evading boundary

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
void scanArea();

void setup() {
  Serial.begin(9600);

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
}

void loop() {
  // Check if start switch is activated
  if (digitalRead(JSUMO_SWITCH)) {
    // Handle 5-second delay at the start
    if (!isStartDelayCompleted) {
      if (startTime == 0) {
        startTime = millis();
      }
      
      if (millis() - startTime >= 5000) { // 5-second delay
        isStartDelayCompleted = true;
      } else {
        stopMovement(); // Stay still during delay
        return;
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
    
    // Priority 2: Attack if opponent detected by bump sensors
    if (bumpSensorState > 0) {
      attackMode = 1;
      attackOpponent();
      return;
    }
    
    // Read IR and ultrasonic sensors
    int leftDist = getIRDistance(IR_REFLECT_LEFT);
    int centerDist = getIRDistance(IR_REFLECT_CENTER);
    int rightDist = getIRDistance(IR_REFLECT_RIGHT);
    long backDist = getUltrasonicDistance();
    
    // Priority 3: Attack based on sensor readings
    if (centerDist < IR_DETECTION_THRESHOLD) {
      // Opponent is directly in front
      attackMode = 1;
      moveForward(255, 255); // Full speed ahead
    } else if (leftDist < IR_DETECTION_THRESHOLD && leftDist < rightDist) {
      // Opponent is more to the left
      attackMode = 2;
      moveForward(150, 255); // Turn left while moving forward
    } else if (rightDist < IR_DETECTION_THRESHOLD && rightDist < leftDist) {
      // Opponent is more to the right
      attackMode = 2;
      moveForward(255, 150); // Turn right while moving forward
    } else if (backDist < 30 && backDist > 0) {
      // Opponent is behind (within 30cm)
      attackMode = 2;
      // Quick 180-degree turn to face opponent
      turnRight(255, 255);
      delay(500); // Adjust based on your robot's turning speed
    } else {
      // No opponent detected, search mode
      attackMode = 0;
      searchOpponent();
    }
  } else {
    // Robot is off
    stopMovement();
    isStartDelayCompleted = false;
    startTime = 0;
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
  // Convert analog reading to distance in centimeters
  // The formula below is a rough approximation for the GP2Y0A02YK0F sensor
  // You may need to calibrate this for your actual sensor
  if (reading < 40) return IR_MAX_DISTANCE; // Too far or not reliable
  
  float distance = 9462.0 / (reading - 16.92);
  
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
  // Change scan direction every 3 seconds
  if (millis() - lastScanTime > 3000) {
    scanDirection = -scanDirection;
    lastScanTime = millis();
  }
  
  if (scanDirection == 1) {
    // Clockwise spin with forward movement
    turnRight(180, 180);
  } else {
    // Counter-clockwise spin with forward movement
    turnLeft(180, 180);
  }
}

// Attack opponent based on sensor readings
void attackOpponent() {
  // If both bump sensors are triggered, go full power
  if ((bumpSensorState & ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) == 
      ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) {
    moveForward(255, 255);
  }
  // If left bump sensor is triggered, push harder on left side
  else if (bumpSensorState & (1 << BUMP_LEFT_BIT)) {
    moveForward(255, 200);
  }
  // If right bump sensor is triggered, push harder on right side
  else if (bumpSensorState & (1 << BUMP_RIGHT_BIT)) {
    moveForward(200, 255);
  }
}

// Handle boundary detection and evasion
void evadeBoundary() {
  // Determine which sensors detected the boundary and back away accordingly
  
  // Front sensors detected boundary
  if ((lineSensorState & ((1 << LINE_FL_BIT) | (1 << LINE_FR_BIT))) > 0) {
    // Back away from the boundary
    moveBackward(255, 255);
    delay(300);
    
    // Turn away from the boundary
    if (lineSensorState & (1 << LINE_FL_BIT)) {
      turnRight(255, 255);
    } else {
      turnLeft(255, 255);
    }
    delay(200);
  }
  // Back sensors detected boundary
  else if ((lineSensorState & ((1 << LINE_BL_BIT) | (1 << LINE_BR_BIT))) > 0) {
    // Move away from the boundary
    moveForward(255, 255);
    delay(300);
    
    // Turn away from the boundary
    if (lineSensorState & (1 << LINE_BL_BIT)) {
      turnRight(255, 255);
    } else {
      turnLeft(255, 255);
    }
    delay(200);
  }
}