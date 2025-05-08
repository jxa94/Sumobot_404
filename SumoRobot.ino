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
#define BUMP_LEFT 8   // connect to 8
#define BUMP_RIGHT 9  // connect to 9

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int SPEED_SLOW = 40;    // Slow search speed
const int SPEED_MEDIUM = 80; // Medium attack speed
const int SPEED_FAST = 120;   // Fast attack speed
const int SPEED_MAX = 200;    // Maximum speed for direct contact

const int ONstate = 1;
const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

// IR sensor thresholds and constants
const int IR_MAX_DISTANCE = 150;         // cm - maximum reliable distance
const int IR_MIN_DISTANCE = 20;          // cm - minimum reliable distance
const int IR_DETECTION_THRESHOLD = 80;   // cm - consider opponent detected below this value
const int IR_CLOSE_THRESHOLD = 40;       // cm - opponent is very close
const int IR_TOLERANCE = 10;             // Percentage tolerance for sensor readings to avoid small adjustments

// Global variables
unsigned long startTime = 0;
bool isInitialScanComplete = false;
unsigned long lastScanTime = 0;
unsigned long lastAdjustmentTime = 0;    // For continuous direction adjustment
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int bumpSensorState = 0;
int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side

// Continuous scanning variables
int lastLeftDist = IR_MAX_DISTANCE;
int lastCenterDist = IR_MAX_DISTANCE;
int lastRightDist = IR_MAX_DISTANCE;
long lastBackDist = 100;

// Direction and tracking
int preferredDirection = 0;  // For maintaining pursuit direction
unsigned long lastOpponentDetection = 0;
bool isOpponentVisible = false;

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
void attackOpponent(int leftDist, int centerDist, int rightDist);
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch();
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist);

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
        isOpponentVisible = false;
        preferredDirection = 0;
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
    
    // Execute strategy based on sensors and continuous scanning
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
  
  // Update last distances
  lastLeftDist = leftDist;
  lastCenterDist = centerDist;
  lastRightDist = rightDist;
  lastBackDist = backDist;
  
  // Check if opponent is detected in any direction
  if (centerDist < IR_DETECTION_THRESHOLD || 
      leftDist < IR_DETECTION_THRESHOLD || 
      rightDist < IR_DETECTION_THRESHOLD ||
      (backDist > 0 && backDist < 30)) {
    
    isOpponentVisible = true;
    lastOpponentDetection = millis();
    
    // Determine preferred direction
    if (backDist > 0 && backDist < 30) {
      // Opponent is behind, choose a turn direction
      preferredDirection = (random(2) == 0) ? -1 : 1; // Randomly choose turn direction
    } else if (centerDist < leftDist && centerDist < rightDist) {
      preferredDirection = 0; // straight ahead
    } else if (leftDist < rightDist) {
      preferredDirection = -1; // left
    } else {
      preferredDirection = 1; // right
    }
  }
}

// Execute strategy based on sensors and continuous scanning
void executeStrategy() {
  // Read current sensor data
  int leftDist = getIRDistance(IR_REFLECT_LEFT);
  int centerDist = getIRDistance(IR_REFLECT_CENTER);
  int rightDist = getIRDistance(IR_REFLECT_RIGHT);
  long backDist = getUltrasonicDistance();
  
  // Update last distances for tracking
  lastLeftDist = leftDist;
  lastCenterDist = centerDist;
  lastRightDist = rightDist;
  lastBackDist = backDist;
  
  // First check bump sensors as they indicate direct contact
  if (bumpSensorState > 0) {
    attackMode = 1;
    attackOpponent(leftDist, centerDist, rightDist);
    return;
  }
  
  // Check if opponent is detected - use more conservative thresholds
  // to ensure we only move when we're confident about opponent detection
  bool opponentDetected = (centerDist < IR_DETECTION_THRESHOLD || 
                          leftDist < IR_DETECTION_THRESHOLD || 
                          rightDist < IR_DETECTION_THRESHOLD ||
                          (backDist > 0 && backDist < 30));
  
  if (opponentDetected) {
    // Opponent detected - now we can move
    isOpponentVisible = true;
    lastOpponentDetection = millis();
    attackMode = 1;
    
    // Adjust direction and move toward opponent
    if (centerDist < IR_DETECTION_THRESHOLD) {
      // Opponent directly ahead - move forward
      if (centerDist < IR_CLOSE_THRESHOLD) {
        // Close opponent - higher speed
        moveForward(SPEED_FAST, SPEED_FAST);
      } else {
        // Distant opponent - moderate speed
        moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
      }
    } else if (leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD) {
      // Opponent is to the left - turn left then move
      turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
      // Only move forward if opponent is close enough
      if (leftDist < IR_CLOSE_THRESHOLD) {
        moveForward(SPEED_MEDIUM, SPEED_FAST);
      }
    } else if (rightDist < leftDist && rightDist < IR_DETECTION_THRESHOLD) {
      // Opponent is to the right - turn right then move
      turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
      // Only move forward if opponent is close enough
      if (rightDist < IR_CLOSE_THRESHOLD) {
        moveForward(SPEED_FAST, SPEED_MEDIUM);
      }
    }
  } else {
    // No opponent detected - ONLY rotate in place to search, no forward movement
    isOpponentVisible = false;
    attackMode = 0;
    searchOpponent(); // This now only performs rotation without forward movement
  }
}

// Adjust direction to keep facing the opponent - more conservative approach
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  // Check if we need to perform a major direction adjustment
  if (backDist > 0 && backDist < 30 && 
      (leftDist >= IR_DETECTION_THRESHOLD && 
       centerDist >= IR_DETECTION_THRESHOLD && 
       rightDist >= IR_DETECTION_THRESHOLD)) {
    // Opponent is behind us, turn around quickly but don't move forward
    if (scanDirection == 1) {
      turnRight(SPEED_FAST, SPEED_FAST);
    } else {
      turnLeft(SPEED_FAST, SPEED_FAST);
    }
    return;
  }
  
  // Calculate difference percentages to implement tolerance
  int leftRightDiff = 0;
  if (leftDist < IR_DETECTION_THRESHOLD && rightDist < IR_DETECTION_THRESHOLD) {
    // Calculate percentage difference between left and right sensors
    if (leftDist <= rightDist) {
      leftRightDiff = ((rightDist - leftDist) * 100) / rightDist;
    } else {
      leftRightDiff = -((leftDist - rightDist) * 100) / leftDist;
    }
  }
  
  // Only move if we have a clear detection of the opponent
  bool clearDetection = (centerDist < IR_DETECTION_THRESHOLD || 
                         leftDist < IR_DETECTION_THRESHOLD || 
                         rightDist < IR_DETECTION_THRESHOLD);
                         
  if (!clearDetection) {
    // No clear detection, just rotate in place to search
    searchOpponent();
    return;
  }
  
  // Opponent is in front - fine-tune direction
  if (centerDist < IR_DETECTION_THRESHOLD) {
    // Opponent directly ahead - move forward
    if (centerDist < IR_CLOSE_THRESHOLD) {
      // Close opponent - higher speed
      moveForward(SPEED_FAST, SPEED_FAST);
    } else {
      // Distant opponent - moderate speed
      moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
    }
    preferredDirection = 0;
  } 
  else if (leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD && leftRightDiff > IR_TOLERANCE) {
    // Opponent is to the left AND difference is significant (> tolerance)
    if (leftDist < IR_CLOSE_THRESHOLD) {
      // Close opponent - turn and move
      moveForward(SPEED_SLOW, SPEED_FAST);
    } else {
      // Distant opponent - just turn, don't move forward yet
      turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
    }
    preferredDirection = -1;
  } 
  else if (rightDist < leftDist && rightDist < IR_DETECTION_THRESHOLD && leftRightDiff < -IR_TOLERANCE) {
    // Opponent is to the right AND difference is significant (> tolerance)
    if (rightDist < IR_CLOSE_THRESHOLD) {
      // Close opponent - turn and move
      moveForward(SPEED_FAST, SPEED_SLOW);
    } else {
      // Distant opponent - just turn, don't move forward yet
      turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
    }
    preferredDirection = 1;
  }
  else if (leftDist < IR_DETECTION_THRESHOLD || rightDist < IR_DETECTION_THRESHOLD) {
    // Opponent is detected but difference is within tolerance
    if (min(leftDist, rightDist) < IR_CLOSE_THRESHOLD) {
      // Only move forward if opponent is close
      moveForward(SPEED_FAST, SPEED_FAST);
    } else {
      // Otherwise just turn to face opponent better
      if (leftDist < rightDist) {
        turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
      } else {
        turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
      }
    }
  }
  else {
    // No clear reading but we have a preferred direction
    if (preferredDirection < 0) {
      turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
    } else if (preferredDirection > 0) {
      turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
      // If no preferred direction, revert to search
      searchOpponent();
    }
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

// Improved search pattern to find opponent - ONLY rotation, no forward movement
void searchOpponent() {
  // Use oscillating pattern for searching - change direction regularly
  unsigned long currentTime = millis();
  
  // Change scan direction every 3 seconds to cover more area
  if (currentTime - lastScanTime > 3000) {
    scanDirection = -scanDirection;
    lastScanTime = currentTime;
    
    // No forward movement during search to avoid falling off edge
  }
  
  // Only perform in-place rotation for searching
  // This prevents the robot from moving blindly and falling off the edge
  if (scanDirection == 1) {
    // Clockwise spin
    turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
  } else {
    // Counter-clockwise spin
    turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
  }
}

// Attack opponent based on sensor readings
void attackOpponent(int leftDist, int centerDist, int rightDist) {
  // If both bump sensors are triggered, go full power
  if ((bumpSensorState & ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) == 
      ((1 << BUMP_LEFT_BIT) | (1 << BUMP_RIGHT_BIT))) {
    moveForward(SPEED_MAX, SPEED_MAX); // Maximum speed
  }
  // If left bump sensor is triggered, push harder on left side
  else if (bumpSensorState & (1 << BUMP_LEFT_BIT)) {
    moveForward(SPEED_MAX, SPEED_FAST);
  }
  // If right bump sensor is triggered, push harder on right side
  else if (bumpSensorState & (1 << BUMP_RIGHT_BIT)) {
    moveForward(SPEED_FAST, SPEED_MAX);
  }
}