#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right
#define M2_RPWM 10  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2_LPWM 11  // Left/Backward PWM pin - connect to LPWM on BTS7960

// Motor left
#define M1_RPWM 5   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1_LPWM 6   // Left/Backward PWM pin - connect to LPWM on BTS7960

// Starter switch: Digital input
#define JSUMO_SWITCH 2

// Bump sensors - moved from RX/TX pins
#define BUMP_LEFT 12   // connect to 12
#define BUMP_RIGHT 4  // connect to 4

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

// Motor balancing factor - RIGHT motor will run at this percentage of LEFT motor's speed
const float RIGHT_MOTOR_FACTOR = 0.95; // 95% of left motor speed - adjust as needed

const int ONstate = 1;
const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

// IR sensor thresholds and constants
const int IR_MAX_DISTANCE = 150;         // cm - maximum reliable distance
const int IR_MIN_DISTANCE = 20;          // cm - minimum reliable distance
const int IR_DETECTION_THRESHOLD = 80;   // cm - consider opponent detected below this value
const int IR_CLOSE_THRESHOLD = 40;       // cm - opponent is very close
const int IR_TOLERANCE = 10;             // Percentage tolerance for sensor readings to avoid small adjustments

// Flag to ignore left IR sensor (set to true if sensor is faulty)
const bool IGNORE_LEFT_IR = true;        // Set to true to ignore left IR sensor

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

// JSUMO switch variables (simplified from old driver)
bool robotActive = false;
unsigned long waitStartTime = 0;
bool waitOnce = false;

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
  
  // Ensure motors are stopped
  stopMovement();
}

void loop() {
  // Read the Jsumo switch - acts as a one-time trigger
  int JSUMOdata = digitalRead(JSUMO_SWITCH);

  // If Jsumo switch is pressed and robot isn't active yet, activate it
  if (JSUMOdata && !robotActive) {
    waitStartTime = millis();
    waitOnce = true;
    robotActive = true;
    isInitialScanComplete = false;
    startTime = 0;
  }
  
  // Run robot logic if it's active
  if (robotActive) {
    // Only proceed after initial 3-second delay
    if (millis() - waitStartTime >= 3000) {
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
        } else {
          isInitialScanComplete = true;
        }
      } else {
        // Update sensor readings
        updateSensorStates();
        
        // Execute strategy based on sensors and continuous scanning
        executeStrategy();
      }
    } else {
      // During initial 3-second delay, ensure we're not moving
      stopMovement();
    }
  } else {
    // Robot not yet activated
    stopMovement();
  }
}

// Perform initial scan using only sensors (no movement)
void performInitialScan() {
  // Read IR sensors - if ignoring left IR, set to max distance
  int leftDist = IGNORE_LEFT_IR ? IR_MAX_DISTANCE : getIRDistance(IR_REFLECT_LEFT);
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
  // Note: If left IR is ignored, it won't contribute to detection
  if (centerDist < IR_DETECTION_THRESHOLD || 
      (!IGNORE_LEFT_IR && leftDist < IR_DETECTION_THRESHOLD) || 
      rightDist < IR_DETECTION_THRESHOLD ||
      (backDist > 0 && backDist < 30)) {
    
    isOpponentVisible = true;
    lastOpponentDetection = millis();
    
    // Determine preferred direction based on available sensors
    if (backDist > 0 && backDist < 30) {
      // Opponent is behind, choose a turn direction
      preferredDirection = (random(2) == 0) ? -1 : 1; // Randomly choose turn direction
    } else if (centerDist < rightDist && (!IGNORE_LEFT_IR ? (centerDist < leftDist) : true)) {
      preferredDirection = 0; // straight ahead
    } else if (!IGNORE_LEFT_IR && leftDist < rightDist) {
      preferredDirection = -1; // left
    } else {
      preferredDirection = 1; // right (default if left is ignored)
    }
  }
}

// Execute strategy based on sensors and continuous scanning
void executeStrategy() {
  // Read current sensor data - if ignoring left IR, set to max distance
  int leftDist = IGNORE_LEFT_IR ? IR_MAX_DISTANCE : getIRDistance(IR_REFLECT_LEFT);
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
  
  // Check if opponent is detected (excluding left IR if ignored)
  bool opponentDetected = (centerDist < IR_DETECTION_THRESHOLD || 
                          (!IGNORE_LEFT_IR && leftDist < IR_DETECTION_THRESHOLD) || 
                          rightDist < IR_DETECTION_THRESHOLD ||
                          (backDist > 0 && backDist < 30));
  
  if (opponentDetected) {
    isOpponentVisible = true;
    lastOpponentDetection = millis();
    attackMode = 1;
    
    // Adjust direction to face opponent
    adjustDirectionToOpponent(leftDist, centerDist, rightDist, backDist);
  } else {
    // Check if we've recently seen the opponent (within 1 second)
    if (millis() - lastOpponentDetection < 1000) {
      // Continue in the same direction briefly to try to reacquire
      if (preferredDirection < 0 && !IGNORE_LEFT_IR) {
        turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
      } else if (preferredDirection > 0) {
        turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
      } else {
        moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
      }
    } else {
      // No opponent detected for a while, go back to search mode
      isOpponentVisible = false;
      attackMode = 0;
      searchOpponent();
    }
  }
}

// Adjust direction to keep facing the opponent
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  // Check if we need to perform a major direction adjustment
  if (backDist > 0 && backDist < 30 && 
      (leftDist >= IR_DETECTION_THRESHOLD && 
       centerDist >= IR_DETECTION_THRESHOLD && 
       rightDist >= IR_DETECTION_THRESHOLD)) {
    // Opponent is behind us, turn around quickly
    if (scanDirection == 1) {
      turnRight(SPEED_FAST, SPEED_FAST);
    } else {
      turnLeft(SPEED_FAST, SPEED_FAST);
    }
    return;
  }
  
  // Calculate difference percentages to implement tolerance
  // Only if left IR is not ignored
  int leftRightDiff = 0;
  if (!IGNORE_LEFT_IR && leftDist < IR_DETECTION_THRESHOLD && rightDist < IR_DETECTION_THRESHOLD) {
    // Calculate percentage difference between left and right sensors
    if (leftDist <= rightDist) {
      leftRightDiff = ((rightDist - leftDist) * 100) / rightDist;
    } else {
      leftRightDiff = -((leftDist - rightDist) * 100) / leftDist;
    }
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
  // Only consider left sensor if not ignored
  else if (!IGNORE_LEFT_IR && leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD && leftRightDiff > IR_TOLERANCE) {
    // Opponent is to the left AND difference is significant (> tolerance) - adjust left while moving forward
    if (leftDist < IR_CLOSE_THRESHOLD) {
      // Close opponent - sharper turn
      moveForward(SPEED_SLOW, SPEED_FAST);
    } else {
      // Distant opponent - gentle turn
      moveForward(SPEED_MEDIUM, SPEED_FAST);
    }
    preferredDirection = -1;
  } 
  else if (rightDist < IR_DETECTION_THRESHOLD && (IGNORE_LEFT_IR || rightDist < leftDist) && 
           (IGNORE_LEFT_IR || leftRightDiff < -IR_TOLERANCE)) {
    // Opponent is to the right AND (left is ignored OR difference is significant) - adjust right while moving forward
    if (rightDist < IR_CLOSE_THRESHOLD) {
      // Close opponent - sharper turn
      moveForward(SPEED_FAST, SPEED_SLOW);
    } else {
      // Distant opponent - gentle turn
      moveForward(SPEED_FAST, SPEED_MEDIUM);
    }
    preferredDirection = 1;
  }
  else if ((!IGNORE_LEFT_IR && leftDist < IR_DETECTION_THRESHOLD) || rightDist < IR_DETECTION_THRESHOLD) {
    // Opponent is detected but difference is within tolerance - move forward without turning
    if (!IGNORE_LEFT_IR && min(leftDist, rightDist) < IR_CLOSE_THRESHOLD) {
      moveForward(SPEED_FAST, SPEED_FAST);
    } else if (IGNORE_LEFT_IR && rightDist < IR_CLOSE_THRESHOLD) {
      moveForward(SPEED_FAST, SPEED_FAST);
    } else {
      moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
    }
  }
  else {
    // No clear reading but we know opponent is somewhere
    // Continue in the preferred direction if we have one
    // If left IR is ignored, avoid turning left unnecessarily
    if (preferredDirection < 0 && !IGNORE_LEFT_IR) {
      turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
    } else if (preferredDirection > 0) {
      turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
      // If no preferred direction, revert to search
      searchOpponent();
    }
  }
}

// Movement Functions for BTS7960 - MODIFIED for parallel motors with RIGHT_MOTOR_FACTOR
void moveForward(int leftSpeed, int rightSpeed) {
  // Apply the RIGHT_MOTOR_FACTOR to rightSpeed
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  
  // Left motor forward (one direction)
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  // Right motor forward (opposite direction due to parallel setup) - with speed adjustment
  analogWrite(M2_LPWM, adjustedRightSpeed);
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Apply the RIGHT_MOTOR_FACTOR to rightSpeed
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  
  // Left motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  // Right motor backward (opposite direction due to parallel setup) - with speed adjustment
  analogWrite(M2_RPWM, adjustedRightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  // Apply the RIGHT_MOTOR_FACTOR to rightSpeed
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  
  // Left motor backward, Right motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  analogWrite(M2_RPWM, adjustedRightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  // Apply the RIGHT_MOTOR_FACTOR to rightSpeed
  int adjustedRightSpeed = (int)(rightSpeed * RIGHT_MOTOR_FACTOR);
  
  // Left motor forward, Right motor forward
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  analogWrite(M2_LPWM, adjustedRightSpeed);
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

// Improved search pattern to find opponent - modified to account for ignored left IR
void searchOpponent() {
  // Use oscillating pattern for searching - change direction regularly
  unsigned long currentTime = millis();
  
  // Change scan direction every 2.5 seconds to cover more area
  if (currentTime - lastScanTime > 2500) {
    // If left IR is ignored, prefer turning right (clockwise) more often
    if (IGNORE_LEFT_IR) {
      // 75% chance to turn right when left IR is ignored
      scanDirection = (random(4) < 3) ? 1 : -1;
    } else {
      // Normal behavior - alternate direction
      scanDirection = -scanDirection;
    }
    lastScanTime = currentTime;
    
    // Every other direction change, move forward a bit to explore
    static bool moveForwardFlag = false;
    if (moveForwardFlag) {
      // Move forward briefly to explore different areas of the ring
      moveForward(SPEED_SLOW, SPEED_SLOW);
      delay(500); // Short forward movement
    }
    moveForwardFlag = !moveForwardFlag;
  }
  
  // Implement a combination of turning and moving forward for more area coverage
  if (currentTime % 5000 < 3500) {
    // Spin to scan
    if (scanDirection == 1) {
      // Clockwise spin
      turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
      // Only turn counter-clockwise if left IR is not ignored
      if (!IGNORE_LEFT_IR) {
        turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
      } else {
        // Default to right turn when left IR is ignored
        turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
      }
    }
  } else {
    // Move forward in arc (slight turn while moving)
    if (scanDirection == 1 || IGNORE_LEFT_IR) {
      // Bias toward right turns if left IR is ignored
      moveForward(SPEED_MEDIUM, SPEED_SLOW);
    } else {
      moveForward(SPEED_SLOW, SPEED_MEDIUM);
    }
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