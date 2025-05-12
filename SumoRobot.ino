#include <Arduino.h>
#include "Basic.h"

// Starter switch: Digital input
#define JSUMO_SWITCH 7


// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2


// Constants
const int SPEED_SLOW = 25;    // Slow search speed
const int SPEED_SEARCH = 15;  // Very slow rotation speed for searching
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
const int NUM_IR_READINGS = 5;         // Number of readings to average for IR sensors
const int MIN_CONSECUTIVE_DETECTIONS = 3; // Minimum consecutive detections to confirm opponent

// Global variables

// Define robot states
enum RobotState {
  SEARCHING,
  ADJUSTING,
  ATTACKING
};
RobotState currentState = SEARCHING; // Initial state
unsigned long startTime = 0;
bool isInitialScanComplete = false;
unsigned long lastScanTime = 0;
unsigned long lastAdjustmentTime = 0;    // For continuous direction adjustment
int scanDirection = 1; // 1 for clockwise, -1 for counter-clockwise
int bumpSensorState = 0;
// int attackMode = 0; // 0: searching, 1: attacking front, 2: attacking from side // Replaced by currentState
int consecutiveDetections = 0; // Counter for consecutive opponent detections

// Continuous scanning variables
int lastLeftDist = IR_MAX_DISTANCE;
int lastCenterDist = IR_MAX_DISTANCE;
int lastRightDist = IR_MAX_DISTANCE;
long lastBackDist = 100;

int leftDist = 0;
int centerDist = 0;
int rightDist = 0;
long backDist = 0;

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
void searchOpponent();
void attackOpponent(int leftDist, int centerDist, int rightDist);
void performInitialScan();
void executeStrategy();
void handleJsumoSwitch();
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist);

unsigned long lastSearchActionTime = 0;
int searchPhase = 0; // 0: initial rotation, 1: pause, 2: sweep
const unsigned long SEARCH_PHASE_DURATION = 800; // ms for each search phase - 增加持续时间使旋转更稳定

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
  pinMode(BUMP_LEFT, INPUT_PULLUP); // Use internal pull-up resistors
  pinMode(BUMP_RIGHT, INPUT_PULLUP); // Use internal pull-up resistors

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
    
    // Update sensor readings before executing strategy
    leftDist = getIRDistance(IR_REFLECT_LEFT);
    centerDist = getIRDistance(IR_REFLECT_CENTER);
    rightDist = getIRDistance(IR_REFLECT_RIGHT);
    backDist = getUltrasonicDistance();

    // Display IR sensor data for debugging
    Serial.println(F("-- IR Sensors --"));
    Serial.print(F("\tLeft Distance=")); Serial.print(leftDist); Serial.println(F(" cm"));
    Serial.print(F("\tCenter Distance=")); Serial.print(centerDist); Serial.println(F(" cm"));
    Serial.print(F("\tRight Distance=")); Serial.print(rightDist); Serial.println(F(" cm"));

    // Display ultrasonic sensor data for debugging
    Serial.println(F("\n-- Ultrasonic Sensor --"));
    Serial.print(F("Back Distance: "));
    if (backDist < 0) {
      Serial.println(F("No signal received"));
    } else {
      Serial.print(backDist);
      Serial.println(F(" cm"));
    }

    // Execute strategy based on sensors and continuous scanning
    executeStrategy();
  } else {
    // Robot is inactive, ensure motors are stopped
    stopMovement();
    isInitialScanComplete = false;
    startTime = 0;
  }
  // Sensor readings and Serial prints moved up
  delay(1000); // Keep delay if it's for overall loop timing, otherwise consider moving or removing
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
// executeStrategy: Main decision-making function for robot behavior.
void executeStrategy() {
  // Sensor data (leftDist, centerDist, rightDist, backDist, bumpSensorState) is updated in loop() before this.

  // 1. Update Opponent Detection Status (critical for state transitions)
  bool basicClearDetection = (centerDist < IR_DETECTION_THRESHOLD || 
                              leftDist < IR_DETECTION_THRESHOLD || 
                              rightDist < IR_DETECTION_THRESHOLD);

  if (basicClearDetection) {
    if (!isOpponentVisible) {
        consecutiveDetections = 0; // Reset if just became visible
    }
    consecutiveDetections++;
    lastOpponentDetection = millis();
    isOpponentVisible = true;
  } else {
    consecutiveDetections = 0; // Reset if no basic detection
    if (isOpponentVisible && (millis() - lastOpponentDetection > 1500)) { // Timeout for losing sight
      isOpponentVisible = false;
      preferredDirection = 0; 
      // If opponent is lost, the current state's logic will handle transition, typically to SEARCHING.
    }
  }
  bool confirmedClearDetection = (isOpponentVisible && consecutiveDetections >= MIN_CONSECUTIVE_DETECTIONS);

  // 2. Handle Bump Sensor Activation (Highest Priority for state transition)
  if (bumpSensorState > 0) {
    if (currentState != ATTACKING) { // Print only on state change
      Serial.println(F("ATTACKING: Bump sensor triggered!"));
    }
    currentState = ATTACKING; // Force ATTACKING state on bump
    // attackOpponent will be called within the ATTACKING state's logic
  }

  // 3. State Machine Logic
  switch (currentState) {
    case SEARCHING:
      Serial.println(F("State: SEARCHING"));
      if (confirmedClearDetection) {
        currentState = ADJUSTING; // Opponent found, switch to adjusting. adjustDirectionToOpponent will be called in ADJUSTING.
      } else {
        searchOpponent(); // Perform search action
      }
      break;

    case ADJUSTING:
      Serial.println(F("State: ADJUSTING"));
      if (!isOpponentVisible) { // Lost opponent completely
          currentState = SEARCHING; // Transition to SEARCHING. searchOpponent will be called there.
          break; 
      }
      
      if (confirmedClearDetection) {
        adjustDirectionToOpponent(leftDist, centerDist, rightDist, backDist); // Perform adjustment
        // Check if ready to attack: opponent is centered (preferredDirection == 0 from adjust) AND close
        if (preferredDirection == 0 && centerDist < IR_DETECTION_THRESHOLD) { 
          if (currentState != ATTACKING) { // Print only on state change
            Serial.print(F("ATTACKING: Opponent centered and close. Center_IR_Dist: "));
            Serial.println(centerDist);
          }
          currentState = ATTACKING; // Transition to ATTACKING. attackOpponent will be called there.
        }
        // If still needs adjustment, stay in ADJUSTING state. adjustDirectionToOpponent handles the action.
      } else {
        // Opponent not confirmed (e.g., lost sight briefly, or not enough consecutive detections)
        currentState = SEARCHING; // Go back to searching
      }
      break;

    case ATTACKING:
      Serial.println(F("State: ATTACKING"));
      // If opponent is lost (and not bumped), transition to SEARCHING
      if (!isOpponentVisible && bumpSensorState == 0) { 
          currentState = SEARCHING; // Transition to SEARCHING. searchOpponent will be called there.
          break;
      }

      // If bumped, or if opponent is confirmed, centered, and close, then attack.
      if (bumpSensorState > 0 || (confirmedClearDetection && preferredDirection == 0 && centerDist < IR_DETECTION_THRESHOLD)) {
        attackOpponent(leftDist, centerDist, rightDist); // Perform attack action
        
        // If bump is over AND conditions for attack are no longer met, transition to ADJUSTING.
        // Otherwise, stay in ATTACKING (e.g. continuous bump, or still centered and close).
        if (bumpSensorState == 0 && !(confirmedClearDetection && preferredDirection == 0 && centerDist < IR_DETECTION_THRESHOLD)) {
            currentState = ADJUSTING; // Need to re-evaluate/re-adjust
        }
      } else {
        // Conditions for attack not met (e.g., opponent moved, not centered, not confirmed)
        currentState = ADJUSTING; // Go back to adjusting
      }
      break;
  }
}

// Adjust direction to keep facing the opponent - more conservative approach
// Adjust direction to keep facing the opponent - ONLY rotation
void adjustDirectionToOpponent(int leftDist, int centerDist, int rightDist, long backDist) {
  // Check if we need to perform a major direction adjustment (opponent is behind)
  if (backDist > 0 && backDist < 30 && 
      (leftDist >= IR_DETECTION_THRESHOLD && 
       centerDist >= IR_DETECTION_THRESHOLD && 
       rightDist >= IR_DETECTION_THRESHOLD)) {
    // Opponent is behind us, turn around quickly
    if (scanDirection == 1) { // Current scan direction might influence initial turn
      turnRight(SPEED_FAST, SPEED_FAST); 
    } else {
      turnLeft(SPEED_FAST, SPEED_FAST);
    }
    consecutiveDetections = 0; // Reset detection count after a major turn
    preferredDirection = (scanDirection == 1) ? 1 : -1; // Set preferred direction based on turn
    return;
  }
  
  // Calculate difference percentages to implement tolerance for front IR sensors
  int leftRightDiffPercent = 0;
  if (leftDist < IR_DETECTION_THRESHOLD && rightDist < IR_DETECTION_THRESHOLD) {
    if (leftDist <= rightDist && rightDist > 0) { // Avoid division by zero
      leftRightDiffPercent = ((rightDist - leftDist) * 100) / rightDist;
    } else if (leftDist > 0) { // Avoid division by zero
      leftRightDiffPercent = -((leftDist - rightDist) * 100) / leftDist;
    }
  }
  
  bool basicClearDetection = (centerDist < IR_DETECTION_THRESHOLD || 
                              leftDist < IR_DETECTION_THRESHOLD || 
                              rightDist < IR_DETECTION_THRESHOLD);
  bool confirmedClearDetection = (consecutiveDetections >= MIN_CONSECUTIVE_DETECTIONS && basicClearDetection);
                         
  if (!confirmedClearDetection) {
    // No confirmed opponent, rely on preferredDirection or search
    if (preferredDirection < 0) {
      turnLeft(SPEED_SEARCH, SPEED_SEARCH); // Slower turn if just following preferred direction
    } else if (preferredDirection > 0) {
      turnRight(SPEED_SEARCH, SPEED_SEARCH);
    } else {
      searchOpponent(); // Default to search if no preferred direction and no confirmation
    }
    return;
  }
  
  // Opponent is confirmed in front - fine-tune direction
  if (centerDist < IR_DETECTION_THRESHOLD) {
    // Opponent directly ahead or nearly ahead
    stopMovement(); // Stop turning if opponent is centered
    preferredDirection = 0;
  } 
  else if (leftDist < rightDist && leftDist < IR_DETECTION_THRESHOLD) {
    // Opponent is more to the left
    if (abs(leftRightDiffPercent) > IR_TOLERANCE) { // Only turn if difference is significant
        turnLeft(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
        stopMovement(); // Within tolerance, stop turning
    }
    preferredDirection = -1;
  } 
  else if (rightDist < leftDist && rightDist < IR_DETECTION_THRESHOLD) {
    // Opponent is more to the right
    if (abs(leftRightDiffPercent) > IR_TOLERANCE) { // Only turn if difference is significant
        turnRight(SPEED_MEDIUM, SPEED_MEDIUM);
    } else {
        stopMovement(); // Within tolerance, stop turning
    }
    preferredDirection = 1;
  }
  // If opponent detected by one side sensor but not clearly left/right (e.g. one sensor maxed out)
  // or if within tolerance but not centered, continue with preferred direction or search.
  else if (basicClearDetection) { // Some detection, but not fitting above categories
    if (preferredDirection < 0) {
      turnLeft(SPEED_SEARCH, SPEED_SEARCH);
    } else if (preferredDirection > 0) {
      turnRight(SPEED_SEARCH, SPEED_SEARCH);
    } else {
      // Fallback to a gentle turn if centered but not perfectly, or search if lost
      if (leftDist < rightDist) turnLeft(SPEED_SEARCH, SPEED_SEARCH);
      else turnRight(SPEED_SEARCH, SPEED_SEARCH);
    }
  } else {
    // Lost confirmed detection, rely on preferred direction or search
    if (preferredDirection < 0) {
      turnLeft(SPEED_SEARCH, SPEED_SEARCH);
    } else if (preferredDirection > 0) {
      turnRight(SPEED_SEARCH, SPEED_SEARCH);
    } else {
      searchOpponent();
    }
  }
}



// Improved search pattern to find opponent - ONLY rotation, no forward movement
// searchOpponent: Rotates the robot to find the opponent. ONLY rotation.
void searchOpponent() {
  Serial.println(F("enter search mode - rotating only"));
  unsigned long currentTime = millis();

  // Dynamic search strategy: cycle through rotation, pause, different rotation
  if (currentTime - lastSearchActionTime > SEARCH_PHASE_DURATION) {
    lastSearchActionTime = currentTime;
    searchPhase = (searchPhase + 1) % 3; // Cycle through 0 (rotate), 1 (pause), 2 (rotate different speed/dir)
    if (searchPhase != 1) { // Don't stop if the next phase is a rotation
        // No explicit stopMovement() here, as rotation commands will override previous ones.
        // A brief stop can be added if jerky movements are an issue.
    }
  }

  switch (searchPhase) {
    case 0: // Wider/slower rotation
      if (scanDirection == 1) {
        turnRight(SPEED_SEARCH, SPEED_SEARCH); // Use a consistent search speed
      } else {
        turnLeft(SPEED_SEARCH, SPEED_SEARCH);
      }
      break;
    case 1: // Pause and re-evaluate (sensors will be read in main loop)
      stopMovement(); 
      break;
    case 2: // Potentially different speed or alternating sweep
      // For now, same as phase 0, but could be faster or alternate direction
      if (scanDirection == 1) {
        turnRight(SPEED_SLOW, SPEED_SLOW); // Slightly faster or different pattern
      } else {
        turnLeft(SPEED_SLOW, SPEED_SLOW);
      }
      // Example of alternating sweep (could be a more complex pattern):
      // if ((currentTime / SEARCH_PHASE_DURATION) % 2 == 0) turnLeft(SPEED_SEARCH, SPEED_SEARCH);
      // else turnRight(SPEED_SEARCH, SPEED_SEARCH);
      break;
  }
  
  // Occasionally switch overall scan direction to avoid getting stuck in one rotational pattern
  // This random change should be infrequent enough not to disrupt a good search pattern too often.
  if (millis() - lastScanTime > 5000 && random(100) < 10) { // e.g., every 5 seconds, 10% chance
    scanDirection *= -1;
    lastScanTime = millis(); // Reset timer after changing direction
    searchPhase = 0; // Reset search phase to start new direction cleanly
    Serial.print(F("Search direction switched to: ")); Serial.println(scanDirection);
  }
}

// Attack opponent based on sensor readings
// attackOpponent: Assumes robot is generally facing the opponent.
// Executes forward attack moves based on proximity and confidence.
void attackOpponent(int leftDist, int centerDist, int rightDist) {
  Serial.println(F("enter attack mode"));
  bool highConfidence = (consecutiveDetections >= MIN_CONSECUTIVE_DETECTIONS);

  // 1. Handle Bump Sensor Activation (Highest Priority)
  if (bumpSensorState > 0) {
    moveForward(SPEED_MAX, SPEED_MAX); // Full speed push
    delay(200); // Push for a short duration
    // Consider adding a brief strategic reposition after bump in executeStrategy or here
    return;
  }

  // 2. Attack if Opponent is Detected Centrally and Close Enough
  // Relies on executeStrategy and adjustDirectionToOpponent to ensure the robot is mostly facing the opponent.
  if (centerDist < IR_DETECTION_THRESHOLD) {
    if (highConfidence && centerDist < IR_CLOSE_THRESHOLD) {
      // High confidence and very close: aggressive forward attack
      moveForward(SPEED_FAST, SPEED_FAST);
    } else if (centerDist < IR_CLOSE_THRESHOLD) { // Opponent is close, but confidence might be lower or not "very close"
      // Moderate forward attack/approach
      moveForward(SPEED_MEDIUM, SPEED_MEDIUM);
    } else { // Opponent detected ahead (centerDist < IR_DETECTION_THRESHOLD), but not very close (centerDist >= IR_CLOSE_THRESHOLD)
      // Cautious forward approach
      moveForward(SPEED_MEDIUM, SPEED_MEDIUM); // Or SPEED_SLOW depending on desired behavior
    }
  }
  // If opponent is not clearly in the center and close enough for an attack by the conditions above,
  // this function will do nothing. The overall strategy (decided in executeStrategy)
  // will then determine the next action (e.g., adjust direction, search).
  // Side sensor inputs (leftDist, rightDist) are not used for turning decisions here.
}