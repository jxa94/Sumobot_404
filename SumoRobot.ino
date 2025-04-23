#include <Arduino.h>
#include <SPI.h>

// === STRATEGY CONTROL ===
#define STRATEGY_DEFAULT 0
#define STRATEGY_RUNAWAY 1
#define STRATEGY_SELECT_PIN A5  // Pin to select strategy

// Motor pins
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

// Input pins
#define JSUMO_SWITCH 4
#define LINE_SENSOR_FL 2
#define LINE_SENSOR_FR 3
#define LINE_SENSOR_BL 12
#define LINE_SENSOR_BR 13
#define BUMP_LEFT 7
#define BUMP_RIGHT 8
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// === State Definitions ===
#define STATE_IDLE     0
#define STATE_SEEK     1
#define STATE_ATTACK   2
#define STATE_RETREAT  3
#define STATE_LINE_RECOVERY 4

// Global variables
uint8_t currentState = STATE_IDLE;
uint8_t currentStrategy = STRATEGY_DEFAULT;

// Sensor flags
bool frontDetected = false;
bool leftDetected = false;
bool rightDetected = false;
bool lineFL = false;
bool lineFR = false;
bool lineBL = false;
bool lineBR = false;

// Timing variables
unsigned long lastStateTime = 0;
unsigned long startWaitTime = 0;
bool waitingForStart = false;

// Movement functions
void moveForward(uint8_t leftSpeed, uint8_t rightSpeed) {
  digitalWrite(M1L, HIGH);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M2R, HIGH);
  
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
}

void moveBackward(uint8_t leftSpeed, uint8_t rightSpeed) {
  digitalWrite(M1L, LOW);
  digitalWrite(M1R, HIGH);
  digitalWrite(M2L, HIGH);
  digitalWrite(M2R, LOW);
  
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
}

void turnLeft(uint8_t leftSpeed, uint8_t rightSpeed) {
  digitalWrite(M1L, LOW);
  digitalWrite(M1R, HIGH);
  digitalWrite(M2L, LOW);
  digitalWrite(M2R, HIGH);
  
  analogWrite(M1R, leftSpeed);
  analogWrite(M2R, rightSpeed);
}

void turnRight(uint8_t leftSpeed, uint8_t rightSpeed) {
  digitalWrite(M1L, HIGH);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, HIGH);
  digitalWrite(M2R, LOW);
  
  analogWrite(M1L, leftSpeed);
  analogWrite(M2L, rightSpeed);
}

void stopMovement() {
  digitalWrite(M1L, LOW);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M2R, LOW);
}

// Simplified sensor state update 
void updateSensorStates() {
  // Read bump and ultrasonic sensors
  rightDetected = digitalRead(BUMP_RIGHT);
  leftDetected = digitalRead(BUMP_LEFT);
  frontDetected = digitalRead(ULTRASONIC_ECHO);
  
  // Line sensors (inverted logic - LOW when line detected)
  lineFL = !digitalRead(LINE_SENSOR_FL);
  lineFR = !digitalRead(LINE_SENSOR_FR);
  lineBL = !digitalRead(LINE_SENSOR_BL);
  lineBR = !digitalRead(LINE_SENSOR_BR);
}

// Checks if any line sensor is active
bool lineDetected() {
  return lineFL || lineFR || lineBL || lineBR;
}

// Ultrasonic trigger function
void triggerUltrasonic() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
}

void setup() {
  // Set pin modes using standard Arduino functions
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M2R, OUTPUT);
  
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BR, INPUT_PULLUP);
  
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);
  
  pinMode(JSUMO_SWITCH, INPUT);
  pinMode(STRATEGY_SELECT_PIN, INPUT_PULLUP);
  
  // Set PWM frequencies for motor control (closest equivalent to the direct register setup)
  // This uses standard Arduino functions instead of direct register manipulation
  
  // Initialize
  updateSensorStates();
  lastStateTime = micros();
}

// Execute Default Strategy (aggressive)
void executeDefaultStrategy() {
  switch (currentState) {
    case STATE_IDLE:
      stopMovement();
      // Check start switch
      if (digitalRead(JSUMO_SWITCH)) {
        if (!waitingForStart) {
          startWaitTime = micros();
          waitingForStart = true;
        }
        
        // 3 second wait
        if (micros() - startWaitTime >= 3000000UL) {
          currentState = STATE_SEEK;
          lastStateTime = micros();
        }
      } else {
        waitingForStart = false;
      }
      break;
      
    case STATE_SEEK:
      // Find opponent - spin
      turnRight(150, 150);
      
      // Check for opponent or line
      if (frontDetected || leftDetected || rightDetected) {
        currentState = STATE_ATTACK;
        lastStateTime = micros();
      }
      
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY;
        lastStateTime = micros();
      }
      break;
      
    case STATE_ATTACK:
      // Execute attack based on sensor inputs
      if (frontDetected) {
        moveForward(255, 255);  // Full attack
      } 
      else if (leftDetected) {
        moveForward(150, 255);  // Turn left
      }
      else if (rightDetected) {
        moveForward(255, 150);  // Turn right
      }
      else {
        currentState = STATE_SEEK;
        lastStateTime = micros();
      }
      
      // Check for line
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY;
        lastStateTime = micros();
      }
      break;
      
    case STATE_LINE_RECOVERY:
      // Handle line detection
      if (lineFL || lineFR) {
        moveBackward(255, 255);
        
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_RETREAT;
          lastStateTime = micros();
        }
      }
      else if (lineBL || lineBR) {
        moveForward(255, 255);
        
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_RETREAT;
          lastStateTime = micros();
        }
      }
      else {
        currentState = STATE_SEEK;
        lastStateTime = micros();
      }
      break;
      
    case STATE_RETREAT:
      // Turn away from line
      turnRight(255, 255);
      
      if (micros() - lastStateTime >= 400000UL) {
        currentState = STATE_SEEK;
        lastStateTime = micros();
      }
      break;
  }
}

// Execute Runaway Strategy (evasive)
void executeRunawayStrategy() {
  switch (currentState) {
    case STATE_IDLE:
      // Same idle behavior as default
      stopMovement();
      if (digitalRead(JSUMO_SWITCH)) {
        if (!waitingForStart) {
          startWaitTime = micros();
          waitingForStart = true;
        }
        
        if (micros() - startWaitTime >= 3000000UL) {
          currentState = STATE_SEEK;
          lastStateTime = micros();
        }
      } else {
        waitingForStart = false;
      }
      break;
      
    case STATE_SEEK:
      // In runaway, move in evasive patterns
      moveForward(150, 100);  // Curved path
      
      if (frontDetected || leftDetected || rightDetected) {
        currentState = STATE_RETREAT;
        lastStateTime = micros();
      }
      
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY;
        lastStateTime = micros();
      }
      break;
      
    case STATE_RETREAT:
      // Run away from opponent
      if (frontDetected) {
        moveBackward(255, 255);
      } 
      else if (leftDetected) {
        turnRight(255, 255);
      }
      else if (rightDetected) {
        turnLeft(255, 255);
      }
      else {
        if (micros() - lastStateTime >= 500000UL) {
          currentState = STATE_SEEK;
          lastStateTime = micros();
        }
      }
      
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY;
        lastStateTime = micros();
      }
      break;
      
    case STATE_LINE_RECOVERY:
      // Same line recovery as default
      if (lineFL || lineFR) {
        moveBackward(255, 255);
        
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_ATTACK;
          lastStateTime = micros();
        }
      }
      else if (lineBL || lineBR) {
        moveForward(255, 255);
        
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_ATTACK;
          lastStateTime = micros();
        }
      }
      else {
        currentState = STATE_SEEK;
        lastStateTime = micros();
      }
      break;
      
    case STATE_ATTACK:
      // In runaway mode, use ATTACK state for turning
      turnRight(200, 200);
      
      if (micros() - lastStateTime >= 500000UL) {
        currentState = STATE_SEEK;
        lastStateTime = micros();
      }
      break;
  }
}

void loop() {
  // Update sensor readings
  updateSensorStates();
  triggerUltrasonic();
  
  // Check strategy selection
  currentStrategy = digitalRead(STRATEGY_SELECT_PIN) ? STRATEGY_RUNAWAY : STRATEGY_DEFAULT;
  
  // Execute selected strategy
  if (currentStrategy == STRATEGY_DEFAULT) {
    executeDefaultStrategy();
  } else {
    executeRunawayStrategy();
  }
}