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

// Function prototypes
void testLineSensors();
void testBumpSensors();
void testIRSensors();
void testUltrasonic();
void testJSumoSwitch();
void testAllSensors();
long getUltrasonicDistance();

void setup() {
  Serial.begin(9600);
  
  // Initialize all sensor pins
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_BR, INPUT_PULLUP);
  
  pinMode(BUMP_LEFT, INPUT);
  pinMode(BUMP_RIGHT, INPUT);
  
  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);
  
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  pinMode(JSUMO_SWITCH, INPUT);
  
  Serial.println("===== Sensor Test Program =====");
  Serial.println("Select a test by entering a number:");
  Serial.println("1: Line Sensors");
  Serial.println("2: Bump Sensors");
  Serial.println("3: IR Reflectance Sensors");
  Serial.println("4: Ultrasonic Sensor");
  Serial.println("5: JSumo Switch");
  Serial.println("6: All Sensors");
  Serial.println("Enter your choice:");
}

void loop() {
  if (Serial.available() > 0) {
    int choice = Serial.parseInt();
    Serial.print("Running test #");
    Serial.println(choice);
    
    switch (choice) {
      case 1:
        testLineSensors();
        break;
      case 2:
        testBumpSensors();
        break;
      case 3:
        testIRSensors();
        break;
      case 4:
        testUltrasonic();
        break;
      case 5:
        testJSumoSwitch();
        break;
      case 6:
        testAllSensors();
        break;
      default:
        Serial.println("Invalid choice. Please select 1-6.");
    }
    
    // Display menu again
    Serial.println("\n===== Sensor Test Program =====");
    Serial.println("Select a test by entering a number:");
    Serial.println("1: Line Sensors");
    Serial.println("2: Bump Sensors");
    Serial.println("3: IR Reflectance Sensors");
    Serial.println("4: Ultrasonic Sensor");
    Serial.println("5: JSumo Switch");
    Serial.println("6: All Sensors");
    Serial.println("Enter your choice:");
  }
}

void testLineSensors() {
  Serial.println("\n--- Line Sensor Test ---");
  Serial.println("Place and remove the robot over white/black boundaries to see readings");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    Serial.print("FL: ");
    Serial.print(digitalRead(LINE_SENSOR_FL) ? "HIGH" : "LOW");
    Serial.print("\tFR: ");
    Serial.print(digitalRead(LINE_SENSOR_FR) ? "HIGH" : "LOW");
    Serial.print("\tBL: ");
    Serial.print(digitalRead(LINE_SENSOR_BL) ? "HIGH" : "LOW");
    Serial.print("\tBR: ");
    Serial.println(digitalRead(LINE_SENSOR_BR) ? "HIGH" : "LOW");
    delay(500);
  }
  Serial.read(); // Clear the buffer
}

void testBumpSensors() {
  Serial.println("\n--- Bump Sensor Test ---");
  Serial.println("Press the bump sensors to see readings");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    Serial.print("Left: ");
    Serial.print(digitalRead(BUMP_LEFT) ? "Pressed" : "Not Pressed");
    Serial.print("\tRight: ");
    Serial.println(digitalRead(BUMP_RIGHT) ? "Pressed" : "Not Pressed");
    delay(500);
  }
  Serial.read(); // Clear the buffer
}

void testIRSensors() {
  Serial.println("\n--- IR Reflectance Sensor Test ---");
  Serial.println("Place objects in front of IR sensors to see readings");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    Serial.print("Left: ");
    Serial.print(analogRead(IR_REFLECT_LEFT));
    Serial.print("\tCenter: ");
    Serial.print(analogRead(IR_REFLECT_CENTER));
    Serial.print("\tRight: ");
    Serial.println(analogRead(IR_REFLECT_RIGHT));
    delay(500);
  }
  Serial.read(); // Clear the buffer
}

long getUltrasonicDistance() {
  // Clear the trigger pin
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
  
  // Calculate the distance
  long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  return distance;
}

void testUltrasonic() {
  Serial.println("\n--- Ultrasonic Sensor Test ---");
  Serial.println("Move objects in front of the ultrasonic sensor to see readings");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    long distance = getUltrasonicDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
  }
  Serial.read(); // Clear the buffer
}

void testJSumoSwitch() {
  Serial.println("\n--- JSumo Switch Test ---");
  Serial.println("Toggle the JSumo switch to see readings");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    Serial.print("JSumo Switch: ");
    Serial.println(digitalRead(JSUMO_SWITCH) ? "ON" : "OFF");
    delay(500);
  }
  Serial.read(); // Clear the buffer
}

void testAllSensors() {
  Serial.println("\n--- All Sensors Test ---");
  Serial.println("Displaying readings from all sensors");
  Serial.println("Press any key to stop the test");
  
  while (!Serial.available()) {
    // Line sensors
    Serial.println("Line Sensors:");
    Serial.print("FL: ");
    Serial.print(digitalRead(LINE_SENSOR_FL) ? "HIGH" : "LOW");
    Serial.print("\tFR: ");
    Serial.print(digitalRead(LINE_SENSOR_FR) ? "HIGH" : "LOW");
    Serial.print("\tBL: ");
    Serial.print(digitalRead(LINE_SENSOR_BL) ? "HIGH" : "LOW");
    Serial.print("\tBR: ");
    Serial.println(digitalRead(LINE_SENSOR_BR) ? "HIGH" : "LOW");
    
    // Bump sensors
    Serial.println("Bump Sensors:");
    Serial.print("Left: ");
    Serial.print(digitalRead(BUMP_LEFT) ? "Pressed" : "Not Pressed");
    Serial.print("\tRight: ");
    Serial.println(digitalRead(BUMP_RIGHT) ? "Pressed" : "Not Pressed");
    
    // IR sensors
    Serial.println("IR Reflectance Sensors:");
    Serial.print("Left: ");
    Serial.print(analogRead(IR_REFLECT_LEFT));
    Serial.print("\tCenter: ");
    Serial.print(analogRead(IR_REFLECT_CENTER));
    Serial.print("\tRight: ");
    Serial.println(analogRead(IR_REFLECT_RIGHT));
    
    // Ultrasonic sensor
    Serial.println("Ultrasonic Sensor:");
    Serial.print("Distance: ");
    Serial.print(getUltrasonicDistance());
    Serial.println(" cm");
    
    // JSumo switch
    Serial.println("JSumo Switch:");
    Serial.print("State: ");
    Serial.println(digitalRead(JSUMO_SWITCH) ? "ON" : "OFF");
    
    Serial.println("------------------------------");
    delay(1000);
  }
  Serial.read(); // Clear the buffer
}