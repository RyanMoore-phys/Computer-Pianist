/*
 * Physics 124 Lab 4: FINAL SUBMISSION CODE
 * Status: Hardware Fixed. Logic Polished.
 * * CHANGES FROM PREVIOUS:
 * 1. Added "Startup Safety": Checks if sensor is blocked immediately on boot.
 * 2. Fixed "Restoration Timer": Uses subtraction instead of modulo (%) for reliability.
 * 3. Startup Angle: Defaults to 90 (Center) to prevent jumping to 0 on start.
 */

#include <Servo.h>

// --- Pin Setup ---
const int PIN_SERVO = 9;
const int PIN_PHOTO_LEFT = A0;
const int PIN_PHOTO_RIGHT = A1;
const int PIN_PROXIMITY = 2; 

// --- Tuning Parameters ---
const int DEAD_BAND = 15;       // Slightly higher to prevent jitter
const int SLEEP_THRESHOLD = 10; // Low threshold for dark rooms
const int MAX_STEP = 5;         // Smooth speed

Servo myServo;
int currentAngle = 90;          // Start centered (Safer than 0)

// Volatile variables for Interrupts
volatile int maxAllowedAngle = 180; 

// Timer variable for restoring range
unsigned long lastRestoreTime = 0;

void setup() {
  Serial.begin(9600);
  
  myServo.attach(PIN_SERVO);
  myServo.write(currentAngle);
  
  // Use INPUT_PULLUP to stabilize the IR signal
  pinMode(PIN_PROXIMITY, INPUT_PULLUP);
  
  // 1. INTERRUPT: Catches new objects arriving
  attachInterrupt(digitalPinToInterrupt(PIN_PROXIMITY), handleCollision, FALLING);

  // 2. STARTUP SAFETY: Checks if object is ALREADY there
  if (digitalRead(PIN_PROXIMITY) == LOW) {
    maxAllowedAngle = 90; // Limit range immediately
    Serial.println("Warning: Started with obstruction!");
  }

  Serial.println("--- SYSTEM READY ---");
}

void loop() {
  // --- A. SAFETY & RESTORATION LOGIC ---
  
  // If currently blocked, ensure we obey the limit
  if (digitalRead(PIN_PROXIMITY) == LOW) {
    if (currentAngle > maxAllowedAngle) {
       currentAngle = maxAllowedAngle;
       myServo.write(currentAngle);
    }
  }
  // If CLEAR, slowly restore range (Fixes the % bug)
  else if (maxAllowedAngle < 180) {
    if (millis() - lastRestoreTime > 100) { // Every 100ms
       maxAllowedAngle++;
       lastRestoreTime = millis();
    }
  }

  // --- B. SENSOR READING ---
  int valLeft = analogRead(PIN_PHOTO_LEFT);
  int valRight = analogRead(PIN_PHOTO_RIGHT);
  int avg = (valLeft + valRight) / 2;
  int diff = valLeft - valRight;

  // --- C. MOVEMENT LOGIC ---
  if (avg < SLEEP_THRESHOLD) {
    moveToAngle(90); // Go to sleep
  }
  else {
    if (abs(diff) > DEAD_BAND) {
      // Proportional Speed
      int stepSize = map(abs(diff), DEAD_BAND, 600, 1, MAX_STEP);
      stepSize = constrain(stepSize, 1, MAX_STEP);

      if (valLeft > valRight) {
        moveToAngle(currentAngle + stepSize); // Left
      } else {
        moveToAngle(currentAngle - stepSize); // Right
      }
    }
  }

  delay(30); // Loop speed
}

// --- Helper: Safe Move ---
void moveToAngle(int target) {
  // Enforce Dynamic Limit
  if (target > maxAllowedAngle) target = maxAllowedAngle;
  
  // Enforce Hardware Limits
  if (target < 0) target = 0;
  if (target > 180) target = 180;

  currentAngle = target;
  myServo.write(currentAngle);
}

// --- Interrupt: Emergency Stop ---
void handleCollision() {
  // Back off 20 degrees immediately
  int safePos = currentAngle - 20; 
  if (safePos < 0) safePos = 0;

  maxAllowedAngle = safePos;
  currentAngle = safePos;
  myServo.write(currentAngle);
}
