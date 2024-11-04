#include <VarSpeedServo.h>

// Lengths of the arms in cm
const float activeArmLength = 10.0;  // Active arm length
const float passiveArmLength = 20.0; // Passive arm length

// Servo objects
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;

// Vacuum pump control object
VarSpeedServo vacuumPump;

// Motor pins
const int servoPin1 = 8;
const int servoPin2 = 9;
const int servoPin3 = 10;

// Solenoid valve and vacuum pump pins
const int solenoidValvePin = 11; // Pin for solenoid valve
const int vacuumPumpPin = 12;    // Pin for vacuum pump

// Home position angle (in degrees)
const int homeAngle = 20; // Home angle for all servos

void setup() {
  // Attach servos to their respective pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  // Attach the vacuum pump to its pin
  vacuumPump.attach(vacuumPumpPin);

  // Set solenoid valve pin as output
  pinMode(solenoidValvePin, OUTPUT);

  // Move servos to the home position
  moveToHomePosition();
  delay(10000); // Wait for 5 seconds at the home position
}

void loop() {
  // First target coordinates
  float targetX1 = 30.0;  // X coordinate in cm
  float targetY1 = 10.0;  // Y coordinate in cm
  float targetZ1 = -10.0; // Z coordinate in cm

  // Move to the first target position
  moveToTarget(targetX1, targetY1, targetZ1);

  // Activate the solenoid valve and vacuum pump after reaching the first target
  digitalWrite(solenoidValvePin, HIGH); // Solenoid ON
  vacuumPump.write(180);                // Turn the vacuum pump ON (Suction ON)
  delay(10000);                          // Hold for 5 seconds at the first target position

  // Second target coordinates
  float targetX2 = -30.0; // X coordinate in cm
  float targetY2 = 10.0;  // Y coordinate in cm
  float targetZ2 = -10.0; // Z coordinate in cm

  // Move to the second target position
  moveToTarget(targetX2, targetY2, targetZ2);
  delay(10000); // Hold for 5 seconds at the second target position

  // Return to the home position
  moveToHomePosition();

  // Deactivate the solenoid valve and vacuum pump after reaching home
  digitalWrite(solenoidValvePin, LOW); // Solenoid OFF
  vacuumPump.write(0);                 // Turn the vacuum pump OFF (Suction OFF)
  delay(10000);                         // Hold for 5 seconds at the home position

  // Optional delay before the next iteration
  delay(2000);
}

// Function to move all servos to the home position
void moveToHomePosition() {
  servo1.write(homeAngle, 30); // 30 = speed
  servo2.write(homeAngle, 30);
  servo3.write(homeAngle, 30);
}

// Function to move servos to a target position
void moveToTarget(float x, float y, float z) {
  float angle1 = calculateInverseKinematics(x, y, z, 0);
  float angle2 = calculateInverseKinematics(x, y, z, 120);
  float angle3 = calculateInverseKinematics(x, y, z, 240);

  servo1.write(angle1, 30); // 30 = speed
  servo2.write(angle2, 30);
  servo3.write(angle3, 30);
}

// Function to calculate the inverse kinematics for a given target position
float calculateInverseKinematics(float x, float y, float z, float baseAngle) {
  float baseRad = radians(baseAngle);

  // Project the target position onto the corresponding plane
  float projectedX = x * cos(baseRad) + y * sin(baseRad);
  float projectedY = y * cos(baseRad) - x * sin(baseRad);

  // Calculate the squared distance
  float distanceSquared = projectedX * projectedX + z * z;

  // Use trigonometric relationships to solve for the servo angle
  float angle = acos((distanceSquared + activeArmLength * activeArmLength - passiveArmLength * passiveArmLength) /
                     (2 * activeArmLength * sqrt(distanceSquared))) + atan2(z, projectedX);

  // Convert the angle from radians to degrees
  return degrees(angle);
}
