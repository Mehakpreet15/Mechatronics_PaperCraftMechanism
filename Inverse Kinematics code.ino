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

// Solenoid valve pin
const int solenoidValvePin = 11; // Pin for solenoid valve
const int vacuumPumpPin = 12;     // Pin for vacuum pump

// Home position coordinates
float homeX = -15.0; // Home X coordinate in cm
float homeY = -15.0; // Home Y coordinate in cm
float homeZ = -10.0; // Home Z coordinate in cm

// Home position angle (in degrees)
const int homeAngle = 20; // Start at 20 degrees

void setup() {
  // Attach servos to their respective pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  // Attach the vacuum pump to its pin
  vacuumPump.attach(vacuumPumpPin);
  pinMode(solenoidValvePin, OUTPUT); // Set solenoid valve pin as output

  // Move all servos to their home position (20 degrees)
  moveToHomePosition();
  delay(5000); // Wait for 5 seconds at the home position
}

void loop() {
  // First target coordinates
  float targetX1 = 30.0;  // X coordinate in cm (relative to home)
  float targetY1 = 10.0;  // Y coordinate in cm (relative to home)
  float targetZ1 = -10.0; // Z coordinate in cm

  // Calculate the inverse kinematics for the first target position
  float angle1 = calculateInverseKinematics(targetX1, targetY1, targetZ1, 0);
  float angle2 = calculateInverseKinematics(targetX1, targetY1, targetZ1, 120);
  float angle3 = calculateInverseKinematics(targetX1, targetY1, targetZ1, 240);

  // Move servos to the calculated angles for the first target position
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  delay(500); // Brief delay to ensure position is reached

  // Activate the solenoid valve (turn it ON)
  digitalWrite(solenoidValvePin, HIGH); // Solenoid ON
  vacuumPump.write(180); // Turn the vacuum pump ON (Suction is ON)

  // Move to 3 cm below the target position for suction
  float suctionZ = targetZ1 - 3.0; // Adjust Z coordinate for suction

  // Move to the suction position
  angle1 = calculateInverseKinematics(targetX1, targetY1, suctionZ, 0);
  angle2 = calculateInverseKinematics(targetX1, targetY1, suctionZ, 120);
  angle3 = calculateInverseKinematics(targetX1, targetY1, suctionZ, 240);

  // Move servos to the calculated angles for the suction position
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);

  delay(5000); // Wait for 5 seconds at the suction position

  // Second target coordinates
  float targetX2 = -30.0; // X coordinate in cm (relative to home)
  float targetY2 = 10.0;  // Y coordinate in cm (relative to home)
  float targetZ2 = -10.0; // Z coordinate in cm

  // Calculate the inverse kinematics for the second target position
  angle1 = calculateInverseKinematics(targetX2, targetY2, targetZ2, 0);
  angle2 = calculateInverseKinematics(targetX2, targetY2, targetZ2, 120);
  angle3 = calculateInverseKinematics(targetX2, targetY2, targetZ2, 240);

  // Move servos to the calculated angles for the second target position
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);

  delay(5000); // Wait for 5 seconds at the second target position

  // Return to the home position
  moveToHomePosition();

  // Deactivate the solenoid valve (turn it OFF)
  digitalWrite(solenoidValvePin, LOW); // Solenoid OFF
  vacuumPump.write(0); // Turn the vacuum pump OFF (Suction is OFF)
  delay(5000); // Wait for 5 seconds at the home position

  // Optional delay before the next loop iteration
  delay(2000);
}

// Function to move all servos to the home position (20 degrees)
void moveToHomePosition() {
  servo1.write(homeAngle);
  servo2.write(homeAngle);
  servo3.write(homeAngle);
}

// Function to stop all motors
void stopMotors() {
  servo1.write(homeAngle); // You can set it to home position or stop at current position
  servo2.write(homeAngle);
  servo3.write(homeAngle);
}

// Function to calculate the inverse kinematics for a given target position
float calculateInverseKinematics(float x, float y, float z, float baseAngle) {
  // Convert the base angle to radians
  float baseRad = radians(baseAngle);

  // Project the target position onto the corresponding plane
  float projectedX = x * cos(baseRad) + y * sin(baseRad);
  float projectedY = y * cos(baseRad) - x * sin(baseRad);

  // Calculate the squared distance from the base to the projected position
  float distanceSquared = projectedX * projectedX + z * z;

  // Use trigonometric relationships to solve for the servo angle
  float angle = acos((distanceSquared + activeArmLength * activeArmLength - passiveArmLength * passiveArmLength) /
                     (2 * activeArmLength * sqrt(distanceSquared))) + atan2(z, projectedX);

  // Convert the angle from radians to degrees
  return degrees(angle);
}
