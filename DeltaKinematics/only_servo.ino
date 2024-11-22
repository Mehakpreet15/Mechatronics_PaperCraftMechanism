#include <DeltaKinematics.h>
#include <VarSpeedServo.h>

// Delta robot kinematics
DeltaKinematics DK(70, 300, 139, 112);

// Servos for the Delta robot
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4; // Optional for vacuum pump or additional mechanism
VarSpeedServo my_vacuum_pump;

// Define home position
int homePos1 = 30;
int homePos2 = 65;
int homePos3 = 1;
int homePos4 = 1; // Fixed angle for servo4

// Define vacuum pump pin
int vacuum_pump_pin = 12;

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Attach the vacuum pump to the vacuum pump control object
  my_vacuum_pump.attach(vacuum_pump_pin);

  // Start with vacuum pump OFF (0 is OFF, 180 is ON)
  my_vacuum_pump.write(0);

  // Attach servos to pins
  servo1.attach(7);
  delay(100);
  servo2.attach(6);
  delay(100);
  servo3.attach(5);
  delay(100);
  servo4.attach(9);
  delay(100);

  // Move servos to initial home position
  moveServos(homePos1, homePos2, homePos3, homePos4);
  delay(2000); // Initial delay to let servos settle
}

void loop()
{
  // Define a series of Cartesian positions for the Delta robot
  float positions[][3] = {
      {0, 0, -300},
      {0, 0, -270},
      {100, 100, -270},
      {50, 50, -250},
      {-50, -50, -250},
      {0, 0, -300} // Return to home
  };

  // Iterate through positions and move servos accordingly
  for (int i = 0; i < 6; i++)
  {
    DK.x = positions[i][0];
    DK.y = positions[i][1];
    DK.z = positions[i][2];
    DK.inverse();

    // Log the Cartesian and servo angles
    Serial.print("Moving to Cartesian position (");
    Serial.print(DK.x);
    Serial.print(", ");
    Serial.print(DK.y);
    Serial.print(", ");
    Serial.print(DK.z);
    Serial.println(")");

    Serial.print("Servo angles: ");
    Serial.print(DK.a);
    Serial.print(", ");
    Serial.print(DK.b);
    Serial.print(", ");
    Serial.println(DK.c);

    // Move the servos to calculated angles
    moveServos(DK.a, DK.b, DK.c, homePos4);

    delay(5000); // Pause for 5 seconds to observe each position

    // Activate/deactivate vacuum pump based on position index
    if (i == 1)
    { // Turn vacuum pump ON at second position
      my_vacuum_pump.write(180);
      Serial.println("Vacuum pump activated.");
    }
    else if (i == 5)
    { // Turn vacuum pump OFF at final position
      my_vacuum_pump.write(0);
      Serial.println("Vacuum pump deactivated.");
    }
  }

  Serial.println("Completed all positions. Repeating...");
  delay(2000); // Wait for 2 seconds before repeating
}

// Function to move servos to specified angles with smooth transition
void moveServos(int angle1, int angle2, int angle3, int angle4)
{
  // Use the VarSpeedServo library to move with speed (90 for smooth transition)
  servo1.write(angle1, 90); // Move servo1 to angle1 at speed 90
  servo2.write(angle2, 90); // Move servo2 to angle2 at speed 90
  servo3.write(angle3, 90); // Move servo3 to angle3 at speed 90
  servo4.write(angle4, 90); // Move servo4 to angle4 at speed 90

  // Print angles for debugging
  Serial.print("Moving to positions - Servo1: ");
  Serial.print(angle1);
  Serial.print(", Servo2: ");
  Serial.print(angle2);
  Serial.print(", Servo3: ");
  Serial.print(angle3);
  Serial.print(", Servo4: ");
  Serial.println(angle4);
}

