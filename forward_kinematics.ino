#include <VarSpeedServo.h>

VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo my_vacuum_pump;

// Define home position
int homePos1 = 30;
int homePos2 = 65;
int homePos3 = 1;
int homePos4 = 1;  // Fixed angle for servo4

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

  // Attach servos to pins with a small delay after each
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
  delay(2000);  // Initial delay to let servos settle
}

void loop() 
{
  // Define a series of positions and actions
  int positions[][4] = {
    {30, 65, 1, 1},
    {10, 55, 1, 1},
    {80, 10, 10, 1},
    {90, 38, 15, 1},
    {75, 20, 1, 1},
    {20, 15, 20, 1},
    {25, 25, 20, 1},
    {25, 25, 20, 180},
    {25, 25, 20, -180},
    {75, 20, 1, 1},
    {30, 65, 1, 1},
    {65, 101, 35, 1},
   
    
  };

  // Move to each position with specified delay between moves
  for (int i = 0; i < 12; i++) {
    moveServos(positions[i][0], positions[i][1], positions[i][2], positions[i][3]);
    delay(5000); // Pause for 5 seconds to observe each position

    // Activate/deactivate vacuum pump based on the position index
    if (i == 1) {  // Turn vacuum pump ON at second position
      my_vacuum_pump.write(180);
      Serial.println("Vacuum pump activated.");
    } else if (i == 11) {  // Turn vacuum pump OFF at final position
      my_vacuum_pump.write(0);
      Serial.println("Vacuum pump deactivated.");
    }
  }

  Serial.println("Completed all positions. Repeating...");
  delay(2000);  // Wait for 2 seconds before repeating
}

// Function to move servos to specified angles with smooth transition
void moveServos(int angle1, int angle2, int angle3, int angle4) 
{
  // Use the VarSpeedServo library to move with speed (90 for smooth transition)
  servo1.write(angle1, 90);  // Move servo1 to angle1 at speed 90
  servo2.write(angle2, 90);  // Move servo2 to angle2 at speed 90
  servo3.write(angle3, 90);  // Move servo3 to angle3 at speed 90
  servo4.write(angle4, 90);  // Move servo4 to angle4 at speed 90

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

