#include <VarSpeedServo.h>
 
// Create a vacuum pump control object
VarSpeedServo my_vacuum_pump;
 
// Attach vacuump pump to digital pin on the arduino
int vacuum_pump_pin = 10;
 
void setup() {
  // Attach the vacuum pump to the vacuum pump control object
  my_vacuum_pump.attach(vacuum_pump_pin);
 
  // Start with vacuum pump ON (0 is OFF, 180 is ON)
  my_vacuum_pump.write(180);
}
 
// The vacuum suction cup turns ON for five seconds and then
// turns OFF for five seconds.
void loop() {
  my_vacuum_pump.write(180); // Turn the vacuum pump ON (Suction is ON)
  delay(10000); // Wait five seconds
  my_vacuum_pump.write(0); // Turn the vacuum pump OFF (Suction is OFF)
  delay(2000); // Wait five seconds
   
}
