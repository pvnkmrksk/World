#include <AccelStepper.h>

AccelStepper stepper(1, STEP, DIR);

int spd = 1000;    // The current speed in steps/second
int sign = 1;      // Either 1, 0 or -1

// User input for servo and position
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int i;               // iterator

void setup()
{  
  Serial.begin(115200);
  stepper.setMaxSpeed(1250);
  stepper.setSpeed(1000);    
}

void loop()
{  



 if (Serial.available() > 2) {
    // Read the first byte
    
    startbyte = Serial.read();
    // If it's really the startbyte (255) ...
    if (startbyte == 255) {
      // ... then get the next two bytes
      for (i=0;i<2;i++) {
        userInput[i] = Serial.read();
      }
     
      // First byte = servo to move?
      servo = userInput[0];
      // Second byte = which position?
      pos = userInput[1];

      // Packet error checking and recovery
      if (pos == 255) { servo = 255; }

      // Assign new position to appropriate servo
      switch (servo) {
        case 1:
//          servo1.write(pos);    // move servo1 to 'pos'
          stepper.moveTo(pos);
          stepper.setSpeed(900);
//      Serial.println("moving");
//      Serial.println(pos);
        Serial.write(analogRead(A0)/4);
          break;
      }}}

  stepper.runSpeedToPosition();
//  stepper.runSpeed();
//    
//            stepper.moveTo(pos);
//            stepper.setSpeed(100000);
//            stepper.runSpeedToPosition();
}
