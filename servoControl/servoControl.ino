/*
 * ------------------------------
 *   MultipleSerialServoControl
 * ------------------------------
 *
 * Uses the Arduino Serial library
 *  (http://arduino.cc/en/Reference/Serial)
 * and the Arduino Servo library
 *  (http://arduino.cc/en/Reference/Servo)
 * to control multiple servos from a PC using a USB cable.
 *
 * Dependencies:
 *   Arduino 0017 or higher
 *     (http://www.arduino.cc/en/Main/Software)
 *   Python servo.py module
 *     (http://principialabs.com/arduino-python-4-axis-servo-control/)
 *
 * Created:  23 December 2009
 * Author:   Brian D. Wendt
 *   (http://principialabs.com/)
 * Version:  1.1
 * License:  GPLv3
 *   (http://www.fsf.org/licensing/)
 *
 */

// Import the Arduino Servo library
#include <Servo.h> 

// Create a Servo object for each servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
// TO ADD SERVOS:
//   Servo servo5;
//   etc...

//// Common servo setup values
//int minPulse = 600;   // minimum servo position, us (microseconds)
//int maxPulse = 2400;  // maximum servo position, us

// User input for servo and position
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int i;               // iterator


int valveLedPin=2;
int buzzerPin1=9;
int buzzerPin2=10;
int buzzerPin3=11;
int pinState = LOW;
int s1=3;           //servo 1 pin
int valve1 =7;
int valve2 =8;
int valve3 =4;

void setup()
{ 
  // Attach each Servo object to a digital pin
  //servo1.attach(s1, minPulse, maxPulse);
servo1.attach(s1);
  // TO ADD SERVOS:
  //   servo5.attach(YOUR_PIN, minPulse, maxPulse);
  //   etc...

  // Valve and LED on Pin 13 for digital on/off demo
  pinMode(valve1,OUTPUT);
  pinMode(valve2,OUTPUT);
  pinMode(valve3,OUTPUT);
  
  // Open the serial connection, 115200 baud
  Serial.begin(115200);
} 

void loop() 
{ 
  // Wait for serial input (min 3 bytes in buffer)
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
//      Serial.println(pos);

      // Packet error checking and recovery
      if (pos == 255) { servo = 255; }

      // Assign new position to appropriate servo
      switch (servo) {
        case 1:
          servo1.write(pos);    // move servo1 to 'pos'
          break;


   // TO ADD SERVOS:
   //     case 5:
   //       servo5.write(pos);
   //       break;
   // etc...

        
        // LED and Valve on Pin 13 for digital on/off demo
        case 97:
          if (pos == 1) {
            digitalWrite(valve1, HIGH);
            //pinState = HIGH;
            tone(buzzerPin1,500);
          }
          if (pos == 0) {
            digitalWrite(valve1, LOW);
            noTone(buzzerPin1);
          }
          break;

          
        case 98:
          if (pos == 1) {
            digitalWrite(valve2, HIGH);
            tone(buzzerPin2,750);
          }
          if (pos == 0) {
            digitalWrite(valve2, LOW);
            noTone(buzzerPin2);
          }
          break;

        case 99:
          if (pos == 1) {
            digitalWrite(valve3, HIGH);
            tone(buzzerPin3,1250);
          }
          if (pos == 0) {
            digitalWrite(valve3, LOW);
            noTone(buzzerPin3);
          }
          break;

      }
    }
    //   Serial.print(digitalRead(valve1));
//        Serial.print("\t");                 //send a "tab" over serial
//        Serial.print(digitalRead(valve2));
//        Serial.print("\t");                 //send a "tab" over serial
//        Serial.print(digitalRead(valve3));
//        Serial.println();                   //ends the line of serial communication
       
        
  }

}

