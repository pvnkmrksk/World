/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial monitor.

  The circuit:
   potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
   LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

// These constants won't change.  They're used to give names
// to the pins used:
const int flow1 = A0;  // Analog input pin that the potentiometer is attached to
const int flow2 = A1;
const int flow3 = A2;
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensor1 = 0;        // value read from the pot
int sensor2 = 0;
int sensor3 = 0;
float output1 = 0;        // value output to the PWM (analog out)
float output2 = 0;
float output3 = 0;
const float factor = 5.0 / 1023.0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensor1 = analogRead(flow1);
  // map it to the range of the analog out:
  delay(1);
  output1 = sensor1 * 5.0 / 1023.0;
  //output1=map(sensor1,0,1023,0,5);

  // change the analog out value:
  sensor2 = analogRead(flow2);
  delay(1);
  output2 = sensor2 * 5.0 / 1023.0;
  //  output2=map(sensor2,0,1023,0,5);
    sensor3 = analogRead(flow3);
  delay(1);
    output3 = sensor3*5.0/1023.0;

  //output3=map(sensor3,0,1023,0,5);

  // print the results to the serial monitor:
  //  Serial.print("flow1,2,3\n");
  //  Serial.println(output1);
  //  Serial.println(output2);
  //  Serial.println(output3);
  //  Serial.println("\n");
  //
  //  Serial.print(analogRead(flow1)*factor);     //read xpin and send value over serial
  //  Serial.print("\t");                 //send a "tab" over serial
  //  Serial.print(analogRead(flow2)*factor);
  //  Serial.print("\t");
  //  Serial.print(analogRead(flow3)*factor);
  //  Serial.println();                   //ends the line of serial communication
  //

  Serial.print(output1);     //read xpin and send value over serial
  Serial.print("\t");                 //send a "tab" over serial
  Serial.print(output2);
    Serial.print("\t");
    Serial.print(output3);
  Serial.println();                   //ends the line of serial communication

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(200);
}
