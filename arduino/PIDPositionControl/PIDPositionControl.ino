/*
*			Example sketch for PID position control!
*
*	This example demonstrates how easy closed loop position control can be achieved using the uStepper !
*	The only thing needed to activate closed loop control, is in the stepper.setup() function, where the
*	object is initiated with the keyword "PID", followed by the microstepping setting, faultTolerance (in steps),
*	hysteresis (in steps) and the P, I and D coefficients. For more information, check out the documentation:
*	http://ustepper.com/docs/html/index.html
*
*	Once the PID is activated, the use of the library is the same as without the PID !
*
*/

#include <uStepper.h>

uStepper stepper(1000,1000);
float scal=20.48;//11.37;//20.48;
float rn=0;
float cmd;
float got;
bool dir;
float go = 160;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 6;           // interval at which to blink (milliseconds)

void setup(void)
{
	Serial.begin(115200);
// stepper.setup();
 stepper.setup(PID,FULL,2,1,5,5,5);     //Initiate the stepper object to use closed loop PID control
//	stepper.setup(PID,FULL,2,1,5,0.02,0.006);     //Initiate the stepper object to use closed loop PID control
                                                      	//The behaviour of the controller can be adjusted by tuning 
  Serial.setTimeout(10);                                        	//the P, I and D paramenters in this initiation (the last three parameters)
                                                      	//Also the hysteresis and faultTolerance can be adjusted by adjusting 
                                                      	//parameter 3 and 4. For more information, check out the documentation:
                                                      	//http://ustepper.com/docs/html/index.html

//	stepper.moveSteps(200,CCW,HARD);               	//turn shaft 3200 steps, counterClockWise (equal to one revolution)
  Serial.print(stepper.encoder.angle);    //print out the current angle of the motor shaft.

}

void loop(void)
{

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    rn = random(3)-1;
    go +=rn;
  Serial.println();
  Serial.println("got\tangle\tcmd");
  Serial.print(got);    //print out the current angle of the motor shaft.
  Serial.print("\t");
  Serial.print(stepper.encoder.angle/scal);    //print out the current angle of the motor shaft.
  Serial.print("\t");
  Serial.println(cmd);    //print out the current angle of the motor shaft.
  }

//    Serial.print(stepper.encoder.angle/scal);    //print out the current angle of the motor shaft.
//    Serial.print("\t");
//  if (Serial.available()>0){
//  float go = Serial.parseInt();
  
  if (go>170){go=70;Serial.println("\n\n\nfull turn\n\n\n");}
  else if (go<70){go=170;Serial.println("\n\n\nrever full turn\n\n\n");}
  
  go=constrain(go,70,170);
  
  if (go!=0){
    got=go;
    } 
  if (got!=0){
  cmd=int(got-(stepper.encoder.angle/scal))%200;
  }
  if (cmd!=0 && got!=0) {
    if (cmd>0 ){
      dir=false;
     }

     if (cmd<0){
      dir=true;
      cmd=abs(cmd);
     }
    stepper.moveSteps(cmd,dir,HARD);
   
   }
 }

