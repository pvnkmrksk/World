#include <uStepper.h>

uStepper stepper;
float scal=20.48;//11.37;//20.48;

void setup() {
  // put your setup code here, to run once:
  stepper.setup();
  stepper.setMaxAcceleration(2000);
  stepper.setMaxVelocity(1500);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
//  if(!stepper.getMotorState())
//  {
//   
//    stepper.moveSteps(3000, !stepper.getCurrentDirection(), HARD);
//  }
  delay(200);
//   Serial.print("Temperature: ");
//   Serial.print(stepper.temp.getTemp());
//   Serial.println(" Degrees Celsius");
   Serial.print("Angle: ");
   Serial.print(stepper.encoder.angle/scal);
   Serial.println(" Degrees");
}
