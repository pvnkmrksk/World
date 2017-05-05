
#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

int pos = 50;

void setup()
{  
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(100);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    delay(3000);
    pos = -pos;
    stepper.moveTo(pos);\
  }
  stepper.run();
}
