import servo
from random import randint
increment=5
while True:
    try:
        print "Enter the servoAngle to be set, \n " \
              "Ctrl+C to Exit \n" \
              "Return for a random integer"
        angleInput=(raw_input())
        if not angleInput:
            angleInput=randint( 0,255)
        elif angleInput=="a":
            servoAngle+=increment
        else:
            servoAngle=int(angleInput)
        servo.move(1,servoAngle)

    except ValueError:
        print "[Error] enter a integer less than 255 \n"
    except KeyboardInterrupt:
        print " thanks for controlling the servo"
        break