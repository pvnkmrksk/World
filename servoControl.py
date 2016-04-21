import servo
while True:
    try:
        print "Enter the servoAngle to be set"
        servoAngle=int(raw_input())
        servo.move(1,servoAngle)
    except KeyboardInterrupt:
        print " thanks for controlling the servo"
        break