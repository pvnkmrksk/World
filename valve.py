import servo
while True:
    try:
        print "Enter the valve state 0=off, 1=on"
        valveState=(raw_input())
        servoAngle=int(valveState)
        servo.move(99,servoAngle)#99 is the valve case


    except KeyboardInterrupt:
        print " thanks for controlling the valve"
        servo.move(99,0) #close valve on exit
        break

    except ValueError:
        print "enter 0 or 1"