import servo



def cleanExit():
    print " thanks for controlling the valve"

    servo.move(96, 0)  # close valve on exit
    servo.move(97, 0)  # close valve on exit
    servo.move(99, 0)  # close valve on exit

print "Enter the valve number "
valve = (raw_input())

while True:
    try:
        print "Enter the valve state 0=off, 1=on"
        valveState=(raw_input())
        servoAngle=int(valveState)
        servo.move(int(valve),servoAngle)#97,98,99 is the valve case


    except KeyboardInterrupt:
        try:
            print "what valve do you want to control, type 'exit' or ctrl+c to exit"
            valve = (raw_input())

            if valve=='exit':
                cleanExit()
                break

        except KeyboardInterrupt:
            cleanExit()
            break


    except ValueError:
        print "enter 0 or 1"

