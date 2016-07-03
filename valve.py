import servo
while True:
    try:
        print "Enter the valve state 0=off, 1=on"
        valveState=int(raw_input())
        servo.move(99, valveState) #99 is the valve case
    except KeyboardInterrupt:
        print " thanks for controlling the valve"
        break