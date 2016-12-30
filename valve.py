from classes.valveHandler import ValveHandler
increment=5
v1=ValveHandler(casePort=97,compression=True,baud=115200)
v2=ValveHandler(casePort=98,compression=True,baud=115200)
v3=ValveHandler(casePort=99,compression=True,baud=115200)
# while True:
#     try:
#         print "Enter the servoAngle to be set, \n " \
#               "Ctrl+C to Exit \n" \
#               "Return for a random integer"
#         angleInput=(raw_input())
#         # if not angleInput:
#         #     angleInput=randint( 0,255)
#         # elif angleInput=="a":
#         #     servoAngle+=increment
#         # else:
#         #     servoAngle=int(angleInput)
#         servoAngle=int(angleInput)
#         # print "servoangle uis",servoAngle
#         v.move(servoAngle)
#         print "analog 1 value is",ord(v.serPort.read())
#         print ('\n\n')
#         # servo.move(1,servoAngle)
#
#     except ValueError:
#         print "[Error] enter a integer less than 255 \n"
#     except KeyboardInterrupt:
#         print " thanks for controlling the servo"
#         break
#     #     breakwhile True:
    # try:
    #     print "Enter the servoAngle to be set, \n " \
    #           "Ctrl+C to Exit \n" \
    #           "Return for a random integer"
    #     angleInput=(raw_input())
    #     if not angleInput:
    #         angleInput=randint( 0,255)
    #     elif angleInput=="a":
    #         servoAngle+=increment
    #     else:
    #         servoAngle=int(angleInput)
    #     servo.move(1,servoAngle)
    #
    # except ValueError:
    #     print "[Error] enter a integer less than 255 \n"
    # except KeyboardInterrupt:
    #     print " thanks for controlling the servo"
    #     break



def cleanExit():
    print " thanks for controlling the valve"

    v1.move(0)  # close valve on exit
    v2.move(0)  # close valve on exit
    v3.move(0)  # close valve on exit

print "Enter the valve number "
valve = (raw_input())

while True:
    try:
        print "Enter the valve state 0=off, 1=on"
        try:
            valveState=bool(int(raw_input()))*1
        except TypeError:
            print "enter either 1 or 0"
            continue
        if valve==1:
            v1.move(valveState)
        elif valve==2:
            v2.move(valveState)
        elif valve==3:
            v3.move(valveState)
        else:
            print "no valve activated"

    except KeyboardInterrupt:
        try:
            print "what valve do you want to control, type 'exit' or ctrl+c to exit"
            valve = int(raw_input())

            if valve=='exit':
                cleanExit()
                break

        except KeyboardInterrupt:
            cleanExit()
            break


    except ValueError:
        print "enter 0 or 1"

