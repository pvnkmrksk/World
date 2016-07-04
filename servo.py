#!/usr/bin/env python

################################################
# Module:   servo.py
# Created:  2 April 2008
# Author:   Brian D. Wendt
#   http://principialabs.com/
# Version:  0.3
# License:  GPLv3
#   http://www.fsf.org/licensing/
'''
Provides a serial connection abstraction layer
for use with Arduino "MultipleSerialServoControl" sketch.
'''
################################################

import serial


usbport0 = '/dev/ttyACM0'
usbport1 = '/dev/ttyACM1'
# Set up serial baud rate

try :
    ser = serial.Serial(usbport0, 9600)

except serial.SerialException:
    ser = serial.Serial(usbport1, 9600)
        

def move(servo, angle):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    ser.write(chr(255))
    ser.write(chr(servo))

    if (0 <= angle <= 180):
        # ser.write(chr(255))
        # ser.write(chr(servo))
        ser.write(chr(angle))

    elif (180<angle<=240):
        ser.write(chr(180))

    elif (300<angle<=360):
        ser.write(chr(0))
    #60 degrees of buffer space to handle noise in turning. Will only bounce to other side if 30 degrees away from
        #end point like. Will bounce to 0 if it is 300 to 360. Will bounce to 180 only if between 180 and 240
    # else:
    #     print "Servo angle must be an integer between 0 and 180.\n"

i=0
j=0

if __name__ =="__main__":

    for i in range(0,180,18):
        for j in range(1,96):
            move(1,i)
            print(i,j)
