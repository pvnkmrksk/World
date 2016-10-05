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
from params import parameters
from scipy import signal
import numpy as np


usbport0 = '/dev/ttyACM0'
usbport1 = '/dev/ttyACM1'
usbport2 = '/dev/ttyACM2'
usbport3 = '/dev/ttyACM3'
# Set up serial baud rate

try :
    ser = serial.Serial(usbport0, 9600)
    print "Arduino connected via port 0 \n \n"

except:
    print "moving on"
    pass
# except serial.SerialException:
#     try:
#         ser = serial.Serial(usbport1, 9600)
#         print "Arduino connected via port 1 \n \n"
#
#     except serial.SerialException:
#         try:
#             ser = serial.Serial(usbport2, 9600)
#             print "Arduino connected via port 2 \n \n"
#
#         except serial.SerialException:
#             ser = serial.Serial(usbport3, 9600)
#             print "Arduino connected via port 3 \n \n"
#

frame=0


def move(servo, angle):
    global prevValve,currValve, frame
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    try:
        ser.write(chr(255))
        ser.write(chr(servo))
        ser.write((chr(int(angle))))
    except:
        pass
    #
    # lb=0
    # ub=180

    # elif (lb < angle <= ub) :
    #     # ser.write(chr(255))
    #     # ser.write(chr(servo))
    #     ser.write(chr(angle))
    #
    # elif (ub<angle<=240):
    #     ser.write(chr(ub))
    #
    # elif (300<angle<=360) or angle <lb:
    #     ser.write(chr(lb))
    #60 degrees of buffer space to handle noise in turning. Will only bounce to other side if 30 degrees away from
        #end point like. Will bounce to 0 if it is 300 to 360. Will bounce to 180 only if between 180 and 240
    # else:
    #     print "Servo angle must be an integer between 0 and 180.\n"


if __name__ =="__main__":

    for i in range(0,180,18):
        for j in range(1,96):
            move(1,i)
            print(i,j)
