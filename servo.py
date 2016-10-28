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
    ser = serial.Serial(usbport0, 1200)
    print "Arduino connected via port 0 \n \n"

# except:
#     print "moving on"
except serial.SerialException:
    try:
        ser = serial.Serial(usbport1, 1200)
        print "Arduino connected via port 1 \n \n"

    except serial.SerialException:
        try:
            ser = serial.Serial(usbport2, 1200)
            print "Arduino connected via port 2 \n \n"

        except serial.SerialException:
            ser = serial.Serial(usbport3, 1200)
            print "Arduino connected via port 3 \n \n"

        except:
            pass


frame=0


def move(servo, angle, anglePrev):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    if angle!=anglePrev: #update only of different
        try:
            ser.write(chr(255))
            ser.write(chr(servo))
            ser.write((chr(int(angle))))
        except:
            print "something reallly bad"






if __name__ =="__main__":

    for i in range(0,180,18):
        for j in range(1,96):
            move(1,i)
            print(i,j)
