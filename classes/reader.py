'''
Reads in data over a serial connection and plots the results live. Before closing, the data is saved to a .txt file.
'''

import serial
import matplotlib.pyplot as plt
import numpy as np
from classes.getArduino import GetArduino
import time
import rospy
from World.msg import MsgArduino

from Arduino import Arduino
import numpy as np
from matplotlib import pyplot as plt

class ArduinoReader():
    def __init__(self):
        connected=False
        while not connected:
            try:
                self.board = Arduino('115200')#,port="/dev/ttyACM2")  # plugged in via USB, serial com at rate 9600
                connected =True
            except ValueError:
                connected=False

        print "arduino connected"
        try:
            rospy.init_node('ard')
        except Exception as e:
            print e

        self.board.pinMode(0, "INPUT")
        self.board.pinMode(1, "INPUT")
        self.board.pinMode(2, "INPUT")
        self.board.pinMode(3, "INPUT")
        self.factor=5/1023.0


# ga=GetArduino()
# ser=ga.ser



# x=y=z=[]


    
    def publisher(self,data):
        """
        publishes the trajectory data using ROS node
        Args:
            data: message in MsgTrajectory format to be published
    
        Returns:
    
        """
        trajectory = rospy.Publisher('arduino', MsgArduino, queue_size=600)
        trajectory.publish(data)
    
    
    def message(self,flow1=None,flow2=None,flow3=None,osc=None,valve1=None,valve2=None,valve3=None):
        mes = MsgArduino()  # create message
        mes.header.stamp = rospy.Time.now()  # set time stamp
    
        mes.flow1=flow1
        mes.flow2=flow2
        mes.flow3=flow3
        mes.osc=osc
    
        mes.valve1=valve1
        mes.valve2=valve2
        mes.valve3=valve3
        return mes

    def update(self):
        f1=self.board.analogRead(0)*self.factor
        f2=self.board.analogRead(1)*self.factor
        f3=self.board.analogRead(2)*self.factor
        osc=self.board.analogRead(3)
        v1=self.board.digitalRead(2)
        v2=self.board.digitalRead(3)
        v3=self.board.digitalRead(4)
        # print x_
        self.publisher(self.message(f1,f2,f3,osc,v1,v2,v3))

    # finally:
    
        # rows = zip(x_, y_, z_)  # combines lists together
        # row_arr = np.array(rows)  # creates array from list
        # np.savetxt("test_radio2.txt",
        #            row_arr)  # save data in file (load w/np.loadtxt())
    
    
