from beginner.msg import MsgFlystate, MsgTrajectory
import rospy, sys
from std_msgs.msg import String


import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg') # THIS MAKES IT FAST!
import matplotlib.cm as cm
from worldGen import WorldGen
x=y=None
keys=String()

def callback( data):
    """ Returns Wing Beat Amplitude Difference from received data"""
    global x,y,key
    x=data.position.x
    y=data.position.y
    keys =data.key
    return x,y,keys


def listener():
    """ Listens to Kinefly Flystate topic"""
    rospy.Subscriber("/trajectory", MsgTrajectory, callback)



def trajectory():
    listener()
    plt.scatter(x,y,s=1,c=t)
    plt.pause(0.01)
    plt.draw()


def initPlot():

    initPlot=WorldGen()
    initPlot.initPositions()
    initPlot.plotPositions()
    plt.axis([0,255,0,255])


def press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'c':
        plt.clf()
        initPlot()
        fig.canvas.draw()
    if event.key == 'x':
        global play
        play=False
        plt.close()

# keys=None
plt.ion()

fig, ax = plt.subplots()

fig.canvas.mpl_connect('key_press_event', press)

rospy.init_node('plot')
initPlot()
plt.draw()


play=True
t=0
while play:
    trajectory()

    t+=1

 print("Done")