from beginner.msg import MsgFlystate, MsgTrajectory
import rospy, sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg') # THIS MAKES IT FAST!
import matplotlib.cm as cm
from hcpWorldGen import WorldGen
x=y=None
def callback( data):
    """ Returns Wing Beat Amplitude Difference from received data"""
    global x,y
    x=data.position.x
    y=data.position.y
    return x,y
    
    
def listener():
    """ Listens to Kinefly Flystate topic"""
    rospy.Subscriber("/trajectory", MsgTrajectory, callback)
    # print "dfsd"



def trajectory():
    # fig, ax = plt.subplots()
    listener()
    plt.scatter(x,y,s=2,c=t)
    # print x,y
    plt.draw()
    plt.pause(0.1)
    plt.show()



plt.ion()
# fig=plt.figure()
hl, = plt.plot([], [])

initPlot=WorldGen()
initPlot.initPositions()
initPlot.plotPositions()

plt.axis([0,255,0,255])
rospy.init_node('plot')

play=True
t=0
while play:
    trajectory()
    t+=1

# ani = animation.FuncAnimation(fig, trajectory, interval=1000)
# plt.show()

# trajectory()