from beginner.msg import MsgFlystate, MsgTrajectory
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg') # THIS MAKES IT FAST!
import matplotlib.cm as cm
x=y=None
def callback( data):
    """ Returns Wing Beat Amplitude Difference from received data"""
    global x,y
    x=data.position.x
    y=data.position.y
    # wbad = data.left.angles[0] - data.right.angles[0]
    # wbas = data.left.angles[0] + data.right.angles[0]
    # scaledWbad=lrGain*data.left.angles[0] - data.right.angles[0]
    # if disabledFly:
    #     wbad=scaledWbad
    # print x,y
    return x,y
    
    
def listener():
    """ Listens to Kinefly Flystate topic"""
    rospy.Subscriber("/trajectory", MsgTrajectory, callback)
    # print "dfsd"



def trajectory():
    # fig, ax = plt.subplots()
    rospy.init_node('plot')
    listener()
    plt.scatter(x,y,s=2,c=n)
    # print x,y
    print n
    plt.draw()
    plt.pause(0.05)


n=0
plt.ion()
# fig=plt.figure()
hl, = plt.plot([], [])
plt.axis([0,255,0,255])


while n<1000:
    trajectory()
    n+=1
    print n
# ani = animation.FuncAnimation(fig, trajectory, interval=1000)
# plt.show()

# trajectory()