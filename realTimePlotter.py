from World.msg import MsgFlystate, MsgTrajectory
import rospy, sys
from std_msgs.msg import String
from params import parameters
from classes.fieldGen import FieldGen
# modules
# ------------------------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from worldGen import WorldGen
f=FieldGen()
scale=20
def callback( data):
    """ Returns Wing Beat Amplitude Difference from received data"""
    global x,y,frame
    x[frame]=data.position.x
    y[frame]=data.position.y

    return x,y


def listener():
    """ Listens to Kinefly Flystate topic"""
    rospy.Subscriber("/trajectory", MsgTrajectory, callback)


def initPlot():
    # plt.axis([0,255,0,255])
    # if parameters['loadOdour']:
    #     from world import odourFieldGen, plumeStripGen
    #     odourFieldGen()

    # odourField = f.odourPacket(width=257, height=257, scale=scale,
    #                                          packetFrequency=20, plot=False,
    #                                          packetDuration=.02)
    # odourField = f.odourPacket(width=parameters['worldSize'],
    #                                          height=parameters['worldSize'],
    #                                          scale=parameters['fieldRescale'],
    #                                          packetFrequency=parameters['packetFrequency'],
    #                                          plot=False,
    #                                        velocity=parameters['maxSpeed'],
    #
    #                            packetDuration=parameters['packetDuration'])
    #
    # plt.imshow(odourField,cmap='Greys',interpolation=None,alpha=.1,aspect='auto')
    # plt.tight_layout()

    initPlot=WorldGen()
    initPlot.initPositions()
    initPlot.plotPositions()



rospy.init_node('plot')
dur=1
# dur=200000
dt=30./parameters['fps']
x=np.zeros(dur)
y=np.zeros(dur)
frame=0

listener()
for i in range(1000):
    # print x,y
    import time
    time.sleep(0.1)
plt.close('all')  # close all previous plots
fig = plt.figure(1, figsize=(12,12))
ax = plt.axes()
# ax = plt.axes(xlim=(0, 255), ylim=(0, 255))
scat = ax.scatter([], [], s=.1,c='r')


def press(event):
    global x,y
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'c':

        x = np.zeros(dur)
        y = np.zeros(dur)

    if event.key == 'x':
        global play
        play=False
        plt.close()

def init():
    initPlot()
    scat.set_offsets([])
    return scat,

def animate(i):
    global frame,scale
    frame=i

    data = np.hstack((scale*x[:i,np.newaxis], scale*y[:i, np.newaxis]))
    scat.set_offsets(data)
    return scat,

fig.canvas.mpl_connect('key_press_event', press)

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(x)+1,
                               interval=dt, blit=True, repeat=False)


plt.show()

