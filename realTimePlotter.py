from beginner.msg import MsgFlystate, MsgTrajectory
import rospy, sys
from std_msgs.msg import String
# modules
# ------------------------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from worldGen import WorldGen

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

    initPlot=WorldGen()
    initPlot.initPositions()
    initPlot.plotPositions()
    plt.axis([0,255,0,255])



rospy.init_node('plot')
dur=200000
dt=10/165.
x=np.zeros(dur)
y=np.zeros(dur)
frame=0

listener()
plt.close('all')  # close all previous plots
fig = plt.figure(1)
ax = plt.axes(xlim=(0, 255), ylim=(0, 255))
scat = ax.scatter([], [], s=.1)


def press(event):
    global x,y
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'c':

        # plt.clf()
        x = np.zeros(dur)
        y = np.zeros(dur)
        # plt.close('all')  # close all previous plots
        # fig = plt.figure(1)
        # ax = plt.axes(xlim=(0, 255), ylim=(0, 255))
        # scat = ax.scatter([], [], s=1)

        # fig.canvas.draw()
    if event.key == 'x':
        global play
        play=False
        plt.close()

def init():
    initPlot()
    scat.set_offsets([])
    return scat,

def animate(i):
    global frame
    frame=i
    data = np.hstack((x[:i,np.newaxis], y[:i, np.newaxis]))
    scat.set_offsets(data)
    return scat,

fig.canvas.mpl_connect('key_press_event', press)

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(x)+1,
                               interval=dt, blit=True, repeat=False)


plt.show()


# from beginner.msg import MsgFlystate, MsgTrajectory
# import rospy, sys
# from std_msgs.msg import String
#
#
# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('TkAgg') # THIS MAKES IT FAST!
# import matplotlib.cm as cm
# from worldGen import WorldGen
# x=y=None
# keys=String()
#
# def callback( data):
#     """ Returns Wing Beat Amplitude Difference from received data"""
#     global x,y,key
#     x=data.position.x
#     y=data.position.y
#     keys =data.key
#     return x,y,keys
#
#
# def listener():
#     """ Listens to Kinefly Flystate topic"""
#     rospy.Subscriber("/trajectory", MsgTrajectory, callback)
#
#
#
# def trajectory():
#     listener()
#     plt.scatter(x,y,s=1,c=t)
#     plt.pause(0.01)
#     plt.draw()
#
#
# def initPlot():
#
#     initPlot=WorldGen()
#     initPlot.initPositions()
#     initPlot.plotPositions()
#     plt.axis([0,255,0,255])
#
#
# def press(event):
#     print('press', event.key)
#     sys.stdout.flush()
#     if event.key == 'c':
#         plt.clf()
#         initPlot()
#         fig.canvas.draw()
#     if event.key == 'x':
#         global play
#         play=False
#         plt.close()
#
# # keys=None
# plt.ion()
#
# fig, ax = plt.subplots()
#
# fig.canvas.mpl_connect('key_press_event', press)
#
# rospy.init_node('plot')
# initPlot()
# plt.draw()
#
#
# play=True
# t=0
# while play:
#     trajectory()
#
#     t+=1


# def save(self, path, ext='png', close=True, verbose=True):
#         """Save a figure from pyplot.
#         Parameters
#         ----------
#         path : string
#             The path (and filename, without the extension) to save the
#             figure to.
#         ext : string (default='png')
#             The file extension. This must be supported by the active
#             matplotlib backend (see matplotlib.backends module).  Most
#             backends support 'png', 'pdf', 'ps', 'eps', and 'svg'.
#         close : boolean (default=True)
#             Whether to close the figure after saving.  If you want to save
#             the figure multiple times (e.g., to multiple formats), you
#             should NOT close it in between saves or you will have to
#             re-plot it.
#         verbose : boolean (default=True)
#             Whether to print information about when and where the image
#             has been saved.
#         """
#
#         # Extract the directory and filename from the given path
#         directory = os.path.split(path)[0]
#         filename = "%s.%s" % (os.path.split(path)[1], ext)
#         if directory == '':
#             directory = '.'
#
#         # If the directory does not exist, create it
#         if not os.path.exists(directory):
#             os.makedirs(directory)
#
#         # The final path to save to
#         savepath = os.path.join(directory, filename)
#
#         if verbose:
#             print("Saving figure to '%s'..." % savepath),
#
#         # Actually save the figure
#         plt.savefig(savepath)
#
#         # Close it
#         if close:
#             plt.close()
#
#         if verbose:
#             print("Done")