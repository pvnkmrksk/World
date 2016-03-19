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



def trajectory():
    listener()
    plt.scatter(x,y,s=1,c=t)
    # plt.draw()
    plt.pause(0.001)
    # plt.show()



plt.ion()
hl, = plt.plot([], [])

initPlot=WorldGen()
initPlot.initPositions()
initPlot.plotPositions()

plt.axis([0,255,0,255])
rospy.init_node('plot')
plt.show()
play=True
t=0
while play:
    trajectory()
    t+=1


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