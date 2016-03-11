# system imports

from datetime import datetime
import sys, time, subprocess, os  # ROS imports
import rospy, rostopic, std_msgs.msg
from beginner.msg import MsgFlystate, MsgTrajectory

from direct.showbase.ShowBase import ShowBase  # Panda imports
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog
from panda3d.core import loadPrcFileData, NodePath, TextNode
from pandac.PandaModules import CompassEffect, ClockObject
from direct.gui.OnscreenText import OnscreenText

import matplotlib.pyplot as plt  # plotting imports
from matplotlib.path import Path
import matplotlib.patches as patches
import cPickle as pickle
from params import parameters, assertions

# import servo

try:
    # Checkif rosmaster is running or not.
    rostopic.get_topic_class('/rosout')
    is_rosmaster_running = True
except rostopic.ROSTopicIOException as e:
    roscore = subprocess.Popen('roscore')  # then start roscore yourself
    time.sleep(1)  # wait a bit to be sure the roscore is really launched
    subprocess.Popen(["roslaunch", "Kinefly", "main.launch"])


class MyApp(ShowBase):
    def __init__(self):

        ShowBase.__init__(self)  # start the app
        self.initParams()  # run this 1st. Loads all content and params.

        # loadPrcFileData("", "win-size 1360 384")  # set window size
        loadPrcFileData("", "win-size 2720 768")  # set window size
        rospy.init_node('world')  # init node

        self.initPlot()  # load the plot 1st so that the active window is panda

        self.modelLoader()  # load the models
        self.listener()  # wba subscriber using ROS
        self.keyboardSetup()  # keyboard input
        self.createEnvironment()  # make sky
        self.makeLabels()  # overlays setup
        self.taskMgr.add(self.updateTask, "update")  # A task to run every frame, some keyboard setup and our speed

    # parametric functions constantly modified
    def initParams(self):
        self.dict2Var(parameters)
        self.list2Exec(assertions)

        self.playerPrevPos = (self.playerInitPos[0], self.playerInitPos[1])
        self.camLens.setFar(self.maxDistance)
        self.camLens.setFov(self.camFOV)
        if self.lockFps:
            self.fpsLimit(self.fps)
        if self.frameRecord:
            self.record(dur=self.recordDur, fps=self.recordFps)
        self.fig = plt.figure()
        self.bagRecordingState = False
        self.loaderDictKeys = ("leftTree", "leftRedSphere", "leftGreenSphere",
                               "rightTree", "rightRedSphere", "rightGreenSphere")
        self.loaderDict = dict.fromkeys(self.loaderDictKeys, False)

    def initPlot(self):
        if self.loadTrajectory:
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlim(0, self.worldSize)
            self.ax.set_ylim(0, self.worldSize)

            plt.plot(self.posR[0], self.posR[1], 'gs')
            plt.plot(self.posL[0], self.posL[1], 'gs')
            plt.ion()
            plt.show()

    # housekeeping functions written in stone
    # frameupdate
    def updatePlayer(self):
        # global prevPos, currentPos, ax, fig, treePos, redPos
        # Global Clock by default, panda runs as fast as it can frame to frame
        scalefactor = self.speed * (globalClock.getDt())
        climbfactor = 0.1  # (.001) * scalefactor
        bankfactor = 2  # .5  * scalefactor
        speedfactor = scalefactor


        # closed loop
        if (self.keyMap["closed"] != 0):
            self.player.setH(self.player.getH() - self.wbad * self.gain)

        # Climb and Fall
        if (self.keyMap["climb"] != 0):  # and self.speed > 0.00):
            # faster you go, quicker you climb
            self.player.setZ(self.player.getZ() + climbfactor)

        elif (self.keyMap["fall"] != 0):  # and self.speed > 0.00):
            self.player.setZ(self.player.getZ() - climbfactor)

        # Left and Right
        if (self.keyMap["left"] != 0):  # and self.speed > 0.0):
            self.player.setH(self.player.getH() + bankfactor)
        elif (self.keyMap["right"] != 0):  # and self.speed > 0.0):
            self.player.setH(self.player.getH() - bankfactor)

        # throttle control
        if (self.keyMap["accelerate"] != 0):
            self.speed += self.speedIncrement
            if (self.speed > self.maxSpeed):
                self.speed = self.maxSpeed
        elif (self.keyMap["decelerate"] != 0):
            self.speed -= self.speedIncrement
            if (self.speed < 0.0):
                self.speed = 0.0
        # handbrake
        if (self.keyMap["handBrake"] != 0):
            self.speed = 0

        # reverse gear
        if (self.keyMap["reverse"] != 0):
            self.speed -= self.speedIncrement

        # move forwards
        self.player.setY(self.player, speedfactor)

        # respect max camera distance else you
        # cannot see the floor post loop the loop!
        if (self.player.getZ() > self.maxDistance):
            self.player.setZ(self.maxDistance)

        elif (self.player.getZ() < 0):
            self.player.setZ(0)

        # and now the X/Y world boundaries:
        if (self.player.getX() < 0):
            self.player.setX(0)
        elif (self.player.getX() > self.worldSize):
            self.player.setX(self.worldSize)

        if (self.player.getY() < 0):
            self.player.setY(0)
        elif (self.player.getY() > self.worldSize):
            self.player.setY(self.worldSize)

        # reset to initial position
        if (self.keyMap["init"] != 0):
            self.player.setPos(self.world, self.playerInitPos)
            self.player.setH(self.playerInitH)

        # update new init position
        if (self.keyMap["newInit"] != 0):
            self.playerInitPos = self.player.getPos(self.world)
            self.playerInitH = self.player.getH(self.world)
            print "new init pos is ", self.playerInitPos
            print "new init H is ", self.playerInitH

        # update gain
        if (self.keyMap["gain-up"] != 0):
            self.gain += self.gainIncrement
            print "gain is", self.gain
        elif (self.keyMap["gain-down"] != 0):
            self.gain -= self.gainIncrement
            print "gain is ", self.gain

        # update newTopSpeed
        if (self.keyMap["newTopSpeed"] != 0):
            self.maxSpeed = self.speed
            print "new max speed is", self.maxSpeed

        #update left by right gain for diabled flies
        if (self.keyMap["lrGain-up"]!=0):
            self.lrGain+=self.gainIncrement
            print "lrGain is ",self.lrGain

        if (self.keyMap["lrGain-down"]!=0):
            self.lrGain-=self.gainIncrement
            print "lrGain is ",self.lrGain

    def updateTask(self, task):

        self.updatePlayer()
        self.updateCamera()
        self.bagControl()

        if self.loadTrajectory:
            self.trajectory()
        if self.loadHUD:
            self.updateLabel()

        if self.loadWind:
            self.windTunnel(0)
        self.publisher()

        return Task.cont

    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 0, 0, 0)
        self.camera.setHpr(self.player, self.camHpr)


    # models
    def modelLoader(self):

        self.loadingStringParser(self.leftObjects, "left")
        self.loadingStringParser(self.rightObjects, "right")

        if self.loadWorld:
            self.worldLoader()
        if self.loaderDict["leftTree"]:
            self.leftTreeLoader()
        if self.loaderDict["leftRedSphere"]:
            self.leftRedSphereLoader()
        if self.loaderDict["leftGreenSphere"]:
            self.leftGreenSphereLoader()
        if self.loaderDict["rightTree"]:
            self.rightTreeLoader()
        if self.loaderDict["rightRedSphere"]:
            self.rightRedSphereLoader()
        if self.loaderDict["rightGreenSphere"]:
            self.rightGreenSphereLoader()

    def loadingStringParser(self, loadingString, side):
        for i in loadingString:
            if i == "t":
                self.loaderDict[side + "Tree"] = True
            elif i == "r":
                self.loaderDict[side + "RedSphere"] = True
            elif i == "g":
                self.loaderDict[side + "GreenSphere"] = True

    def worldLoader(self):
        self.world = self.loader.loadModel("models/world" + str(self.worldSize) + ".bam")  # loads the world_size
        self.world.reparentTo(self.render)  # render the world
        # the Player
        self.player = NodePath("player")
        self.player.setPos(self.world, self.playerInitPos)
        self.player.setH(self.world, self.playerInitH)  # heading angle is 0

    def leftTreeLoader(self):
        self.leftTree = self.loader.loadModel(self.treePath)
        self.leftTree.setPos(self.world, self.leftTreePos)
        self.leftTree.setScale(self.treeScale)
        self.treeTex = self.loader.loadTexture(self.treeTexPath)
        self.leftTree.setTexture(self.treeTex)
        self.leftTree.reparentTo(self.render)

    def rightTreeLoader(self):
        self.rightTree = self.loader.loadModel(self.treePath)
        self.rightTree.setPos(self.world, self.rightTreePos)
        self.rightTree.setScale(self.treeScale)
        self.treeTex = self.loader.loadTexture(self.treeTexPath)
        self.rightTree.setTexture(self.treeTex)
        self.rightTree.reparentTo(self.render)

    def leftGreenSphereLoader(self):
        self.leftGreenSphere = self.loader.loadModel(self.spherePath)
        self.leftGreenSphere.setPos(self.world, self.leftSpherePos)
        self.leftGreenSphere.setScale(self.sphereScale)
        self.greenTex = self.loader.loadTexture(self.greenTexPath)
        self.leftGreenSphere.setTexture(self.greenTex)
        self.leftGreenSphere.reparentTo(self.render)

    def rightGreenSphereLoader(self):
        self.rightGreenSphere = self.loader.loadModel(self.spherePath)
        self.rightGreenSphere.setPos(self.world, self.rightSpherePos)
        self.rightGreenSphere.setScale(self.sphereScale)
        self.greenTex = self.loader.loadTexture(self.greenTexPath)
        self.rightGreenSphere.setTexture(self.greenTex)
        self.rightGreenSphere.reparentTo(self.render)

    def leftRedSphereLoader(self):
        self.leftRedSphere = self.loader.loadModel(self.spherePath)
        self.leftRedSphere.setPos(self.world, self.leftSpherePos)
        self.leftRedSphere.setScale(self.sphereScale)
        self.redTex = self.loader.loadTexture(self.redTexPath)
        self.leftRedSphere.setTexture(self.redTex)
        self.leftRedSphere.reparentTo(self.render)

    def rightRedSphereLoader(self):
        self.rightRedSphere = self.loader.loadModel(self.spherePath)
        self.rightRedSphere.setPos(self.world, self.rightSpherePos)
        self.rightRedSphere.setScale(self.sphereScale)
        self.redTex = self.loader.loadTexture(self.redTexPath)
        self.rightRedSphere.setTexture(self.redTex)
        self.rightRedSphere.reparentTo(self.render)


    # feedback
    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber("/kinefly/flystate", MsgFlystate, self.callback)

    def publisher(self):
        data = self.message()
        trajectory = rospy.Publisher('trajectory', MsgTrajectory, queue_size=600)
        trajectory.publish(data)
        # print self.wbad

    def message(self):

        mes = MsgTrajectory()
        mes.header.stamp = rospy.Time.now()
        mes.position = self.player.getPos()
        mes.orientation = self.player.getHpr()
        mes.wbad = self.wbad
        mes.wbas = self.wbas
        mes.speed = self.speed
        mes.gain = self.gain
        mes.closed = self.keyMap["closed"]
        return mes

    def callback(self, data):
        """ Returns Wing Beat Amplitude Difference from received data"""
        self.wbad = data.left.angles[0] - data.right.angles[0]
        self.wbas = data.left.angles[0] + data.right.angles[0]
        self.scaledWbad=self.lrGain*data.left.angles[0] - data.right.angles[0]
        if self.disabledFly:
            self.wbad=self.scaledWbad
        return self.wbad

    def bagFilenameGen(self):
        self.timeNow = str(datetime.now().strftime('%Y-%m-%d__%H:%M:%S'))
        self.bagFilename = "bags/"+self.fly + "_" + self.leftObjects + "_" + self.rightObjects + "_gain" + str(
                            self.gain) + "_speed_" + str(self.maxSpeed) + "_trial_" \
                           + str(self.trialNo) + "_" + self.timeNow
        print self.bagFilename

    def bagger(self):
        self.bagFilenameGen()
        self.bagCommand = "rosbag record --lz4 --output-name=" + self.bagFilename + " " + self.bagTopics
        # print self.bagCommand
        self.runBagCommand = subprocess.Popen(self.bagCommand, shell=True, stdout=subprocess.PIPE)
        rospy.loginfo("Bag recording started")
        time.sleep(0.15)

    def bagControl(self):
        if (self.keyMap["startBag"] == 1):
            self.bagger()
            self.bagRecordingState = True
            self.pickler(parameters,self.bagFilename)
        elif (self.keyMap["stopBag"] != 0):
            # self.runBagCommand.send_signal(subprocess.signal.SIGINT) #send signal on stop command
            self.terminate_ros_node("/record")
            self.bagRecordingState = False
            rospy.loginfo("Bag recording stopped")

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)


    # environment
    def createEnvironment(self):
        # Fog to hide a performance tweak:
        colour = (0.0, 0.0, 0.0)
        expfog = Fog("scene-wide-fog")
        expfog.setColor(*colour)
        expfog.setExpDensity(0.004)
        render.setFog(expfog)
        base.setBackgroundColor(*colour)

        # Our sky
        skysphere = loader.loadModel('models/sky.egg')
        skysphere.setEffect(CompassEffect.make(self.render))
        skysphere.setScale(self.maxDistance / 2)  # bit less than "far"
        skysphere.setZ(-5)
        # NOT render - you'll fly through the sky!:
        skysphere.reparentTo(self.camera)

        # Our lighting
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor(Vec4(.6, .6, .6, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(Vec3(0, -10, -10))
        directionalLight.setColor(Vec4(1, 1, 1, 1))
        directionalLight.setSpecularColor(Vec4(1, 1, 1, 1))
        render.setLight(render.attachNewNode(ambientLight))
        render.setLight(render.attachNewNode(directionalLight))

    # Key control
    def keyboardSetup(self):
        self.keyMap = {"left": 0, "right": 0, "climb": 0, "fall": 0,
                       "accelerate": 0, "decelerate": 0, "handBrake": 0, "reverse": 0,
                       "closed": 0, "gain-up": 0, "gain-down": 0,"lrGain-up":0,
                       "lrGain-down":0,
                       "init": 0, "newInit": 0, "newTopSpeed": 0, "clf": 0, "saveFig": 0,
                       "startBag": 0, "stopBag": 0}

        self.accept("escape", self.winClose)
        self.accept("a", self.setKey, ["climb", 1])
        self.accept("a-up", self.setKey, ["climb", 0])
        self.accept("z", self.setKey, ["fall", 1])
        self.accept("z-up", self.setKey, ["fall", 0])
        self.accept("arrow_left", self.setKey, ["left", 1])
        self.accept("arrow_left-up", self.setKey, ["left", 0])
        self.accept("arrow_right", self.setKey, ["right", 1])
        self.accept("arrow_right-up", self.setKey, ["right", 0])
        self.accept("arrow_down", self.setKey, ["decelerate", 1])
        self.accept("arrow_down-up", self.setKey, ["decelerate", 0])
        self.accept("arrow_up", self.setKey, ["accelerate", 1])
        self.accept("arrow_up-up", self.setKey, ["accelerate", 0])
        self.accept("o", self.setKey, ["closed", 0])
        self.accept("p", self.setKey, ["closed", 1])
        self.accept("r", self.setKey, ["reverse", 1])
        self.accept("r-up", self.setKey, ["reverse", 0])
        self.accept("s", self.setKey, ["handBrake", 1])
        self.accept("s-up", self.setKey, ["handBrake", 0])
        self.accept("i", self.setKey, ["init", 1])
        self.accept("i-up", self.setKey, ["init", 0])
        self.accept("u", self.setKey, ["gain-up", 1])
        self.accept("u-up", self.setKey, ["gain-up", 0])
        self.accept("y", self.setKey, ["gain-down", 1])
        self.accept("y-up", self.setKey, ["gain-down", 0])
        self.accept("]", self.setKey, ["newInit", 1])
        self.accept("]-up", self.setKey, ["newInit", 0])
        self.accept("q", self.setKey, ["clf", 1])
        self.accept("q-up", self.setKey, ["clf", 0])
        self.accept("w", self.setKey, ["saveFig", 1])
        self.accept("w-up", self.setKey, ["saveFig", 0])
        self.accept("e", self.setKey, ["startBag", 1])
        self.accept("e-up", self.setKey, ["startBag", 0])
        self.accept("d", self.setKey, ["stopBag", 1])
        self.accept("d-up", self.setKey, ["stopBag", 0])
        self.accept("t", self.setKey, ["newTopSpeed", 1])
        self.accept("t-up", self.setKey, ["newTopSpeed", 0])
        self.accept("v", self.setKey, ["lrGain-down", 1])
        self.accept("v-up", self.setKey, ["lrGain-down", 0])
        self.accept("b", self.setKey, ["lrGain-up", 1])
        self.accept("b-up", self.setKey, ["lrGain-up", 0])

        base.disableMouse()  # or updateCamera will fail!

    def setKey(self, key, value):
        self.keyMap[key] = value

    def winClose(self):
        self.closeWindow(self.win)

    # labels
    def makeStatusLabel(self, i):
        return OnscreenText(style=2, fg=(0, 0, 0, 0.5), bg=(0.4, 0.4, 0.4, 0.8),
                            scale=0.05, pos=(-3.1, 0.92 - (.08 * i)), mayChange=1)

    def makeLabels(self):
        self.positionLabel = self.makeStatusLabel(0)
        self.orientationLabel = self.makeStatusLabel(1)
        self.speedLabel = self.makeStatusLabel(2)
        self.gainLabel = self.makeStatusLabel(3)
        self.closedLabel = self.makeStatusLabel(4)
        self.bagRecordingLabel = self.makeStatusLabel(5)

    def updateLabel(self):
        self.positionLabel.setText(self.vec32String(self.player.getPos(), "x", "y", "z"))
        self.orientationLabel.setText(self.vec32String(self.player.getHpr(), "H", "P", "R"))
        self.speedLabel.setText("Speed: " + str(self.speed))
        self.gainLabel.setText("Gain: " + str(self.gain))
        self.closedLabel.setText("Closed Loop: " + str(bool(self.keyMap["closed"])))
        self.bagRecordingLabel.setText("Recording Bag: " + str(bool(self.bagRecordingState)))

    # content handlers
    def vec32String(self, vector, a, b, c):
        """returns a rounded string of vec 3 interspersed with a,b,c as headings"""
        return a + ":" + str(round(vector[0])) + " " + b + ":" + str(round(vector[1])) + " " + c + ":" + str(
            round(vector[2]))

    def dict2Var(self, dict):
        """converts a dict to a variable using exec for assignment
            use it with caution"""
        for key, val in dict.items():
            exec (key + '=val')

    def list2Exec(self, list):
        """
        Converts the list items to eval statement
        """
        for key in list:
            exec (key)

    # trajectory
    def trajectory(self):
        if (self.frameNum % self.trajectoryUpdateInterval == 1):
            self.playerCurrentPos = (self.player.getX(), self.player.getY())
            self.verts = [(self.playerPrevPos), (self.playerCurrentPos)]
            self.codes = [Path.MOVETO, Path.LINETO]
            self.path = Path(self.verts, self.codes)

            self.patch = patches.PathPatch(self.path, facecolor='white', lw=0.5)  # ,color=hex(self.frameNum))
            self.ax.add_patch(self.patch)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.playerPrevPos = self.playerCurrentPos

        if (self.keyMap["clf"] != 0):
            plt.clf()
            self.initPlot()

        if (self.keyMap["saveFig"] != 0):
            self.bagFilenameGen()
            self.save("trajectory" + self.bagFilename, ext="png", close=False, verbose=True)
            self.save("trajectory" + self.bagFilename, ext="svg", close=False, verbose=True)
            # pickle.dump(self.fig,file(self.bagFilename+"fig",'w'))
        self.frameNum += 1

    def save(self, path, ext='png', close=True, verbose=True):
        """Save a figure from pyplot.
        Parameters
        ----------
        path : string
            The path (and filename, without the extension) to save the
            figure to.
        ext : string (default='png')
            The file extension. This must be supported by the active
            matplotlib backend (see matplotlib.backends module).  Most
            backends support 'png', 'pdf', 'ps', 'eps', and 'svg'.
        close : boolean (default=True)
            Whether to close the figure after saving.  If you want to save
            the figure multiple times (e.g., to multiple formats), you
            should NOT close it in between saves or you will have to
            re-plot it.
        verbose : boolean (default=True)
            Whether to print information about when and where the image
            has been saved.
        """

        # Extract the directory and filename from the given path
        directory = os.path.split(path)[0]
        filename = "%s.%s" % (os.path.split(path)[1], ext)
        if directory == '':
            directory = '.'

        # If the directory does not exist, create it
        if not os.path.exists(directory):
            os.makedirs(directory)

        # The final path to save to
        savepath = os.path.join(directory, filename)

        if verbose:
            print("Saving figure to '%s'..." % savepath),

        # Actually save the figure
        plt.savefig(savepath)

        # Close it
        if close:
            plt.close()

        if verbose:
            print("Done")

    def pickler(self,obj, path):
        """
        Pickle a Python object
        """
        with open(path, "wb") as pfile:
            pickle.dump(obj, pfile)

#----------------------------------------------------------------------
    def depickler(self,path):
        """
        Extracts a pickled Python object and returns it
        """
        with open(path, "rb") as pfile:
            data = pickle.load(pfile)
        return data

    # testing functions not stable
    # screen capture
    def record(self, dur, fps):
        self.movie('frames/movie', dur, fps=fps, format='jpg', sd=5)

    def fpsLimit(self, fps):
        globalClock.setMode(ClockObject.MLimited)
        globalClock.setFrameRate(fps)

    # to be implemented functions fully unstable
    def windTunnel(self, windDirection):
        self.servoAngle = (int(self.player.getH()) -90)% 360 -180
        print self.servoAngle
        servo.move(1,self.servoAngle)

        # def modelLoader(self, modelName, modelPath, modelPos, modelScale, texName, texPath, modelParent):
        #     modelName = self.loader.loadModel(modelPath)
        #     modelName.setPos(modelParent, modelPos)
        #     modelName.setScale(modelScale)
        #
        #     texName = self.loader.loadTexture(texPath)
        #     modelName.setTexture(texName)
        #     modelName.reparentTo(self.render)


app = MyApp()
app.run()

print 2 + 3
