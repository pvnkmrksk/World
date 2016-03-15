# system imports

from datetime import datetime
import sys, time, subprocess, os  # ROS imports
import rospy, rostopic, std_msgs.msg, rosbag
from beginner.msg import MsgFlystate, MsgTrajectory
from std_msgs.msg import String
from rospy_message_converter import message_converter

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
        self.initInput()
        self.initOutput()
        self.initFeedback()
        self.taskMgr.add(self.updateTask, "update")  # A task to run every frame, some keyboard setup and our speed


    def initParams(self):
        #
        # parameter["fps"] =parameters["fps"]
        # parameters["trajectoryUpdateInterval"] =parameters["trajectoryUpdateInterval"]
        # parameters["disabledFly"]=parameters["disabledFly"]
        # parameters["speed"] =parameters["speed"]
        # parameters["spherePath"] =parameters["spherePath"]
        # parameters["rightObjects"]=parameters["rightObjects"]
        # parameters["bagTopics"]=parameters["bagTopics"]
        # parameters["wbad"] =parameters["wbad"]
        # parameters["loadHUD"]=parameters["loadHUD"]
        # parameters["camFOV"]=parameters["camFOV"]
        # parameters["leftObjects"]=parameters["leftObjects"]
        # parameters["loadWorld"]=parameters["loadWorld"]
        # parameters["maxSpeed"] =parameters["maxSpeed"]
        # parameters["gain"] =parameters["gain"]
        # parameters["gainIncrement"] =parameters["gainIncrement"]
        # parameters["redTexPath"] =parameters["redTexPath"]
        # parameters["speedIncrement"] =parameters["speedIncrement"]
        # parameters["treeScale"] =parameters["treeScale"]
        # parameters["playerInitPos"] =parameters["playerInitPos"]
        # parameters["treePath"] =parameters["treePath"]
        # parameters["recordDur"] =parameters["recordDur"]
        # parameters["treeTexPath"] =parameters["treeTexPath"]
        # parameters["maxDistance"] =parameters["maxDistance"]
        # parameters["camHpr"]=parameters["camHpr"]
        # parameters["recordFps"] =parameters["recordFps"]
        # parameters["playerInitH"] =parameters["playerInitH"]
        # parameters["lockFps"] =parameters["lockFps"]
        # parameters["frameRecord"] =parameters["frameRecord"]
        # parameters["fly"]=parameters["fly"]
        # parameters["sphereZ"] =parameters["sphereZ"]
        # parameters["loadTrajectory"]=parameters["loadTrajectory"]
        # parameters["frameNum"] =parameters["frameNum"]
        # parameters["objectSpacing"] =parameters["objectSpacing"]
        # parameters["sphereScale"] =parameters["sphereScale"]
        # parameters["greenTexPath"] =parameters["greenTexPath"]
        # parameters["worldSize"] =parameters["worldSize"]
        # parameters["loadWind"]=parameters["loadWind"]
        # parameters["lrGain"]=parameters["lrGain"]
        # parameters["trialNo"]=parameters["trialNo"]
        # parameters["wbad"] =parameters["wbas"]
        # parameters["treeZ"] =parameters["treeZ"]


        self.posL = (parameters["worldSize"] / 2 - parameters["objectSpacing"], parameters["worldSize"]/2)
        self.posR = (parameters["worldSize"] / 2 + parameters["objectSpacing"], parameters["worldSize"] / 2)
        self.leftTreePos = Vec3(self.posL, parameters["treeZ"])
        self.rightTreePos = Vec3(self.posR, parameters["treeZ"])
        self.leftSpherePos = Vec3(self.posL, parameters["sphereZ"])
        self.rightSpherePos = Vec3(self.posR, parameters["sphereZ"])
        # self.dict2Var(parameters)
        # self.list2Exec(assertions)

        self.playerPrevPos = (parameters["playerInitPos"][0], parameters["playerInitPos"][1])
        self.camLens.setFar(parameters["maxDistance"])
        self.camLens.setFov(parameters["camFOV"])
        if parameters["lockFps"]:
            pass#parameter["fps"]=Limit(parameter["fps"])
        if parameters["frameRecord"]:
            self.record(dur=parameters["recordDur"], fps=parameters["recordFps"])
        self.fig = plt.figure()
        self.bagRecordingState = False
        self.loaderDictKeys = ("leftTree", "leftRedSphere", "leftGreenSphere",
                               "rightTree", "rightRedSphere", "rightGreenSphere")
        self.loaderDict = dict.fromkeys(self.loaderDictKeys, False)

    def initInput(self):
        self.keyboardSetup()

    def initOutput(self):
        # loadPrcFileData("", "win-size 1360 384")  # set window size
        self.initPlot()  # load the plot 1st so that the active window is panda
        loadPrcFileData("", "win-size 2720 768")  # set window size
        self.modelLoader()
        self.createEnvironment()
        self.makeLabels()

    def initFeedback(self):
        rospy.init_node('world')
        self.listener()




    # input functions
    # Key control
    def keyboardSetup(self):
        self.keyMap = {"left": 0, "right": 0, "climb": 0, "fall": 0,
                       "accelerate": 0, "decelerate": 0, "handBrake": 0, "reverse": 0,
                       "closed": 0, "gain-up": 0, "gain-down": 0, "lrGain-up": 0,
                       "lrGain-down": 0,
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





    # output functions
    def initPlot(self):
        if parameters["loadTrajectory"]:
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlim(0, parameters["worldSize"])
            self.ax.set_ylim(0, parameters["worldSize"])

            plt.plot(self.posR[0], self.posR[1], 'gs')
            plt.plot(self.posL[0], self.posL[1], 'gs')
            plt.ion()
            plt.show()

            # parametric functions constantly modified

    # models
    def modelLoader(self):

        self.loadingStringParser(parameters["leftObjects"], "left")
        self.loadingStringParser(parameters["rightObjects"], "right")

        if parameters["loadWorld"]:
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
        self.world = self.loader.loadModel("models/world" + str(parameters["worldSize"]) + ".bam")  # loads the world_size
        self.world.reparentTo(self.render)  # render the world
        # the Player
        self.player = NodePath("player")
        self.player.setPos(self.world, parameters["playerInitPos"])
        self.player.setH(self.world, parameters["playerInitH"])  # heading angle is 0

    def leftTreeLoader(self):
        self.leftTree = self.loader.loadModel(parameters["treePath"])
        self.leftTree.setPos(self.world, self.leftTreePos)
        self.leftTree.setScale(parameters["treeScale"])
        self.treeTex = self.loader.loadTexture(parameters["treeTexPath"])
        self.leftTree.setTexture(self.treeTex)
        self.leftTree.reparentTo(self.render)

    def rightTreeLoader(self):
        self.rightTree = self.loader.loadModel(parameters["treePath"])
        self.rightTree.setPos(self.world, self.rightTreePos)
        self.rightTree.setScale(parameters["treeScale"])
        self.treeTex = self.loader.loadTexture(parameters["treeTexPath"])
        self.rightTree.setTexture(self.treeTex)
        self.rightTree.reparentTo(self.render)

    def leftGreenSphereLoader(self):
        self.leftGreenSphere = self.loader.loadModel(parameters["spherePath"])
        self.leftGreenSphere.setPos(self.world, self.leftSpherePos)
        self.leftGreenSphere.setScale(parameters["sphereScale"])
        self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
        self.leftGreenSphere.setTexture(self.greenTex)
        self.leftGreenSphere.reparentTo(self.render)

    def rightGreenSphereLoader(self):
        self.rightGreenSphere = self.loader.loadModel(parameters["spherePath"])
        self.rightGreenSphere.setPos(self.world, self.rightSpherePos)
        self.rightGreenSphere.setScale(parameters["sphereScale"])
        self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
        self.rightGreenSphere.setTexture(self.greenTex)
        self.rightGreenSphere.reparentTo(self.render)

    def leftRedSphereLoader(self):
        self.leftRedSphere = self.loader.loadModel(parameters["spherePath"])
        self.leftRedSphere.setPos(self.world, self.leftSpherePos)
        self.leftRedSphere.setScale(parameters["sphereScale"])
        self.redTex = self.loader.loadTexture(parameters["redTexPath"])
        self.leftRedSphere.setTexture(self.redTex)
        self.leftRedSphere.reparentTo(self.render)

    def rightRedSphereLoader(self):
        self.rightRedSphere = self.loader.loadModel(parameters["spherePath"])
        self.rightRedSphere.setPos(self.world, self.rightSpherePos)
        self.rightRedSphere.setScale(parameters["sphereScale"])
        self.redTex = self.loader.loadTexture(parameters["redTexPath"])
        self.rightRedSphere.setTexture(self.redTex)
        self.rightRedSphere.reparentTo(self.render)

        # environment

    # sky load
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
        skysphere.setScale(parameters["maxDistance"] / 2)  # bit less than "far"
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
        self.speedLabel.setText("Speed: " + str(parameters["speed"]))
        self.gainLabel.setText("Gain: " + str(parameters["gain"]))
        self.closedLabel.setText("Closed Loop: " + str(bool(self.keyMap["closed"])))
        self.bagRecordingLabel.setText("Recording Bag: " + str(bool(self.bagRecordingState)))

    # content handlers
    def vec32String(self, vector, a, b, c):
        """returns a rounded string of vec 3 interspersed with a,b,c as headings"""
        return a + ":" + str(round(vector[0])) + " " + b + ":" + str(round(vector[1])) + " " + c + ":" + str(
            round(vector[2]))





    # feedback
    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber("/kinefly/flystate", MsgFlystate, self.callback)

    def callback(self, data):
        """ Returns Wing Beat Amplitude Difference from received data"""
        parameters["wbad"] = data.left.angles[0] - data.right.angles[0]
        parameters["wbas"] = data.left.angles[0] + data.right.angles[0]
        self.scaledWbad = parameters["lrGain"] * data.left.angles[0] - data.right.angles[0]
        if parameters["disabledFly"]:
            parameters["wbad"] = self.scaledWbad
        return parameters["wbad"]

    def publisher(self):
        data = self.message()
        trajectory = rospy.Publisher('trajectory', MsgTrajectory, queue_size=600)
        trajectory.publish(data)
        # print parameters["wbad"]

    def message(self):

        mes = MsgTrajectory()
        mes.header.stamp = rospy.Time.now()
        mes.position = self.player.getPos()
        mes.orientation = self.player.getHpr()
        mes.wbad = parameters["wbad"]
        mes.wbas = parameters["wbas"]
        mes.speed = parameters["speed"]
        mes.gain = parameters["gain"]
        mes.closed = self.keyMap["closed"]
        return mes





    # frameupdate

    def updateTask(self, task):

        self.updatePlayer()
        self.updateCamera()
        self.bagControl()

        if parameters["loadTrajectory"]:
            self.trajectory()
        if parameters["loadHUD"]:
            self.updateLabel()

        if parameters["loadWind"]:
            self.windTunnel(0)
        self.publisher()

        return Task.cont

    def updatePlayer(self):
        # global prevPos, currentPos, ax, fig, treePos, redPos
        # Global Clock by default, panda runs as fast as it can frame to frame
        scalefactor = parameters["speed"] * (globalClock.getDt())
        climbfactor = 0.1  # (.001) * scalefactor
        bankfactor = 2  # .5  * scalefactor
        speedfactor = scalefactor

        # closed loop
        if (self.keyMap["closed"] != 0):
            self.player.setH(self.player.getH() - parameters["wbad"] * parameters["gain"])

        # Climb and Fall
        if (self.keyMap["climb"] != 0):  # and parameters["speed"] > 0.00):
            # faster you go, quicker you climb
            self.player.setZ(self.player.getZ() + climbfactor)

        elif (self.keyMap["fall"] != 0):  # and parameters["speed"] > 0.00):
            self.player.setZ(self.player.getZ() - climbfactor)

        # Left and Right
        if (self.keyMap["left"] != 0):  # and parameters["speed"] > 0.0):
            self.player.setH(self.player.getH() + bankfactor)
        elif (self.keyMap["right"] != 0):  # and parameters["speed"] > 0.0):
            self.player.setH(self.player.getH() - bankfactor)

        # throttle control
        if (self.keyMap["accelerate"] != 0):
            parameters["speed"] += parameters["speedIncrement"]
            if (parameters["speed"] > parameters["maxSpeed"]):
                parameters["speed"] = parameters["maxSpeed"]
        elif (self.keyMap["decelerate"] != 0):
            parameters["speed"] -= parameters["speedIncrement"]
            if (parameters["speed"] < 0.0):
                parameters["speed"] = 0.0
        # handbrake
        if (self.keyMap["handBrake"] != 0):
            parameters["speed"] = 0

        # reverse gear
        if (self.keyMap["reverse"] != 0):
            parameters["speed"] -= parameters["speedIncrement"]

        # move forwards
        self.player.setY(self.player, speedfactor)

        # respect max camera distance else you
        # cannot see the floor post loop the loop!
        if (self.player.getZ() > parameters["maxDistance"]):
            self.player.setZ(parameters["maxDistance"])

        elif (self.player.getZ() < 0):
            self.player.setZ(0)

        # and now the X/Y world boundaries:
        if (self.player.getX() < 0):
            self.player.setX(0)
        elif (self.player.getX() > parameters["worldSize"]):
            self.player.setX(parameters["worldSize"])

        if (self.player.getY() < 0):
            self.player.setY(0)
        elif (self.player.getY() > parameters["worldSize"]):
            self.player.setY(parameters["worldSize"])

        # reset to initial position
        if (self.keyMap["init"] != 0):
            self.player.setPos(self.world, parameters["playerInitPos"])
            self.player.setH(parameters["playerInitH"])

        # update new init position
        if (self.keyMap["newInit"] != 0):
            parameters["playerInitPos"] = self.player.getPos(self.world)
            parameters["playerInitH"] = self.player.getH(self.world)
            print "new init pos is ", parameters["playerInitPos"]
            print "new init H is ", parameters["playerInitH"]

        # update gain
        if (self.keyMap["gain-up"] != 0):
            parameters["gain"] += parameters["gainIncrement"]
            print "gain is", parameters["gain"]
        elif (self.keyMap["gain-down"] != 0):
            parameters["gain"] -= parameters["gainIncrement"]
            print "gain is ", parameters["gain"]

        # update newTopSpeed
        if (self.keyMap["newTopSpeed"] != 0):
            parameters["maxSpeed"] = parameters["speed"]
            print "new max speed is", parameters["maxSpeed"]

        # update left by right gain for diabled flies
        if (self.keyMap["lrGain-up"] != 0):
            parameters["lrGain"] += parameters["gainIncrement"]
            print "lrGain is ", parameters["lrGain"]

        if (self.keyMap["lrGain-down"] != 0):
            parameters["lrGain"] -= parameters["gainIncrement"]
            print "lrGain is ", parameters["lrGain"]

    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 0, 0, 0)
        self.camera.setHpr(self.player, parameters["camHpr"])

    # recording functions
    def bagControl(self):
        if (self.keyMap["startBag"] == 1):
            self.bagger()
            self.bagRecordingState = True
            self.pickler(parameters, self.bagFilename)
        elif (self.keyMap["stopBag"] != 0):
            # self.runBagCommand.send_signal(subprocess.signal.SIGINT) #send signal on stop command
            self.terminate_ros_node("/record")
            self.bagRecordingState = False
            rospy.loginfo("Bag recording stopped")
            print self.bagFilename
            self.addMetadata()
            print "metadata added"

    def bagger(self):
        self.bagFilenameGen()
        self.bagCommand = "rosbag record --lz4 --output-name=" + self.bagFilename + " " + parameters["bagTopics"]
        # print self.bagCommand
        self.runBagCommand = subprocess.Popen(self.bagCommand, shell=True, stdout=subprocess.PIPE)
        rospy.loginfo("Bag recording started")
        time.sleep(0.15)

    def bagFilenameGen(self):
        self.timeNow = str(datetime.now().strftime('%Y-%m-%d__%H:%M:%S'))
        self.bagFilename = "bags/" + parameters["fly"] + "_" + parameters["leftObjects"] + "_" + parameters["rightObjects"] + "_gain" + str(
            parameters["gain"]) + "_speed_" + str(parameters["maxSpeed"]) + "_trial_" \
                           + str(parameters["trialNo"]) + "_" + self.timeNow
        print self.bagFilename

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def addMetadata(self):
        print "allo " + self.bagFilename
        a=self.bagFilename+".bag"
        time.sleep(5)
        print 'yay',parameters
        print(type(parameters))
        metadata=message_converter.convert_dictionary_to_ros_message("std_msgs/String",parameters)
        print metadata

        with rosbag.Bag(a,'a') as bag:
            for _,_,t in bag.read_messages():
                break
            bag.write('/metadata',metadata,t-roslib.rostime.Duration(0,1))


        # datasave

    def pickler(self, obj, path):
        """
        Pickle a Python object
        """
        with open(path, "wb") as pfile:
            pickle.dump(obj, pfile)

            # ----------------------------------------------------------------------

    def depickler(self, path):
        """
        Extracts a pickled Python object and returns it
        """
        with open(path, "rb") as pfile:
            data = pickle.load(pfile)
        return data

    # wind control
    def windTunnel(self, windDirection):
        self.servoAngle = (int(self.player.getH()) - 90) % 360 - 180
        print self.servoAngle
        servo.move(1, self.servoAngle)

        # def modelLoader(self, modelName, modelPath, modelPos, modelScale, texName, texPath, modelParent):
        #     modelName = self.loader.loadModel(modelPath)
        #     modelName.setPos(modelParent, modelPos)
        #     modelName.setScale(modelScale)
        #
        #     texName = self.loader.loadTexture(texPath)
        #     modelName.setTexture(texName)
        #     modelName.reparentTo(self.render)








    # evals
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
        if (parameters["frameNum"] % parameters["trajectoryUpdateInterval"] == 1):
            self.playerCurrentPos = (self.player.getX(), self.player.getY())
            self.verts = [(self.playerPrevPos), (self.playerCurrentPos)]
            self.codes = [Path.MOVETO, Path.LINETO]
            self.path = Path(self.verts, self.codes)

            self.patch = patches.PathPatch(self.path, facecolor='white', lw=0.5)  # ,color=hex(parameters["frameNum"]))
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
        parameters["frameNum"] += 1

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

    # testing functions not stable
    # screen capture
    def record(self, dur, fps):
        self.movie('frames/movie', dur, fps=fps, format='jpg', sd=5)

    def fpsLimit(self, fps):
        globalClock.setMode(ClockObject.MLimited)
        globalClock.setFrameRate(fps)

        # to be implemented functions fully unstable


app = MyApp()
app.run()

print 2 + 3
