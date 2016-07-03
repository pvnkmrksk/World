# system imports
from __future__ import division
from datetime import datetime
import sys, time, subprocess, os, serial  # ROS imports
import json_tricks as json
import rospy, rostopic, roslib, std_msgs.msg, rosbag
from beginner.msg import MsgFlystate, MsgTrajectory
from std_msgs.msg import String
from rospy_message_converter import message_converter

from direct.showbase.ShowBase import ShowBase  # Panda imports
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog, Camera, PerspectiveLens
from panda3d.core import loadPrcFileData, NodePath, TextNode
from pandac.PandaModules import CompassEffect, ClockObject
from direct.gui.OnscreenText import OnscreenText

import matplotlib.pyplot as plt  # plotting imports
from matplotlib.path import Path
import matplotlib.patches as patches
import cPickle as pickle
import random
import numpy as np
from params import parameters

# If replay world for playback old data, copy existing params to prevent overwrite from the saved state of previous run
if parameters["replayWorld"]:

    # save params from current instance as backup toload back into the playback file
    replay = parameters["replayWorld"]
    scale = parameters["captureScale"]
    start = parameters["captureStart"]
    increment = parameters["playbackIncrement"]

    import easygui
    import pandas as pd

    # replayPath = easygui.fileopenbox(multiple=False, filetypes=["*.pickle"])
    replayPath = "/home/behaviour/catkin/src/beginner/scripts/panda/world/bags/fly4/fly4_quad_rg_gain7.0_speed_3.5_" \
                 "trial_1_2016-04-13__23:31:35.bag_df.pickle"

    print replayPath
    df = pd.read_pickle(replayPath)

    # slice pos and orientation and remove nans
    traj = df.loc[:, "trajectory__orientation_x":"trajectory__position_z"].dropna()
    cols = traj.columns.tolist()  # list of colums
    cols = cols[-3:] + cols[:-3]  # reorder colums. hprpos to poshpr by python splicing
    traj = traj[cols]  # reassign with this order

    # change current parameters to parameters saved in dataframe
    parameters = None

    try:
        parameters = json.loads(df.metadata__data.values[1])

    except:
        parameters = json.loads(df.metadata__data.values[0])
        print "using exceprion"

    # dump back params from vars
    parameters["replayWorld"] = replay
    parameters["captureScale"] = scale
    parameters["captureStart"] = start
    parameters["playbackIncrement"] = increment

#check if arduino servo is connected
try:
    import servo
except serial.serialutil.SerialException:
    parameters["loadWind"] = False
    print ("\n \n \n servo disabled \n \n \n")

# Checkif rosmaster is running else run roscore
try:
    rostopic.get_topic_class('/rosout')
    is_rosmaster_running = True
except rostopic.ROSTopicIOException as e:
    roscore = subprocess.Popen('roscore')  # then start roscore yourself
    time.sleep(1)  # wait a bit to be sure the roscore is really launched
    subprocess.Popen(["roslaunch", "Kinefly", "main.launch"])

#World class definition
class MyApp(ShowBase):

    #init windows size, params, I/O, feedback
    def __init__(self):

        loadPrcFileData("", "win-size " + str(parameters["windowWidth"] / parameters["captureScale"]) + " " +
                        str(parameters["windowHeight"] / parameters["captureScale"]))  # set window size

        # loadPrcFileData('', 'fullscreen true')

        ShowBase.__init__(self)  # start the app
        base.setFrameRateMeter(True)

        self.initParams()  # run this 1st. Loads all content and params.
        self.initInput()
        self.initOutput()
        self.initFeedback()
        self.taskMgr.add(self.updateTask, "update")  # A task to run every frame, some keyboard setup and our speed

    #
    def initParams(self):
        '''
        initializes camera
        initiales screen capture
        generates init positions lists
        generates object positions
        init param values

        Returns:
            None
        '''


        if parameters["lockFps"]:
            print "fps locked"
            self.fpsLimit(165)

        if parameters["frameRecord"]:
            self.record(dur=parameters["recordDur"], fps=parameters["recordFps"])

        # offset for position displacemtn and boundary being 2^n+1,
        parameters["offset"] = ((int(parameters["worldSize"]) - 1) / 2) + 1
        print "offset is ", parameters["offset"]

        #initial position of player pushed around the 4 quadrants with edge effect correction
        parameters["initPosList"] = [(parameters["playerInitPos"][0] + parameters["offset"],
                                      parameters["playerInitPos"][1] + parameters["offset"],
                                      parameters["playerInitPos"][2]),
                                     (parameters["playerInitPos"][0],
                                      parameters["playerInitPos"][1] + parameters["offset"],
                                      parameters["playerInitPos"][2]),
                                     (parameters["playerInitPos"]),
                                     (parameters["playerInitPos"][0] + parameters["offset"],
                                      parameters["playerInitPos"][1], parameters["playerInitPos"][2])]
        print "init pos list", parameters["initPosList"]


        #position of objects generated
        self.odd, self.even, quad = self.quadPositionGenerator(posL=parameters["posL"], posR=parameters["posR"])

        self.servoAngle = None                                      #
        self.bagRecordingState = False
        self.decayTime = -1
        self.boutFrame = 0
        self.lastResetTime = datetime.now()
        self.frame = parameters["captureStart"]

        self.quadSet = set(range(0, 4))
        self.quadSetCopy = self.quadSet.copy()

    def initInput(self):
        '''
        initializes use input via keyboard
        Returns:

        '''
        self.keyboardSetup()

    def initOutput(self):
        '''
        initilizes plotting mechanism
        inits models, world and labels
        inits the wind field
        Returns:
            None

        '''

        self.initPlot()  # load the plot 1st so that the active window is panda

        # loadPrcFileData("", "win-size " + str(parameters["windowWidth"]) + " " + str(
        #     parameters["windowHeight"]))  # set window size
        self.modelLoader()
        self.initDisplayRegion()

        self.createEnvironment()
        self.makeLabels()
        self.windFieldGen()

    def initFeedback(self):
        '''
        initializes the ros nodes
        initilizes the listener node for closing the loop
        Returns:
            None

        '''
        rospy.init_node('world')
        self.listener()

    # input functions
    def keyboardSetup(self):
        '''
        Setup the keybindings to actions

        Returns:
            None

        '''
        self.keyMap = {"left": 0, "right": 0, "climb": 0, "fall": 0,
                       "accelerate": 0, "decelerate": 0, "handBrake": 0, "reverse": 0,
                       "closed": 0, "gain-up": 0, "gain-down": 0, "lrGain-up": 0,
                       "lrGain-down": 0,
                       "init": 0, "newInit": 0, "newTopSpeed": 0, "clf": 0, "saveFig": 0,
                       "startBag": 0, "stopBag": 0, "quad1": 0, "quad2": 0, "quad3": 0, "quad4": 0, "human": 0,
                       "hRight": 0}

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
        self.accept("1", self.setKey, ["quad1", 1])
        self.accept("1-up", self.setKey, ["quad1", 0])
        self.accept("2", self.setKey, ["quad2", 1])
        self.accept("2-up", self.setKey, ["quad2", 0])
        self.accept("3", self.setKey, ["quad3", 1])
        self.accept("3-up", self.setKey, ["quad3", 0])
        self.accept("4", self.setKey, ["quad4", 1])
        self.accept("4-up", self.setKey, ["quad4", 0])
        self.accept("8", self.setKey, ["human", 1])
        # self.accept("8-up",self.setKey,["human",0])
        self.accept("5", self.setKey, ["hRight", 1])
        self.accept("5-up", self.setKey, ["hRight", -1])

        base.disableMouse()  # or updateCamera will fail!

    def setKey(self, key, value):
        '''
        maps the key to value
        Args:
            key: keyboard key name
            value: parameter to update

        Returns:

        '''
        self.keyMap[key] = value

    def winClose(self):
        # self.closeWindow(self.win)
        self.plotter.kill()
        sys.exit()

    # output functions
    def initPlot(self):
        if parameters["loadTrajectory"]:
            self.plotter = subprocess.Popen(["python", "realTimePlotter.py"])
            print "\n \n \n realtime plotter started \n \n \n"
            time.sleep(1)

    # models
    def modelLoader(self):
        if parameters["loadWorld"]:
            self.worldLoader()

    def worldLoader(self):
        # global plotter
        self.worldFilename = "models/world_" + "size:" + parameters["modelSizeSuffix"] + "_obj:" \
                             + parameters["loadingString"] + "_num:" + str(parameters["widthObjects"]) \
                             + "x" + str(parameters["heightObjects"]) + "_lattice:" \
                             + str(parameters["lattice"]) + ".bam"

        print "model file exists:", os.path.isfile(self.worldFilename)

        print "open worldgen?", (not os.path.isfile(self.worldFilename)) or parameters["generateWorld"]
        print "\n \n \n"

        if ((not os.path.isfile(self.worldFilename)) or parameters["generateWorld"]):
            subprocess.Popen(["python", "worldGen.py"])
            time.sleep(3)

        self.world = self.loader.loadModel(self.worldFilename)  # loads the world_size
        self.world.reparentTo(self.render)  # render the world
        # the Player
        self.player = NodePath("player")
        self.player.setPos(self.world, tuple(parameters["playerInitPos"]))
        self.player.setH(self.world, (parameters["playerInitH"]))  # heading angle is 0

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
        skysphere.setScale(parameters["maxDistance"])  # bit less than "far"
        skysphere.setZ(-3)
        # NOT render - you'll fly through the sky!:
        skysphere.reparentTo(self.cameraCenter)

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
        return OnscreenText(style=2, fg=(0, 0, 0, 0.12), bg=(0.4, 0.4, 0.4, 0.18),
                            scale=0.04, pos=(-3.1, 0.92 - (.04 * i)), mayChange=1)

    def makeLabels(self):
        self.positionLabel = self.makeStatusLabel(0)
        self.orientationLabel = self.makeStatusLabel(1)
        self.speedLabel = self.makeStatusLabel(2)
        self.gainLabel = self.makeStatusLabel(3)
        self.servoLabel = self.makeStatusLabel(4)
        self.closedLabel = self.makeStatusLabel(5)
        self.bagRecordingLabel = self.makeStatusLabel(6)

    def updateLabel(self):
        self.positionLabel.setText(self.vec32String(self.player.getPos(), "x", "y", "z"))
        self.orientationLabel.setText(self.vec32String(self.player.getHpr(), "H", "P", "R"))
        self.speedLabel.setText("Speed: " + str(parameters["speed"]))
        self.gainLabel.setText("Gain: " + str(parameters["gain"]))

        self.servoLabel.setText("Servo Angle: " + str(self.servoAngle))
        self.closedLabel.setText("Closed Loop: " + str(bool(self.keyMap["closed"])))
        self.bagRecordingLabel.setText("Recording Bag: " + str(bool(self.bagRecordingState)))

    # content handlers
    def vec32String(self, vector, a, b, c):
        """returns a rounded string of vec 3 interspersed with a,b,c as headings"""
        return a + ":" + str(round(vector[0])) + " " + b + ":" + str(round(vector[1])) + " " + c + ":" + str(
            round(vector[2]))

    # display regions
    def initDisplayRegion(self):

        dr = base.camNode.getDisplayRegion(0)
        dr.setActive(0)

        lens = PerspectiveLens(120, 140)  # tuple(parameters["camFOV"]))

        displayLeft = self.win.makeDisplayRegion(0, 1 / 3, 0, 1)
        camL = Camera('Lcam')
        camL.setLens(lens)
        self.cameraLeft = self.render.attach_new_node(camL)
        displayLeft.setCamera(self.cameraLeft)

        displayCenter = self.win.makeDisplayRegion(1 / 3, 2 / 3, 0, 1)
        camC = Camera('Ccam')
        camC.setLens(lens)
        self.cameraCenter = self.render.attach_new_node(camC)
        displayCenter.setCamera(self.cameraCenter)

        displayRight = self.win.makeDisplayRegion(2 / 3, 1, 0, 1)
        camR = Camera('Rcam')
        camR.setLens(lens)
        self.cameraRight = self.render.attach_new_node(camR)
        displayRight.setCamera(self.cameraRight)

        self.cameraLeft.setPos(self.player, 0, 0, 0)
        self.cameraLeft.setHpr(self.player, tuple(parameters["camHpr"]))

        self.cameraCenter.setPos(self.player, 0, 0, 0)
        self.cameraCenter.setHpr(self.player, tuple(parameters["camHpr"]))

        self.cameraRight.setPos(self.player, 0, 0, 0)
        self.cameraRight.setHpr(self.player, tuple(parameters["camHpr"]))

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

    def publisher(self, data):
        trajectory = rospy.Publisher('trajectory', MsgTrajectory, queue_size=600)
        trajectory.publish(data)

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
        mes.reset = False
        return mes

    # frameupdate
    def updateTask(self, task):

        self.updatePlayer()
        self.updateCamera()
        self.bagControl()
        #
        # if parameters["loadHUD"]:
        #     self.updateLabel()

        if parameters["loadWind"]:
            x, y, z = self.player.getPos()
            windDir = parameters["windField"][x, y]
            self.windTunnel(windDir)

            # self.windTunnel(parameters["windDirection"])

        self.publisher(self.message())

        return Task.cont

    def updatePlayer(self):
        if parameters["replayWorld"]:

            try:
                poshpr = traj.ix[self.frame, :].values
                print "frame is", self.frame
            except IndexError:
                print "Finished playback"
                self.winClose()
            # print poshpr
            self.player.setPosHpr(tuple(poshpr[0:3]), tuple(poshpr[3:]))
            # self.player.setPosHpr(traj.ix[self.frame,:].values)

            self.frame += parameters["playbackIncrement"]
        else:


            # global prevPos, currentPos, ax, fig, treePos, redPos Global Clock by default, panda runs as fast as it can frame to frame
            scalefactor = parameters["speed"] * (globalClock.getDt())
            climbfactor = 0.01  # (.001) * scalefactor
            bankfactor = 2  # .5  * scalefactor
            speedfactor = scalefactor

            # closed loop
            if (self.keyMap["closed"] != 0):
                self.player.setH(self.player.getH() - parameters["wbad"] * parameters["gain"])

            if (self.keyMap["human"] != 0):
                self.player.setH(self.player.getH() + self.keyMap["hRight"] * parameters["gain"])

            # Climb and Fall
            if (self.keyMap["climb"] != 0):  # and parameters["speed"] > 0.00):
                # faster you go, quicker you climb
                self.player.setZ(self.player.getZ() + climbfactor)
                print "z is ", self.player.getZ()

            elif (self.keyMap["fall"] != 0):  # and parameters["speed"] > 0.00):
                self.player.setZ(self.player.getZ() - climbfactor)
                print "z is ", self.player.getZ()

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

            # respect max camera distance else you cannot see the floor post loop the loop!
            if (self.player.getZ() > parameters["maxDistance"]):
                self.player.setZ(parameters["maxDistance"])

            elif (self.player.getZ() < 0):
                self.player.setZ(0)

            # and now the X/Y world boundaries:
            if (self.player.getX() < 0):
                if parameters["quad"]:
                    self.resetPosition("rand")
                else:
                    self.player.setX(0)

            elif (self.player.getX() > parameters["worldSize"]):
                if parameters["quad"]:
                    self.resetPosition("rand")
                else:
                    self.player.setX(parameters["worldSize"])

            if (self.player.getY() < 0):
                if parameters["quad"]:
                    self.resetPosition("rand")
                else:
                    self.player.setY(0)

            elif (self.player.getY() > parameters["worldSize"]):
                if parameters["quad"]:
                    self.resetPosition("rand")
                else:
                    self.player.setY(parameters["worldSize"])

            # reset to initial position
            if (self.keyMap["init"] != 0):
                self.resetPosition("rand")
                time.sleep(0.1)
                # self.player.setPos(self.world, parameters["playerInitPos"])
                # self.player.setH(parameters["playerInitH"])

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

            # respect quad boundary
            if parameters["quad"]:
                # print "x is",self.player.getX
                # print " offsetis",parameters["offset"]
                # x=self.player.getX()
                # y=self.player.getY()
                if (self.player.getX() > parameters["offset"] and self.player.getX() < (parameters["offset"] + 1)):
                    self.resetPosition("rand")
                if (self.player.getY() > parameters["offset"] and self.player.getY() < (parameters["offset"] + 1)):
                    self.resetPosition("rand")

            if self.decayTime > 60:
                parameters["speed"] = 0
                self.keyMap["closed"] = 0
                self.decayTime -= 1
            elif 0 < self.decayTime <= 60:

                self.keyMap["closed"] = self.closedMemory
                self.decayTime -= 1

            elif self.decayTime == 0:
                parameters["speed"] = self.speedMemory
                self.decayTime -= 1

            if self.reachedDestination():
                self.resetPosition("rand")

            # reset position by user input
            for i in range(4):
                if (self.keyMap["quad" + str(i + 1)] != 0):
                    self.resetPosition(i + 1)
                    time.sleep(0.15)

            self.tooLongBoutReset()

    def tooLongBoutReset(self):
        if self.boutFrame > parameters["maxBoutDur"]:
            self.resetPosition("rand")
            print "bout longer than max duration", parameters["maxBoutDur"]
        else:
            self.boutFrame += 1

    def reachedDestination(self):
        oddeven = np.append(self.odd, self.even, axis=0)
        for i in (oddeven):

            if self.isInsideTarget(i):
                # mes=MsgTrajectory()
                # mes.reset=True
                # self.publisher(mes)
                return True
                break

    def quadPositionGenerator(self, posL, posR):

        offset = (int(parameters["worldSize"]) - 1) / 2

        quad3PosL = posL
        quad3PosR = posR

        quad4PosL = (posL[0] + offset, posL[1])
        quad4PosR = (posR[0] + offset, posR[1])

        quad2PosL = (posL[0], posL[1] + offset)
        quad2PosR = (posR[0], posR[1] + offset)

        quad1PosL = (posL[0] + offset, posL[1] + offset)
        quad1PosR = (posR[0] + offset, posR[1] + offset)

        odd = np.array([quad1PosR, quad2PosL, quad3PosL, quad3PosR])
        even = np.array([quad1PosL, quad2PosR, quad4PosL, quad4PosR])
        quad = np.array(
            [[quad1PosL, quad1PosR], [quad2PosL, quad2PosR], [quad3PosL, quad3PosR], [quad4PosL, quad4PosR]])
        # print offset
        # print "even is ",odd
        # print "even is ", even
        return odd, even, quad

    def isInsideTarget(self, target):
        tl, br = self.boundingBoxCoordinates(target, parameters["bboxDist"])
        x, y, z = self.player.getPos()
        if x > tl[0] and x < br[0] and y < tl[1] and y > br[1]:
            return True
        else:
            return False

    def boundingBoxCoordinates(self, target, distance):
        """
        Args:

            obj:the position of object whose bound box has to be found
            distance: the half width of the box | pseudo radius

        Returns:
            tl: top left coordinate.
            br: bottom right coordinate
        """
        tl = (target[0] - distance, target[1] + distance)
        br = (target[0] + distance, target[1] - distance)

        return tl, br

    def resetPosition(self, quad):

        if quad == "rand":
            self.randIndex()
            newPos = parameters["initPosList"][self.index]
            print "random quadrant is ", self.index + 1, "\n"

            # index = random.randrange(len(parameters["initPosList"]))
            # newPos = parameters["initPosList"][index]
            # print "random quadrant is ", index + 1, "\n"

        else:
            newPos = parameters["initPosList"][quad - 1]
            print "Your quadrant is", (quad), "\n"

        self.player.setPos(newPos)
        self.player.setH(parameters["playerInitH"])

        self.decayTime = 240
        self.speedMemory = parameters["speed"]
        self.closedMemory = self.keyMap["closed"]
        print "newPos is", newPos, "\n"

        print "quadrant duration was ", str((datetime.now() - self.lastResetTime).total_seconds())
        print "\n \n \n"

        self.lastResetTime = datetime.now()

        self.boutFrame = 0

        mes = MsgTrajectory()
        mes.reset = True
        self.publisher(mes)
        return newPos

    def randChoice(self):
        self.index = random.choice(list(self.quadSet))
        self.quadSet.remove(self.index)

        print "index is", self.index
        print "set is", self.quadSet

    def randIndex(self):
        if len(self.quadSet) > 0:
            self.randChoice()

        else:
            self.quadSet = self.quadSetCopy.copy()
            self.randChoice()

    def updateCamera(self):
        # see issue content for how we calculated these:
        #
        # if parameters["replayWorld"]:
        #     self.camera.setPos()

        # self.camera.setPos(self.player, 0, 0, 0)
        # self.camera.setHpr(self.player, tuple(parameters["camHpr"]))




        self.cameraLeft.setPos(self.player, 0, 0, 0)
        self.cameraLeft.setH(self.player, 120)  # self.player.getH())#+120)
        #
        self.cameraCenter.setPos(self.player, 0, 0, 0)
        self.cameraCenter.setHpr(self.player, tuple(parameters["camHpr"]))  # (0,-2,0))# self.world, self.player.getH())

        self.cameraRight.setPos(self.player, 0, 0, 0)
        self.cameraRight.setH(self.player, 240)  # self.world, self.player.getH())#-120)

    # recording functions
    def bagControl(self):
        if (self.keyMap["startBag"] == 1):
            self.bagger()
            self.bagRecordingState = True
            file = open(__file__, 'r')
            obj = [file.read(), parameters]
            self.pickler(obj, self.bagFilename)

        elif (self.keyMap["stopBag"] != 0):
            # self.runBagCommand.send_signal(subprocess.signal.SIGINT) #send signal on stop command
            self.terminate_ros_node("/record")
            self.bagRecordingState = False
            rospy.loginfo("\n \n \n Bag recording stopped \n \n \n ")
            self.addMetadata()
            print "metadata added \n \n "
            print "\n \n bagfilename is", self.bagFilename

    def bagger(self):
        self.bagFilenameGen()
        self.bagCommand = "rosbag record --lz4 --output-name=" + self.bagFilename + " " \
                          + parameters["bagTopics"]
        # print self.bagCommand
        self.runBagCommand = subprocess.Popen(self.bagCommand, shell=True, stdout=subprocess.PIPE)
        rospy.loginfo("Bag recording started")
        time.sleep(0.15)

    def bagFilenameGen(self):
        self.timeNow = str(datetime.now().strftime('%Y-%m-%d__%H:%M:%S'))
        if parameters["hcp"]:
            mode = "hcp_"
        elif parameters["quad"]:
            if parameters["loadWind"]:
                mode = "wind_quad_"
            else:
                mode = "quad_"

        self.bagFilename = "bags/" + parameters["fly"] + "_" + mode + parameters["loadingString"] \
                           + "_gain" + str(parameters["gain"]) + "_speed_" + str(parameters["maxSpeed"]) \
                           + "_trial_" + str(parameters["trialNo"]) + "_" + self.timeNow

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
        a = self.bagFilename + ".bag"
        time.sleep(5)  # so that bag file can be transfereed from memory

        metadata = String(json.dumps(parameters))
        print "metadata is:", metadata

        with rosbag.Bag(a, 'a') as bag:
            i = 0
            for _, _, t in bag.read_messages():
                if i == 0:
                    tstamp = t
                i += 1
                break
            bag.write('/metadata', metadata, tstamp)


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
        if windDirection != -1:  # -1 is open loop in wind direction
            self.servoAngle = int((90 - (self.player.getH()) + windDirection - 180) % 360)
        else:
            self.servoAngle = 90
            print "wind in open loop"

        # print "servoangle is", self.servoAngle
        servo.move(1, self.servoAngle)

    def windFieldGen(self):
        self.windField = np.zeros([parameters["worldSize"], parameters["worldSize"]])

        offset = (parameters["worldSize"] - 1) / 2
        world = parameters["worldSize"]
        self.windField[0:offset, 0:offset] = parameters["windQuad"][2]
        self.windField[offset + 1:world, 0:offset] = parameters["windQuad"][3]
        self.windField[0:offset, offset + 1:world] = parameters["windQuad"][1]
        self.windField[offset + 1:world, offset + 1:world] = parameters["windQuad"][0]
        parameters["windField"] = self.windField

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

    # testing functions not stable
    # screen capture
    def record(self, dur, fps):
        self.movie('frames/movie', dur, fps=fps, format='jpg', sd=7)

    def fpsLimit(self, fps):
        globalClock = ClockObject.getGlobalClock()
        globalClock.setMode(ClockObject.MLimited)
        globalClock.setFrameRate(fps)

        # to be implemented functions fully unstable


app = MyApp()
app.run()

print 2 + 3
