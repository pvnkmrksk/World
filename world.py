# system imports

from datetime import datetime
import sys, time, subprocess, os ,json # ROS imports
import rospy, rostopic, roslib,std_msgs.msg, rosbag
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
import random
import numpy as np

from params import parameters


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

        self.camLens.setFar(parameters["maxDistance"])
        self.camLens.setFov(parameters["camFOV"])

        if parameters["lockFps"]:
            pass#parameter["fps"]=Limit(parameter["fps"])
        if parameters["frameRecord"]:
            self.record(dur=parameters["recordDur"], fps=parameters["recordFps"])

        parameters["offset"]=((int(parameters["worldSize"])-1)/2)+1
        print "offset is ", parameters["offset"]

        parameters["initPosList"]=[parameters["playerInitPos"],
                                   (parameters["playerInitPos"][0]+parameters["offset"],parameters["playerInitPos"][1],parameters["playerInitPos"][2]),
                                  (parameters["playerInitPos"][0],parameters["playerInitPos"][1]+parameters["offset"],parameters["playerInitPos"][2]),
        (parameters["playerInitPos"][0]+parameters["offset"],parameters["playerInitPos"][1]+parameters["offset"],parameters["playerInitPos"][2])
                                   ]
        print "init pos list", parameters["initPosList"]
        self.odd,self.even = self.quadPositionGenerator()


        self.bagRecordingState = False
        self.decayTime=-1

    def initInput(self):
        self.keyboardSetup()

    def initOutput(self):
        self.initPlot()  # load the plot 1st so that the active window is panda
        loadPrcFileData("", "win-size "+str(parameters["windowWidth"])+" "+str(parameters["windowHeight"]))  # set window size
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
        # self.closeWindow(self.win)
        self.plotter.kill()
        sys.exit()


    # output functions
    def initPlot(self):
        if parameters["loadTrajectory"]:
            self.plotter=subprocess.Popen(["python", "realTimePlotter.py"])
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
                             + "x" + str(parameters["heightObjects"])+"_lattice:"\
                             +str(parameters["lattice"]) + ".bam"

        print "model file exists:",os.path.isfile(self.worldFilename)

        print "open worldgen?",(not os.path.isfile(self.worldFilename)) or parameters["generateWorld"]
        print "\n \n \n"

        if ((not os.path.isfile(self.worldFilename)) or parameters["generateWorld"]):
            subprocess.Popen(["python", "hcpWorldGen.py"])
            time.sleep(3)


        self.world = self.loader.loadModel(self.worldFilename)  # loads the world_size
        self.world.reparentTo(self.render)  # render the world
        # the Player
        self.player = NodePath("player")
        self.player.setPos(self.world, parameters["playerInitPos"])
        self.player.setH(self.world, parameters["playerInitH"])  # heading angle is 0


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
        skysphere.setScale(parameters["maxDistance"] )  # bit less than "far"
        skysphere.setZ(-3)
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
        return OnscreenText(style=2, fg=(0, 0, 0, 0.12), bg=(0.4, 0.4, 0.4, 0.18),
                            scale=0.04, pos=(-3.1, 0.92 - (.04 * i)), mayChange=1)

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

    def publisher(self,data):
        # data = self.message()
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
        mes.reset=False
        return mes





    # frameupdate

    def updateTask(self, task):

        self.updatePlayer()
        self.updateCamera()
        self.bagControl()
        # print data.position
        #
        # if parameters["loadTrajectory"]:
        #     self.trajectory()
        if parameters["loadHUD"]:
            self.updateLabel()

        if parameters["loadWind"]:
            self.windTunnel(0)
        self.publisher(self.message())

        return Task.cont

    def updatePlayer(self):
        # global prevPos, currentPos, ax, fig, treePos, redPos
        # Global Clock by default, panda runs as fast as it can frame to frame
        scalefactor = parameters["speed"] * (globalClock.getDt())
        climbfactor = 0.001  # (.001) * scalefactor
        bankfactor = 2  # .5  * scalefactor
        speedfactor = scalefactor

        # closed loop
        if (self.keyMap["closed"] != 0):
            self.player.setH(self.player.getH() - parameters["wbad"] * parameters["gain"])

        # Climb and Fall
        if (self.keyMap["climb"] != 0):  # and parameters["speed"] > 0.00):
            # faster you go, quicker you climb
            self.player.setZ(self.player.getZ() + climbfactor)
            print "z is ",self.player.getZ()

        elif (self.keyMap["fall"] != 0):  # and parameters["speed"] > 0.00):
            self.player.setZ(self.player.getZ() - climbfactor)
            print "z is ",self.player.getZ()

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
            if parameters["quad"]:
                self.resetPosition()
            else:
                self.player.setX(0)

        elif (self.player.getX() > parameters["worldSize"]):
            if parameters["quad"]:
                self.resetPosition()
            else:
                self.player.setX(parameters["worldSize"])

        if (self.player.getY() < 0):
            if parameters["quad"]:
                self.resetPosition()
            else:
                self.player.setY(0)

        elif (self.player.getY() > parameters["worldSize"]):
            if parameters["quad"]:
                self.resetPosition()
            else:
                self.player.setY(parameters["worldSize"])

        # reset to initial position
        if (self.keyMap["init"] != 0):
            self.resetPosition()
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

        if parameters["quad"]:
            # print "x is",self.player.getX
            # print " offsetis",parameters["offset"]
            # x=self.player.getX()
            # y=self.player.getY()
            if (self.player.getX()>parameters["offset"] and self.player.getX()<(parameters["offset"]+1)):
                self.resetPosition()
            if (self.player.getY()>parameters["offset"] and self.player.getY()<(parameters["offset"]+1)):
                self.resetPosition()

        if self.decayTime>0:
            parameters["speed"]=0
            self.decayTime-=1
        elif self.decayTime==0:
            parameters["speed"]=self.speedMemory
            self.decayTime-=1

        if self.reachedDestination():
            self.resetPosition()


    def reachedDestination(self):
        oddeven=np.append(self.odd,self.even,axis=0)
        for i in (oddeven):

            if self.isInsideTarget(i):
                mes=MsgTrajectory()
                mes.reset=True
                self.publisher(mes)
                return True
                break

    def quadPositionGenerator(self):
        offset=(int(parameters["worldSize"])-1)/2

        quad3PosL=parameters["posL"]
        quad3PosR=parameters["posR"]

        quad4PosL=(parameters["posL"][0]+offset,parameters["posL"][1])
        quad4PosR=(parameters["posR"][0]+offset,parameters["posR"][1])

        quad2PosL=(parameters["posL"][0],parameters["posL"][1]+offset)
        quad2PosR=(parameters["posR"][0],parameters["posR"][1]+offset)

        quad1PosL=(parameters["posL"][0]+offset,parameters["posL"][1]+offset)
        quad1PosR=(parameters["posR"][0]+offset,parameters["posR"][1]+offset)

        odd=np.array([quad1PosR,quad2PosL,quad3PosL,quad3PosR])
        even=np.array([quad1PosL,quad2PosR,quad4PosL,quad4PosR])

        # print offset
        # print "even is ",odd
        # print "even is ", even
        return odd,even


    def isInsideTarget(self,target):
        tl,br=self.boundingBoxCoordinates(target,parameters["bboxDist"])
        x,y,z=self.player.getPos()
        if x>tl[0] and x<br[0] and y<tl[1] and y>br[1]:
            return True
        else:
            return False



    def boundingBoxCoordinates(self,target,distance):
        """
        Args:

            obj:the position of object whose bound box has to be found
            distance: the half width of the box | pseudo radius

        Returns:
            tl: top left coordinate.
            br: bottom right coordinate
        """
        tl=(target[0]-distance,target[1]+distance)
        br=(target[0]+distance,target[1]-distance)

        return tl,br

    def resetPosition(self):
        newPos=random.choice(parameters["initPosList"])
        self.player.setPos(newPos)
        self.player.setH(parameters["playerInitH"])

        self.decayTime=240
        self.speedMemory=parameters["speed"]
        print "newPos is", newPos

        return newPos

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
            rospy.loginfo("\n \n \n Bag recording stopped \n \n \n ")
            print "\n \n bagfilename is",self.bagFilename
            self.addMetadata()
            print "metadata added \n \n "

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
            mode="hcp_"
        elif parameters["quad"]:
            mode="quad_"
        self.bagFilename = "bags/" + parameters["fly"] + "_" + mode+parameters["loadingString"]  \
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
        a=self.bagFilename+".bag"
        time.sleep(5)#so that bag file can be transfereed from memory

        metadata=String(json.dumps(parameters))
        print "metadata is:",metadata

        with rosbag.Bag(a,'a') as bag:
            i=0
            for _,_,t in bag.read_messages():
                if i==0:
                    tstamp=t
                i+=1
                break
            bag.write('/metadata',metadata,tstamp)


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
        print "servoangle is", self.servoAngle
        servo.move(1, self.servoAngle)




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
        self.movie('frames/movie', dur, fps=fps, format='jpg', sd=5)

    def fpsLimit(self, fps):
        globalClock.setMode(ClockObject.MLimited)
        globalClock.setFrameRate(fps)

        # to be implemented functions fully unstable


app = MyApp()
app.run()

print 2 + 3
