import sys
import time

import rospy
# import servo
from beginner.msg import MsgFlystate
import std_msgs.msg
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog
from panda3d.core import loadPrcFileData, NodePath, TextNode
from pandac.PandaModules import CompassEffect
from pandac.PandaModules import ClockObject
import subprocess, rostopic
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


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
        self.params()  # fire up the params
        self.debug = False  # deubbing, prints pos
        loadPrcFileData("", "win-size 2720 768")  # set window size
        # globalClock.setMode(ClockObject.MLimited)
        # globalClock.setFrameRate(self.fps)
        rospy.init_node('apple')  # init node


        self.modelsLoad()  # load the models
        self.listener()
        self.keyboardSetup()
        self.createEnvironment()

        self.positionLabel=self.makeStatusLabel(0)
        self.orientationLabel=self.makeStatusLabel(1)
        self.speedLabel=self.makeStatusLabel(2)
        self.gainLabel=self.makeStatusLabel(3)

        self.taskMgr.add(self.updateTask, "update")  # A task to run every frame, some keyboard setup and our speed

        self.fig = plt.figure()
        self.initPlot()

    def vec32String(self,vector,a,b,c):
        """returns a rounded string of vec 3 interspersed with a,b,c as headings"""
        return a+":"+str(round(vector[0]))+" "+b+":"+str(round(vector[1]))+" "+c+":"+str(round(vector[2]))

    def updateLabel(self):
        self.positionLabel.setText(self.vec32String(self.player.getPos(),"x","y","z"))
        self.orientationLabel.setText(self.vec32String(self.player.getHpr(),"H","P","R"))
        self.speedLabel.setText("speed:"+str(self.speed))
        self.gainLabel.setText("gain:" + str(self.gain))

    def makeStatusLabel(self, i):
        return OnscreenText(style=2, fg=(0,0,0, 0.5),bg=(0.4,0.4,0.4,0.8), scale=0.05,pos=(-3.1, 0.92 - (.08 * i)), mayChange=1)

    def params(self):
        # booleans
        self.loadSpheres = True  # False
        self.loadTree = True  # False
        self.loadWind = True

        # params
        self.worldSize = 257  # relevant for world boundaries
        self.fps=240

        self.playerInitPos = Vec3(self.worldSize / 2, 50, 1.3)
        self.playerPrevPos = (self.playerInitPos[0], self.playerInitPos[1])
        self.playerInitH = 0
        self.speed = 0.0
        self.maxSpeed = 3.0
        self.speedIncrement = 0.02
        self.gain = 1  # gain for turning
        self.gainIncrement = 0.02

        self.wbad = self.wbas = 0

        self.spacing = 20  # distance from the midpoint
        self.posL = (self.worldSize / 2 + self.spacing, self.worldSize / 2)
        self.posR = (self.worldSize / 2 - self.spacing, self.worldSize / 2)
        self.sphereZ = 4
        self.treeZ = 1
        self.sphereScale = 0.4
        self.treeScale = 0.03

        self.spherePath = "models/sphere.egg"
        self.greenTexPath = "models/green.tga"
        self.redTexPath = "models/red.tga"
        self.treePath = "models/treeHighB.egg"
        self.treeTexPath = "models/BarkBrown.tga"

        self.treePos = Vec3(self.posL, self.treeZ)
        self.redSpherePos = Vec3(self.posL, self.sphereZ)
        self.greenSpherePos = Vec3(self.posR, self.sphereZ)

        self.maxDistance = 200
        self.camLens.setFar(self.maxDistance)
        self.camLens.setFov(120)

        self.frameNum = 0
        self.trajectoryUpdateInterval = 30  # frames between update

    def modelsLoad(self):
        self.worldLoad()  # load the player and model
        self.treeLoad()  # load the tree
        # self.redSphereLoad()  # load the red sphere
        # self.greenSphereLoad()  # load the green sphere

    def updatePlayer(self):
        # global prevPos, currentPos, ax, fig, treePos, redPos
        # Global Clock by default, panda runs as fast as it can frame to frame
        scalefactor = self.speed * (globalClock.getDt()*self.speed)
        climbfactor = 0.1#(.001) * scalefactor
        bankfactor = 1#.5  * scalefactor
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

    def initPlot(self):
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(0, self.worldSize)
        self.ax.set_ylim(0, self.worldSize)

        plt.plot(self.treePos[0], self.treePos[1], 'gs')
        plt.plot(self.redSpherePos[0], self.redSpherePos[1], 'ro')
        plt.ion()
        plt.show()

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

        self.frameNum += 1

        # # global pos, ori
        # # pos.append(self.player.getPos(self.world))
        # # ori.append(self.player.getHpr(self.world))
        # # # print "pos is ",pos
        # # print "ori is ",ori    def greenSphereLoad(self):

    def greenSphereLoad(self):
        self.greenSphere = self.loader.loadModel(self.spherePath)
        self.greenSphere.setPos(self.world, self.greenSpherePos)
        self.greenSphere.setScale(self.sphereScale)
        self.greenTex = self.loader.loadTexture(self.greenTexPath)
        self.greenSphere.setTexture(self.greenTex)
        self.greenSphere.reparentTo(self.render)

    def redSphereLoad(self):
        self.redSphere = self.loader.loadModel(self.spherePath)
        self.redSphere.setPos(self.world, self.redSpherePos)
        self.redSphere.setScale(self.sphereScale)
        self.redTex = self.loader.loadTexture(self.redTexPath)
        self.redSphere.setTexture(self.redTex)
        self.redSphere.reparentTo(self.render)

    def treeLoad(self):
        self.tree = self.loader.loadModel(self.treePath)
        self.tree.setPos(self.world, self.treePos)
        self.tree.setScale(self.treeScale)
        self.treeTex = self.loader.loadTexture(self.treeTexPath)
        self.tree.setTexture(self.treeTex)
        self.tree.reparentTo(self.render)

    def worldLoad(self):
        self.world = self.loader.loadModel("models/world" + str(self.worldSize) + ".bam")  # loads the world_size
        self.world.reparentTo(self.render)  # render the world
        # the Player
        self.player = NodePath("player")
        self.player.setPos(self.world, self.playerInitPos)
        self.player.setH(self.world, self.playerInitH)  # heading angle is 0


    def windTunnel(self, windDirection):
        self.servoAngle = (self.player.getH() % 360) - 180

    def modelLoader(self, modelName, modelPath, modelPos, modelScale, texName, texPath, modelParent):
        modelName = self.loader.loadModel(modelPath)
        modelName.setPos(modelParent, modelPos)
        modelName.setScale(modelScale)

        texName = self.loader.loadTexture(texPath)
        modelName.setTexture(texName)
        modelName.reparentTo(self.render)

    def callback(self, data):
        """ Returns Wing Beat Amplitude Difference from received data"""
        self.wbad = data.left.angles[0] - data.right.angles[0]
        self.wbas = data.left.angles[0] + data.right.angles[0]
        return self.wbad

    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber("/kinefly/flystate", MsgFlystate, self.callback)

        # print self.wbad


    def keyboardSetup(self):
        self.keyMap = {"left": 0, "right": 0, "climb": 0, "fall": 0,
                       "accelerate": 0, "decelerate": 0, "handBrake": 0, "reverse": 0,
                       "closed": 0, "gain-up": 0, "gain-down": 0,
                       "init": 0, "newInit": 0, "clf": 0}

        self.accept("escape", sys.exit)
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
        base.disableMouse()  # or updateCamera will fail!

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

    def setKey(self, key, value):
        self.keyMap[key] = value

    def updateTask(self, task):
        self.updatePlayer()
        self.updateCamera()
        self.trajectory()
        self.updateLabel()
        return Task.cont

    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 0, 0, 0)
        self.camera.setHpr(self.player, 0, 0, 0)

        if self.debug:
            print "POS" + str(self.player.getPos(self.world))
            print "HPR" + str(self.player.getHpr(self.world))


app = MyApp()
app.run()
