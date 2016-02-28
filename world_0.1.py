import sys

import rospy
from beginner.msg import MsgFlystate
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog
from panda3d.core import loadPrcFileData
from pandac.PandaModules import CompassEffect

rospy.init_node('apple')  # record stats # initialize mouse #init node

loadPrcFileData("", "win-size 2720 768")


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.world = self.loader.loadModel("models/world65.bam")
        self.world.reparentTo(self.render)

        # relevant for world boundaries
        self.worldsize = 65

        self.player = self.loader.loadModel("models/alliedflanker.egg")
        self.player.setPos(self.world, 15, 15, 0.01)
        self.player.setH(self.world, 270)
        # self.player.reparentTo(self.render)
        #
        self.tree = self.loader.loadModel("models/treeHighB.egg")
        self.tree.setPos(self.world, 5, 6, 0)
        self.tree.reparentTo(self.render)
        self.tree.setScale(.009, .009, .02)

        self.treeTex = self.loader.loadTexture("models/BarkBlank.tga")
        self.tree.setTexture(self.treeTex)

        # A task to run every frame, some keyboard setup and our speed
        self.taskMgr.add(self.updateTask, "update")
        self.keyboardSetup()
        self.speed = 0.0
        self.maxspeed = 1.0
        self.player.setScale(.2, .2, .2)

        # performance (to be masked later by fog) and view:
        self.maxdistance = 32
        self.camLens.setFar(self.maxdistance)
        self.camLens.setFov(120)
        self.wbad = self.wbas = 0
        self.gain=2
        self.speedIncrement=0.001
        self.createEnvironment()
        self.listener()

    def callback(self, data):
        """ Returns Wing Beat Amplitude Difference from received data"""
        self.wbad = data.left.angles[0] - data.right.angles[0]
        self.wbas = data.left.angles[0] + data.right.angles[0]
        self.wbad*=self.gain
        return self.wbad

    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber("/kinefly/flystate", MsgFlystate, self.callback)

    def keyboardSetup(self):
        self.keyMap = {"left": 0, "right": 0, "climb": 0, "fall": 0, \
                       "accelerate": 0, "decelerate": 0, "fire": 0,"reverse":0}
        self.accept("escape", sys.exit)
        self.accept("a", self.setKey, ["accelerate", 1])
        self.accept("a-up", self.setKey, ["accelerate", 0])
        self.accept("z", self.setKey, ["decelerate", 1])
        self.accept("z-up", self.setKey, ["decelerate", 0])
        self.accept("arrow_left", self.setKey, ["left", 1])
        self.accept("arrow_left-up", self.setKey, ["left", 0])
        self.accept("arrow_right", self.setKey, ["right", 1])
        self.accept("arrow_right-up", self.setKey, ["right", 0])
        self.accept("arrow_down", self.setKey, ["climb", 1])
        self.accept("arrow_down-up", self.setKey, ["climb", 0])
        self.accept("arrow_up", self.setKey, ["fall", 1])
        self.accept("arrow_up-up", self.setKey, ["fall", 0])
        self.accept("space", self.setKey, ["fire", 1])
        # self.accept("space-up", self.setKey, ["fire",1])
        self.accept("p", self.setKey, ["fire", 0])
        self.accept("r", self.setKey, ["reverse", 1])
        self.accept("r-up", self.setKey, ["reverse", 0])

        base.disableMouse()  # or updateCamera will fail!

    def createEnvironment(self):
        # Fog to hide a performance tweak:
        colour = (0.0, 0.0, 0.0)
        expfog = Fog("scene-wide-fog")
        expfog.setColor(*colour)
        expfog.setExpDensity(0.0004)
        render.setFog(expfog)
        base.setBackgroundColor(*colour)

        # Our sky
        skydome = loader.loadModel('models/sky.egg')
        skydome.setEffect(CompassEffect.make(self.render))
        skydome.setScale(self.maxdistance / 2)  # bit less than "far"
        skydome.setZ(-65)  # sink it
        # NOT render - you'll fly through the sky!:
        skydome.reparentTo(self.camera)

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
        return Task.cont

    def updatePlayer(self):
        # Global Clock
        # by default, panda runs as fast as it can frame to frame
        scalefactor = (globalClock.getDt() * self.speed)#*self.speedIncrement)
        climbfactor = scalefactor * 0.5
        bankfactor = 1
        speedfactor = scalefactor * 2.9

        # Climb and Fall
        if (self.keyMap["climb"] != 0 ):#and self.speed > 0.00):
            # faster you go, quicker you climb
            self.player.setZ(self.player.getZ() + climbfactor)
            self.player.setR(self.player.getR() + climbfactor)
            # quickest return: (avoids uncoil/unwind)
            if (self.player.getR() >= 180):
                self.player.setR(-180)
        elif (self.keyMap["fall"] != 0 ):#and self.speed > 0.00):
            self.player.setZ(self.player.getZ() - climbfactor)
            self.player.setR(self.player.getR() - climbfactor)
            # quickest return:
            if (self.player.getR() <= -180):
                self.player.setR(180)
        # autoreturn - add a bit regardless to make sure it happens
        elif (self.player.getR() > 0):
            self.player.setR(self.player.getR() - (climbfactor + 0.1))
            if (self.player.getR() < 0):
                self.player.setR(0)  # avoid jitter
        elif (self.player.getR() < 0):
            self.player.setR(self.player.getR() + (climbfactor + 0.1))
            if (self.player.getR() > 0):
                self.player.setR(0)

        # Left and Right
        if (self.keyMap["left"] != 0):  # and self.speed > 0.0):
            self.player.setH(self.player.getH() + bankfactor)  # +self.wbad)
            # self.player.setP(self.player.getP()+bankfactor)
            # quickest return:
            if (self.player.getP() >= 180):
                self.player.setP(-180)
        elif (self.keyMap["right"] != 0):  # and self.speed > 0.0):
            self.player.setH(self.player.getH() - bankfactor)
            # self.player.setP(self.player.getP()-bankfactor)
            if (self.player.getP() <= -180):
                self.player.setP(180)

        if (self.keyMap["fire"] != 0):
            self.player.setH(self.player.getH() + self.wbad)
            if (self.player.getP() >= 180):
                self.player.setP(-180)
            elif (self.player.getP() <= -180):
                self.player.setP = 180

        # autoreturn
        elif (self.player.getP() > 0):
            self.player.setP(self.player.getP() - (bankfactor + 0.1))
            if (self.player.getP() < 0):
                self.player.setP(0)
        elif (self.player.getP() < 0):
            self.player.setP(self.player.getP() + (bankfactor + 0.1))
            if (self.player.getP() > 0):
                self.player.setP(0)

        # throttle control
        if (self.keyMap["accelerate"] != 0):
            self.speed += self.speedIncrement
            if (self.speed > self.maxspeed):
                self.speed = self.maxspeed
        elif (self.keyMap["decelerate"] != 0):
            self.speed -= self.speedIncrement
            if (self.speed < 0.0):
                self.speed = 0.0

        # reverse gear
        if (self.keyMap["reverse"] != 0):
            self.speed -=1



        # move forwards - our X/Y is inverted, see the issue
        self.player.setX(self.player, -speedfactor)

        # respect max camera distance else you
        # cannot see the floor post loop the loop!
        if (self.player.getZ() > self.maxdistance):
            self.player.setZ(self.maxdistance)
        # should never happen once we add collision, but in case:
        elif (self.player.getZ() < 0):
            self.player.setZ(0)

        # and now the X/Y world boundaries:
        if (self.player.getX() < 0):
            self.player.setX(0)
        elif (self.player.getX() > self.worldsize):
            self.player.setX(self.worldsize)

        if (self.player.getY() < 0):
            self.player.setY(0)
        elif (self.player.getY() > self.worldsize):
            self.player.setY(self.worldsize)

    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 25.6225, 3.8807, 10.2779)
        self.camera.setHpr(self.player, 94.8996, 06.6549, 1.55508)


app = MyApp()
app.run()
