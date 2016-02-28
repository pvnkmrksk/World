import sys
import rospy
from beginner.msg import MsgFlystate
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog, LVecBase3f
from panda3d.core import loadPrcFileData, NodePath
from pandac.PandaModules import CompassEffect
from direct.gui.OnscreenText import OnscreenText
size=257
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        rospy.init_node('apple')  # record stats # initialize mouse #init node
        loadPrcFileData("", "win-size 2720 768")
        self.debug=True
        self.statusLabel=self.makeStatusLabel(0)

        self.world = self.loader.loadModel("models/world"+str(size)+".bam")
        self.world.reparentTo(self.render)

        # relevant for world boundaries
        self.worldsize = size

        # self.player = self.loader.loadModel("models/alliedflanker.egg")
        self.player=NodePath("player")
        self.initPos=LVecBase3f(self.worldsize/2,self.worldsize/2,1.2)
        self.player.setPos(self.world,self.initPos)
        self.player.setH(self.world,0)


        self.speed = 0.0
        self.maxspeed = 25.0
        self.speedIncrement=0.1

        # A task to run every frame, some keyboard setup and our speed
        self.taskMgr.add(self.updateTask, "update")
        self.keyboardSetup()

        # performance (to be masked later by fog) and view:
        self.maxdistance = 100
        self.camLens.setFar(self.maxdistance)
        self.camLens.setFov(90)
        self.createEnvironment()

    def makeStatusLabel(self,i):
        return OnscreenText(style=2,fg=(0.5,1,.5,1),pos=(-1.3,0.92-(.08*i)),mayChange=1)
    def keyboardSetup(self):
        self.keyMap = {"left":0, "right":0, "climb":0, "fall":0, \
                              "accelerate":0, "decelerate":0, "closed":0,"reverse":0,"init":0}
        self.accept("escape", sys.exit)
        self.accept("a", self.setKey, ["accelerate",1])
        self.accept("a-up", self.setKey, ["accelerate",0])
        self.accept("z", self.setKey, ["decelerate",1])
        self.accept("z-up", self.setKey, ["decelerate",0])
        self.accept("arrow_left", self.setKey, ["left",1])
        self.accept("arrow_left-up", self.setKey, ["left",0])
        self.accept("arrow_right", self.setKey, ["right",1])
        self.accept("arrow_right-up", self.setKey, ["right",0])
        self.accept("arrow_down", self.setKey, ["climb",1])
        self.accept("arrow_down-up", self.setKey, ["climb",0])
        self.accept("arrow_up", self.setKey, ["fall",1])
        self.accept("arrow_up-up", self.setKey, ["fall",0])
        self.accept("o", self.setKey, ["closed",0])
        self.accept("p", self.setKey, ["closed",1])
        self.accept("r",self.setKey,["reverse",1])
        self.accept("r-up",self.setKey,["reverse",0])
        self.accept("i",self.setKey,["init",1])
        base.disableMouse() # or updateCamera will fail!

    def createEnvironment(self):
        # Fog to hide a performance tweak:
        colour = (0.0,0.0,0.0)
        expfog = Fog("scene-wide-fog")
        expfog.setColor(*colour)
        expfog.setExpDensity(0.004)
        render.setFog(expfog)
        base.setBackgroundColor(*colour)

        # Our sky
        skysphere = loader.loadModel('models/sky.egg')
        skysphere.setEffect(CompassEffect.make(self.render))
        skysphere.setScale(self.maxdistance/2) # bit less than "far"
        skysphere.setZ(-5)
        # NOT render - you'll fly through the sky!:
        skysphere.reparentTo(self.camera)

        # Our lighting
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor(Vec4(.6, .6, .6, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(Vec3(0,-10,-10))
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
        scalefactor = self.speed*1/250#(globalClock.getDt()*self.speed)
        climbfactor = (.01)+scalefactor * 1
        bankfactor  = (.4)+scalefactor  *6.5
        speedfactor = scalefactor *1

        # Climb and Fall
        if (self.keyMap["climb"]!=0):# and self.speed > 0.00):
            # faster you go, quicker you climb
            self.player.setZ(self.player.getZ()+climbfactor)

        elif (self.keyMap["fall"]!=0):# and self.speed > 0.00):
            self.player.setZ(self.player.getZ()-climbfactor)

        # Left and Right
        if (self.keyMap["left"]!=0):# and self.speed > 0.0):
            self.player.setH(self.player.getH()+bankfactor)
        elif (self.keyMap["right"]!=0):# and self.speed > 0.0):
            self.player.setH(self.player.getH()-bankfactor)

        # throttle control
        if (self.keyMap["accelerate"]!=0):
            self.speed += self.speedIncrement
            if (self.speed > self.maxspeed):
                self.speed = self.maxspeed
        elif (self.keyMap["decelerate"]!=0):
            self.speed -= self.speedIncrement
            if (self.speed < 0.0):
                self.speed = 0.0

        # move forwards - our X/Y is inverted, see the issue
        self.player.setY(self.player,speedfactor)

        # respect max camera distance else you
        # cannot see the floor post loop the loop!
        if (self.player.getZ() > self.maxdistance):
            self.player.setZ(self.maxdistance)
        # should never happen once we add collision, but in case:
        elif (self.player.getZ() < 0.5):
            self.player.setZ(0.5)

        # and now the X/Y world boundaries:
        if (self.player.getX() < 0):
            self.player.setX(0)
        elif (self.player.getX() > self.worldsize):
            self.player.setX(self.worldsize)

        if (self.player.getY() < 0):
            self.player.setY(0)
        elif (self.player.getY() > self.worldsize):
            self.player.setY(self.worldsize)

        #reset to initial position
        if (self.keyMap["init"]!=0):
            self.player.setPos(self.initPos)





    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 0,0,0)#25.6225, 3.8807, 10.2779)
        self.camera.setHpr(self.player,0,0,0)#94.8996,-16.6549,1.55508)
        print "POS"+str(self.player.getPos())
        print "HPR"+str(self.player.getHpr())
app = MyApp()
app.run()