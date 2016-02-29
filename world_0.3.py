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
        loadPrcFileData("", "win-size 2720 768")                        #set window size
        ShowBase.__init__(self)                                         #start the app
        rospy.init_node('apple')                                        #init node
        self.debug=True                                                 #deubbing, prints pos
        self.statusLabel=self.makeStatusLabel(0)

        self.world = self.loader.loadModel("models/world"+str(size)+".bam") #loads the world of dimesions size
        self.world.reparentTo(self.render)                              #render the world

        # relevant for world boundaries
        self.worldSize = size
        self.initPos=LVecBase3f(self.worldSize/2,self.worldSize/2.5,1.3)  #z is factored by 2 dunno why?
        self.initH=0
        #the Player
        self.player=NodePath("player")
        self.player.setPos(self.world,self.initPos)
        self.player.setH(self.world,self.initH)                                  #heading angle is 0


        self.speed = 0.0
        self.maxspeed = 25.0
        self.speedIncrement=0.1

        #spheres

        #Red sphere
        #load model, set position, set scale, apply texture
        self.redSphere=self.loader.loadModel("models/sphere.egg")
        self.redSphere.setPos(self.world,self.worldSize/2-10,self.worldSize/2,1.3)
        self.redSphere.setScale(0.25)
        #load redSphere texture
        self.redTex=self.loader.loadTexture("models/red.tga")
        self.redSphere.setTexture(self.redTex)
        self.redSphere.reparentTo(self.render)

        #Green Sphere
        self.greenSphere=self.loader.loadModel("models/sphere.egg")
        self.greenSphere.setPos(self.redSphere,10,0,0)
        self.greenSphere.setScale(0.25)
        #load greenSphere texture
        self.greenTex=self.loader.loadTexture("models/green.tga")
        self.greenSphere.setTexture(self.greenTex)
        # self.greenSphere.reparentTo(self.render)

        #Load tree
        self.tree=self.loader.loadModel("models/treeHighB.egg")
        self.tree.setPos(self.world,self.worldSize/2+10,self.worldSize/2,1)
        self.tree.setScale(0.01)

        self.treeTex=self.loader.loadTexture("models/BarkBrown.tga")
        self.tree.setTexture(self.treeTex)
        self.tree.reparentTo(self.render)

        #wbad
        self.wbad = self.wbas = 0
        self.closedGain=1 #gain for turning
        self.listener()

        # A task to run every frame, some keyboard setup and our speed
        self.taskMgr.add(self.updateTask, "update")
        self.keyboardSetup()

        # performance (to be masked later by fog) and view:
        self.maxdistance = 200
        self.camLens.setFar(self.maxdistance)
        self.camLens.setFov(120)
        self.createEnvironment()


    def callback(self, data):
        """ Returns Wing Beat Amplitude Difference from received data"""
        self.wbad = data.left.angles[0] - data.right.angles[0]
        self.wbas = data.left.angles[0] + data.right.angles[0]
        return self.wbad

    def listener(self):
        """ Listens to Kinefly Flystate topic"""
        rospy.Subscriber("/kinefly/flystate", MsgFlystate, self.callback)
        # print self.wbad
    def makeStatusLabel(self,i):
        return OnscreenText(style=2,fg=(0.5,1,.5,1),pos=(-1.3,0.92-(.08*i)),mayChange=1)

    def keyboardSetup(self):
        self.keyMap = {"left":0, "right":0, "climb":0, "fall":0, \
                              "accelerate":0, "decelerate":0, "closed":0,"reverse":0,"init":0}
        self.accept("escape", sys.exit)
        self.accept("a", self.setKey, ["climb",1])
        self.accept("a-up", self.setKey, ["climb",0])
        self.accept("z", self.setKey, ["fall",1])
        self.accept("z-up", self.setKey, ["fall",0])
        self.accept("arrow_left", self.setKey, ["left",1])
        self.accept("arrow_left-up", self.setKey, ["left",0])
        self.accept("arrow_right", self.setKey, ["right",1])
        self.accept("arrow_right-up", self.setKey, ["right",0])
        self.accept("arrow_down", self.setKey, ["decelerate",1])
        self.accept("arrow_down-up", self.setKey, ["decelerate",0])
        self.accept("arrow_up", self.setKey, ["accelerate",1])
        self.accept("arrow_up-up", self.setKey, ["accelerate",0])
        self.accept("o", self.setKey, ["closed",0])
        self.accept("p", self.setKey, ["closed",1])
        self.accept("r",self.setKey,["reverse",1])
        self.accept("r-up",self.setKey,["reverse",0])
        self.accept("i",self.setKey,["init",1])
        self.accept("i-up",self.setKey,["init",0])
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

        #closed loop
        if (self.keyMap["closed"] != 0):
            self.player.setH(self.player.getH() + self.wbad*self.closedGain)

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

        # reverse gear
        if (self.keyMap["reverse"]!=0):
            self.speed-=self.speedIncrement

        # move forwards
        self.player.setY(self.player,speedfactor)

        # respect max camera distance else you
        # cannot see the floor post loop the loop!
        if (self.player.getZ() > self.maxdistance):
            self.player.setZ(self.maxdistance)

        elif (self.player.getZ() < 0.5):
            self.player.setZ(0.5)

        # and now the X/Y world boundaries:
        if (self.player.getX() < 0):
            self.player.setX(0)
        elif (self.player.getX() > self.worldSize):
            self.player.setX(self.worldSize)

        if (self.player.getY() < 0):
            self.player.setY(0)
        elif (self.player.getY() > self.worldSize):
            self.player.setY(self.worldSize)

        #reset to initial position
        if (self.keyMap["init"]!=0):
            self.player.setPos(self.world,self.initPos)
            self.player.setH(self.initH)





    def updateCamera(self):
        # see issue content for how we calculated these:
        self.camera.setPos(self.player, 0,0,0)#25.6225, 3.8807, 10.2779)
        self.camera.setHpr(self.player,0,0,0)#94.8996,-16.6549,1.55508)
        if self.debug:
            print "POS"+str(self.player.getPos(self.world))
            print "HPR"+str(self.player.getHpr(self.world))
app = MyApp()
app.run()