from panda3d.core import Vec3
from direct.task import Task
from pandac.PandaModules import VBase3
from params import parameters

class Fly():
    def __init__(self,loader,scenegraph,taskMgr):
        self.loader = loader
        self.render = scenegraph
        self.taskMgr = taskMgr
        self.maxspeed = 200.0
        self.speed = 0
        self.player.reparentTo(self.render)
        self.reset()
        self.calculate()


        self.maxdistance = 400  # default in case below never called

    def setMaxHeight(self, distance):
        """ Maximum flying altitude """
        self.maxdistance = distance

    def reset(self):
        """ Back to start position, orientation, speed """
        self.player.show()
        self.player.setPos(self.startPos)
        self.player.setHpr(self.startHpr)
        self.speed = self.maxspeed / 5

    def calculate(self):
        """ Should be called every frame.
        It calculates how the player should move (position and orientation) """
        self.factor = globalClock.getDt()
        self.scalefactor = self.factor * self.speed
        self.climbfactor = self.scalefactor * 0.3
        self.bankfactor = self.scalefactor
        self.speedfactor = self.scalefactor * 2.9

    # note the collision enhancements
    def climb(self):
        self.player.setZ(self.player.getZ() + climbfactor)
        print "z is ", self.player.getZ()

    def fall(self):
        self.player.setZ(self.player.getZ() - climbfactor)
        print "z is ", self.player.getZ()


    def bankLeft(self):
        self.player.setH(self.player.getH() + bankfactor)
        # self.currentImposeResponse=-bankfactor

    def bankRight(self):
        self.player.setH(self.player.getH() - bankfactor)
        # self.currentImposeResponse = +bankfactor

    def move(self, boundingBox):
        # move forwards - our X/Y is inverted, see the issue
        valid = True
        if self.exploding == False:
            self.player.setFluidX(self.player, -self.speedfactor)
            valid = self.__inBounds(boundingBox)
            self.player.setFluidZ(self.player, -self.gravityfactor)
        return valid

    def accelerate(self):
        parameters["speed"] += parameters["speedIncrement"]
        if (parameters["speed"] > parameters["maxSpeed"]):
            parameters["speed"] = parameters["maxSpeed"]

        # if (parameters["speed"] < 0.0):
        #     parameters["speed"] = 0.0

    def brake(self):
        parameters["speed"] -= parameters["speedIncrement"]


    def attach(self, node):
        return self.player.attachNewNode(node)

    # See Issue 5 video for what this does:
    def lookAtMe(self, camera):
        percent = (self.__speedAsPercentage()) * 2
        camera.setPos(self.player, 9 + (20 * percent), 0, 0)
        # compensate for model problem (see Issue 3)
        camera.setH(self.player.getH() + 90)
        camera.setP(self.player.getR())
        camera.setR(self.player.getP())
        # final adjustments
        camera.setZ(self.player, 3)
        camera.setY(self.player, 1.5)

"""
the factors are essentially keyboard gains. On user input via keyboard, the gain with which the action
happens is controlled by these numbers
There is a fps invariant factor which is implemented using a frame time to normalize for computing power
"""

# global prevPos, currentPos, ax, fig, treePos, redPos Global Clock by default, panda runs as fast as it can frame to frame
scalefactor = parameters["speed"] * (globalClock.getDt())
climbfactor = 0.01
bankfactor = 6
parameters["wbad"] = self.wbad
parameters["wbas"] = self.wbas

# closed loop
"""
In closed loop, the current heading is updated by adding(subtracting) a value that is product of
wbad and gain.
Heading is defined counterclockwise in degrees.
wbad is left-right. positive wbad is left>right --> right turn --> heading increase cw.
Therefore, the negative sign in effect brings about the negative feedback and makes coherent reality.

In human mode, there is a button on activaqtion, a key down is left and key up is right.
This is a costant race to keep stable and inactivity is not a solution.
One has to constantly osscilate up down to keep heading steady.

"""
if (self.keyMap["closed"] != 0):
    self.player.setH(self.player.getH() - parameters["wbad"] * parameters["gain"])

if (self.keyMap["human"] != 0):
    self.player.setH(self.player.getH() + self.keyMap["hRight"] * parameters["gain"])

# Left and Right
"""
this is actually turn ccw and cw. The increment is bankfactor
"""
if (self.keyMap["left"] != 0):  # and parameters["speed"] > 0.0):
    self.player.setH(self.player.getH() + bankfactor)
    # self.currentImposeResponse=-bankfactor
elif (self.keyMap["right"] != 0):  # and parameters["speed"] > 0.0):
    self.player.setH(self.player.getH() - bankfactor)
    # self.currentImposeResponse = +bankfactor
    # else:
    # self.currentImposeResponse=0

if base.mouseWatcherNode.hasMouse():
    # x = base.mouseWatcherNode.getMouseX()
    self.currentImposeResponse = parameters["gain"] * base.mouseWatcherNode.getMouseY()

self.currentImposeResponse = self.prevH - self.player.getH() - self.stim
self.prevH = self.player.getH()

# Climb and Fall
"""
this is strictly not climb and fall. It is actually Z up and Z down.
when key press, z is incremented (decrenmented) by climbfactor
"""
if (self.keyMap["climb"] != 0):  # and parameters["speed"] > 0.00):
    # faster you go, quicker you climb
    self.player.setZ(self.player.getZ() + climbfactor)
    print "z is ", self.player.getZ()

elif (self.keyMap["fall"] != 0):  # and parameters["speed"] > 0.00):
    self.player.setZ(self.player.getZ() - climbfactor)
    print "z is ", self.player.getZ()

# throttle control
"""
this updates the speed until top speed
handbrake sets speed to zero
"""
if (self.keyMap["accelerate"] != 0):
    parameters["speed"] += parameters["speedIncrement"]
    if (parameters["speed"] > parameters["maxSpeed"]):
        parameters["speed"] = parameters["maxSpeed"]
elif (self.keyMap["decelerate"] != 0):
    parameters["speed"] -= parameters["speedIncrement"]
    # if (parameters["speed"] < 0.0):
    #     parameters["speed"] = 0.0

# handbrake
if (self.keyMap["handBrake"] != 0):
    parameters["speed"] = 0

# todo.scrap reverse gear
if (self.keyMap["reverse"] != 0):
    parameters["speed"] -= parameters["speedIncrement"]

# todo.fix latency of one frame move forwards
"""
This finally updates the position of the player, there is adelay of one frame in speed update.

"""
self.player.setY(self.player, scalefactor)

# update gain
"""
THis updates gain by gainIncrement
"""
if (self.keyMap["gain-up"] != 0):
    parameters["gain"] += parameters["gainIncrement"]
    print "gain is", parameters["gain"]
elif (self.keyMap["gain-down"] != 0):
    parameters["gain"] -= parameters["gainIncrement"]
    print "gain is ", parameters["gain"]

if (self.keyMap["lrGain-down"] != 0):
    parameters["lrGain"] -= parameters["gainIncrement"]
    print "lrGain is ", parameters["lrGain"]

# update DCoffset
"""

DC offset is to fix individual errors in allignment of wbad and tethering
When a fly "intends" to fly straight, the wbad should be around 0.
But due to geometry errors and position errors, the zero is not zero.
The DC offset adds or subtracts a constant amount to set to zero
"""
if (self.keyMap["DCoffset-up"] != 0):
    parameters["DCoffset"] += parameters["DCoffsetIncrement"]
    print "ofset is ", parameters["DCoffset"]

if (self.keyMap["DCoffset-down"] != 0):
    parameters["DCoffset"] -= parameters["DCoffsetIncrement"]
    print "ofset is ", parameters["DCoffset"]

# update user controlled valve state
"""
The valve is controlled via servo code itself. The servo 99 case changes digital state of pin 13

"""
if (self.keyMap["valve-on"] != 0):
    self.valve = 1
    servo.move(99, self.valve)

    # print "valve is ", self.valve
if (self.keyMap["valve-off"] != 0):
    self.valve = 0
    servo.move(99, self.valve)

    # print "valve is ", self.valve

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

# respect quad boundary and time reset
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

# reset to initial position
if (self.keyMap["init"] != 0):
    self.resetPosition("rand")
    time.sleep(0.1)
    # self.player.setPos(self.world, parameters["playerInitPos"])
    # self.player.setH(parameters["playerInitH"])

try:
    self.stim = self.stimList[self.frame]
except IndexError:
    self.stim = False
if parameters["imposeStimulus"]:
    try:
        self.player.setH(self.player.getH() + self.stim)
        self.headingResponseMod = self.cumSummer(self.currentImposeResponse, self.stim,
                                                 self.headingResponseMod)
        self.headingMod = self.cumSummer(self.stim, self.stim, self.headingMod)
        self.compensation = self.headingMod - self.headingResponseMod
    except IndexError:
        print "\n \n impose Stimulus Complete \n \n"
        parameters["imposeStimulus"] = False

# todo.scrap update new init position
if (self.keyMap["newInit"] != 0):
    parameters["playerInitPos"] = self.player.getPos(self.world)
    parameters["playerInitH"] = self.player.getH(self.world)
    print "new init pos is ", parameters["playerInitPos"]
    print "new init H is ", parameters["playerInitH"]

# todo.scrap update newTopSpeed
if (self.keyMap["newTopSpeed"] != 0):
    parameters["maxSpeed"] = parameters["speed"]
    print "new max speed is", parameters["maxSpeed"]

# todo.scrap update left by right gain for diabled flies
if (self.keyMap["lrGain-up"] != 0):
    parameters["lrGain"] += parameters["gainIncrement"]
    print "lrGain is ", parameters["lrGain"]

# if imposing turns, don't change quad after too long a bout
if not parameters["imposeStimulus"]:
    self.tooLongBoutReset()

self.frame += 1
