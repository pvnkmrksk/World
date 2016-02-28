#!/usr/bin/env python

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename, AmbientLight, DirectionalLight
from panda3d.core import PandaNode, NodePath, Camera, TextNode
from direct.gui.OnscreenText import OnscreenText
import random
import sys
import os
import math

# Function to put instructions on the screen.
def addInstructions(pos, msg):
    return OnscreenText(text=msg, style=1, fg=(1, 1, 1, 1), scale=.05,
                        shadow=(0, 0, 0, 1), parent=base.a2dTopLeft,
                        pos=(0.08, -pos - 0.04), align=TextNode.ALeft)

# Function to put title on the screen.
def addTitle(text):
    return OnscreenText(text=text, style=1, fg=(1, 1, 1, 1), scale=.07,
                        parent=base.a2dBottomRight, align=TextNode.ARight,
                        pos=(-0.1, 0.09), shadow=(0, 0, 0, 1))


class RoamingflyDemo(ShowBase):
    def __init__(self):
        # Set up the window, camera, etc.
        ShowBase.__init__(self)

        # Set the background color to black
        self.win.setClearColor((0, 0, 0, 1))

        # This is used to store which keys are currently pressed.
        self.keyMap = {
            "left": 0, "right": 0, "forward": 0, "cam-left": 0, "cam-right": 0}

        # # Post the instructions
        # self.title = addTitle(
        #     "Panda3D Tutorial: Roaming fly (Walking on Uneven Terrain)")
        # self.inst1 = addInstructions(0.06, "[ESC]: Quit")
        # self.inst2 = addInstructions(0.12, "[Left Arrow]: Rotate fly Left")
        # self.inst3 = addInstructions(0.18, "[Right Arrow]: Rotate fly Right")
        # self.inst4 = addInstructions(0.24, "[Up Arrow]: Run fly Forward")
        # self.inst6 = addInstructions(0.30, "[A]: Rotate Camera Left")
        # self.inst7 = addInstructions(0.36, "[S]: Rotate Camera Right")

        # Set up the environment
        #
        # This environment model contains collision meshes.  If you look
        # in the egg file, you will see the following:
        #
        #    <Collide> { Polyset keep descend }
        #
        # This tag causes the following mesh to be converted to a collision
        # mesh -- a mesh which is optimized for collision, not rendering.
        # It also keeps the original mesh, so there are now two copies ---
        # one optimized for rendering, one for collisions.

        self.environ = loader.loadModel("models/world65.bam")
        self.environ.reparentTo(render)

        # Create the main character, fly

        # flyStartPos = self.environ.find("**/start_point").getPos()
        self.fly = NodePath("fly")
        self.fly.reparentTo(render)
        self.fly.setPos(15,15,2)

        # Create a floater object, which floats 2 units above fly.  We
        # use this as a target for the camera to look at.

        self.floater = NodePath(PandaNode("floater"))
        self.floater.reparentTo(self.fly)
        self.floater.setZ(2.0)

        # Accept the control keys for movement and rotation

        self.accept("escape", sys.exit)
        self.accept("arrow_left", self.setKey, ["left", True])
        self.accept("arrow_right", self.setKey, ["right", True])
        self.accept("arrow_up", self.setKey, ["forward", True])
        self.accept("a", self.setKey, ["cam-left", True])
        self.accept("s", self.setKey, ["cam-right", True])
        self.accept("arrow_left-up", self.setKey, ["left", False])
        self.accept("arrow_right-up", self.setKey, ["right", False])
        self.accept("arrow_up-up", self.setKey, ["forward", False])
        self.accept("a-up", self.setKey, ["cam-left", False])
        self.accept("s-up", self.setKey, ["cam-right", False])

        taskMgr.add(self.move, "moveTask")

        # >
        # Set up the camera

        self.disableMouse()
        self.camera.setPos(self.fly.getX(), self.fly.getY() + 10, 2)

        # We will detect the height of the terrain by creating a collision
        # ray and casting it downward toward the terrain.  One ray will
        # start above fly's head, and the other will start above the camera.
        # A ray may hit the terrain, or it may hit a rock or a tree.  If it
        # hits the terrain, we can detect the height.  If it hits anything
        # else, we rule that the move is illegal.
        # self.cTrav = CollisionTraverser()
        #
        # self.flyGroundRay = CollisionRay()
        # self.flyGroundRay.setOrigin(0, 0, 9)
        # self.flyGroundRay.setDirection(0, 0, -1)
        # self.flyGroundCol = CollisionNode('flyRay')
        # self.flyGroundCol.addSolid(self.flyGroundRay)
        # self.flyGroundCol.setFromCollideMask(CollideMask.bit(0))
        # self.flyGroundCol.setIntoCollideMask(CollideMask.allOff())
        # self.flyGroundColNp = self.fly.attachNewNode(self.flyGroundCol)
        # self.flyGroundHandler = CollisionHandlerQueue()
        # self.cTrav.addCollider(self.flyGroundColNp, self.flyGroundHandler)
        #
        # self.camGroundRay = CollisionRay()
        # self.camGroundRay.setOrigin(0, 0, 9)
        # self.camGroundRay.setDirection(0, 0, -1)
        # self.camGroundCol = CollisionNode('camRay')
        # self.camGroundCol.addSolid(self.camGroundRay)
        # self.camGroundCol.setFromCollideMask(CollideMask.bit(0))
        # self.camGroundCol.setIntoCollideMask(CollideMask.allOff())
        # self.camGroundColNp = self.camera.attachNewNode(self.camGroundCol)
        # self.camGroundHandler = CollisionHandlerQueue()
        # self.cTrav.addCollider(self.camGroundColNp, self.camGroundHandler)

        # Uncomment this line to see the collision rays
        #self.flyGroundColNp.show()
        #self.camGroundColNp.show()

        # Uncomment this line to show a visual representation of the
        # collisions occuring
        #self.cTrav.showCollisions(render)

        # Create some lighting
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.3, .3, .3, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection((-5, -5, -5))
        directionalLight.setColor((1, 1, 1, 1))
        directionalLight.setSpecularColor((1, 1, 1, 1))
        render.setLight(render.attachNewNode(ambientLight))
        render.setLight(render.attachNewNode(directionalLight))

    # Records the state of the arrow keys
    def setKey(self, key, value):
        self.keyMap[key] = value

    # Accepts arrow keys to move either the player or the menu cursor,
    # Also deals with grid checking and collision detection
    def move(self, task):

        # Get the time that elapsed since last frame.  We multiply this with
        # the desired speed in order to find out with which distance to move
        # in order to achieve that desired speed.
        dt = globalClock.getDt()

        # If the camera-left key is pressed, move camera left.
        # If the camera-right key is pressed, move camera right.

        if self.keyMap["cam-left"]:
            self.camera.setX(self.camera, -20 * dt)
        if self.keyMap["cam-right"]:
            self.camera.setX(self.camera, +20 * dt)

        # save fly's initial position so that we can restore it,
        # in case he falls off the map or runs into something.

        startpos = self.fly.getPos()

        # If a move-key is pressed, move fly in the specified direction.

        if self.keyMap["left"]:
            self.fly.setH(self.fly.getH() + 300 * dt)
        if self.keyMap["right"]:
            self.fly.setH(self.fly.getH() - 300 * dt)
        if self.keyMap["forward"]:
            self.fly.setY(self.fly, -25 * dt)

        # If fly is moving, loop the run animation.
        # If he is standing still, stop the animation.

        # if self.keyMap["forward"] or self.keyMap["left"] or self.keyMap["right"]:
        #     if self.isMoving is False:
        #         self.fly.loop("run")
        #         self.isMoving = True
        # else:
        #     if self.isMoving:
        #         self.fly.stop()
        #         self.fly.pose("walk", 5)
        #         self.isMoving = False

        # If the camera is too far from fly, move it closer.
        # If the camera is too close to fly, move it farther.

        # camvec = self.fly.getPos() - self.camera.getPos()
        # camvec.setZ(0)
        # camdist = camvec.length()
        # camvec.normalize()
        # if camdist > 10.0:
        #     self.camera.setPos(self.camera.getPos() + camvec * (camdist - 10))
        #     camdist = 10.0
        # if camdist < 5.0:
        #     self.camera.setPos(self.camera.getPos() - camvec * (5 - camdist))
        #     camdist = 5.0

        # Normally, we would have to call traverse() to check for collisions.
        # However, the class ShowBase that we inherit from has a task to do
        # this for us, if we assign a CollisionTraverser to self.cTrav.
        #self.cTrav.traverse(render)

        # Adjust fly's Z coordinate.  If fly's ray hit terrain,
        # update his Z. If it hit anything else, or didn't hit anything, put
        # him back where he was last frame.

        # entries = list(self.flyGroundHandler.getEntries())
        # entries.sort(key=lambda x: x.getSurfacePoint(render).getZ())
        #
        # if len(entries) > 0 and entries[0].getIntoNode().getName() == "terrain":
        #     self.fly.setZ(entries[0].getSurfacePoint(render).getZ())
        # else:
        #     self.fly.setPos(startpos)

        # Keep the camera at one foot above the terrain,
        # or two feet above fly, whichever is greater.
        #
        # entries = list(self.camGroundHandler.getEntries())
        # entries.sort(key=lambda x: x.getSurfacePoint(render).getZ())
        #
        # if len(entries) > 0 and entries[0].getIntoNode().getName() == "terrain":
        #     self.camera.setZ(entries[0].getSurfacePoint(render).getZ() + 1.0)
        # if self.camera.getZ() < self.fly.getZ() + 2.0:
        #     self.camera.setZ(self.fly.getZ() + 2.0)

        # The camera should look in fly's direction,
        # but it should also try to stay horizontal, so look at
        # a floater which hovers above fly's head.
        self.camera.lookAt(self.floater)

        return task.cont


demo = RoamingflyDemo()
demo.run()