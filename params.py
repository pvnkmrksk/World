from panda3d.core import Vec3

parameters = {
    # bools

    "self.loadHUD": True,
    "self.loadTrajectory": True,
    "self.lockFps ": True,  # False,
    "self.frameRecord ": False,
    "self.loadWind": False,

    "self.loadWorld":True,
    "self.fly":"fly2",
    "self.leftObjects":"t",
    "self.rightObjects":"0",
    "self.gain ": 5.3,
    "self.maxSpeed ": 7.5,
    "self.trialNo":5,

    "self.objectSpacing ": 45,  # distance from the midpoint,

    "self.fps ": 60,
    "self.trajectoryUpdateInterval ": 60,  # frames between update,

    "self.worldSize ": 257,  # relevant for world boundaries
    "self.playerInitPos ": Vec3(257 / 2, 55, 3),
    "self.playerInitH ": 0,


    "self.sphereZ ": 3,
    "self.treeZ ": 1,

    "self.speed ": 0.0,
    "self.speedIncrement ": 0.05,
    "self.gainIncrement ": 0.02,


    "self.wbad ": 0,
    "self.wbas ": 0,
    "self.sphereScale ": 0.8,
    "self.treeScale ": 0.08,
    "self.maxDistance ": 350,
    "self.camFOV": 120,
    "self.camHpr":Vec3(0,3,0),
    "self.frameNum ": 0,
    "self.recordDur ": 10,
    "self.recordFps ": 30,

    "self.spherePath ": "models/sphere.egg",
    "self.greenTexPath ": "models/green.tga",
    "self.redTexPath ": "models/red.tga",
    "self.treePath ": "models/treeHighB.egg",
    "self.treeTexPath ": "models/BarkBrown.tga",
    "self.bagTopics": "/usb_cam/image_raw /kinefly/image_output /kinefly/flystate /trajectory",

}


assertions = [
    "self.posL = (self.worldSize / 2 - self.objectSpacing, self.worldSize/2)",
    "self.posR = (self.worldSize / 2 + self.objectSpacing, self.worldSize / 2)",
    "self.leftTreePos = Vec3(self.posL, self.treeZ)",
    "self.rightTreePos = Vec3(self.posR, self.treeZ)",
    "self.leftSpherePos = Vec3(self.posL, self.sphereZ)",
    "self.rightSpherePos = Vec3(self.posR, self.sphereZ)"]
