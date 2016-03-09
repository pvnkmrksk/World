from panda3d.core import Vec3

parameters = {
    # bools
    "self.loadWorld":True,
    "self.loadRedSphere":True,  # False
    "self.loadGreenSphere":False,
    "self.loadTree": True,  # False

    "self.loadHUD": True,
    "self.loadTrajectory": False,
    "self.lockFps ": True,  # False,
    "self.frameRecord ": False,
    "self.loadWind": False,


    "self.bagPrefix": "pavan",
    "self.fps ": 60,
    "self.trajectoryUpdateInterval ": 60,  # frames between update,

    "self.worldSize ": 257,  # relevant for world boundaries
    "self.playerInitPos ": Vec3(257 / 2, 50, 1.3),
    "self.playerInitH ": 0,

    "self.objectSpacing ": 20,  # distance from the midpoint,

    "self.gain ": 1,
    "self.sphereZ ": 4,
    "self.treeZ ": 1,

    "self.speed ": 0.0,
    "self.maxSpeed ": 3.0,
    "self.speedIncrement ": 0.02,
    "self.gainIncrement ": 0.02,
    "self.wbad ": 0,
    "self.wbas ": 0,
    "self.sphereScale ": 0.4,
    "self.treeScale ": 0.03,
    "self.maxDistance ": 250,
    "self.camFOV": 120,
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


assertions = ["self.posL = (self.worldSize / 2 + self.objectSpacing, self.worldSize/2)",
    "self.posR = (self.worldSize / 2 - self.objectSpacing, self.worldSize / 2)",
    "self.treePos = Vec3(self.posL, self.treeZ)",
    "self.redSpherePos = Vec3(self.posL, self.sphereZ)",
    "self.greenSpherePos = Vec3(self.posR, self.sphereZ)"]
