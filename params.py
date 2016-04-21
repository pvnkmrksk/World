from panda3d.core import Vec3

parameters = {
    # bools

    "loadHUD": True,
    "loadTrajectory": True,
    "lockFps": True,  # False,
    "frameRecord": False,
    "loadWind": False,
    "disabledFly": False,
    "generateWorld": True,
    "loadWorld": True,

    "hcp": False,
    "quad": True,
    "posL": (32, 64),
    "posR": (96, 64),
    "bboxDist": 5.5,
    "maxBoutDur": 3600,  # frames

    "fly": "fly4",
    "windQuad": [-1, 180, 225, 135],  # -1 is open loop
    "loadingString": "g0",
    "gain": 7.0,
    "maxSpeed": 3.5,
    "trialNo": 2,

    "modelSizeSuffix": "257",
    "modelHeightMap": "models/height" + "257" + "0.png",
    # "modelTextureMap": "models/texture_green_" + "257" + ".png",
    "modelTextureMap": "models/texture" + "257" + "_quadrant_t2.png",

    # "modelHeightMap": "models/height" + "257" + "0.png",
    # "modelTextureMap": "models/texture" + "257" + "s.png",

    "killWorldGen": True,

    "heightObjects": 10,
    "widthObjects": 10,
    "lattice": 25,
    "origin": (0, 0, 0),

    "lrGain": 1.0,

    "fps": 60,
    "trajectoryUpdateInterval": 60,  # frames between update,

    "sphereScale": 1.9,
    "treeScale": 0.04,

    "maxDistance": 65,
    "camFOV": 140,

    "worldSize": 257,  # relevant for world boundaries
    "playerInitPos": (64, 32, 1),
    "playerInitH": 0,
    "windDirection": 190,

    "sphereZ": 4.9,
    "treeZ": 0,

    "speed": 0.0,
    "speedIncrement": 0.05,
    "gainIncrement": 0.02,

    "wbad": 0,
    "wbas": 0,

    "camHpr": (0, -2, 0),
    "frameNum": 0,
    "recordDur": 10,
    "recordFps": 30,
    "windowWidth": 2720,
    "windowHeight": 768,

    "spherePath": "models/sphere.egg",
    # "greenTexPath": "models/green.tga",
    "greenTexPath": "models/greenRed.tga",
    "redTexPath": "models/green.tga",
    # "redTexPath": "models/red.tga",
    "treePath": "models/treeHighB.egg",
    "treeTexPath": "models/BarkBrown.tga",
    "bagTopics": "/usb_cam/image_raw /kinefly/image_output /kinefly/flystate /trajectory",

}



# for key, val in parameters.items():
#     print key+"="+"parameters[\""+key.replace('self.','').strip(" ")+"\"]"
