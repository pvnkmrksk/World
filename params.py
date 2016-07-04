from panda3d.core import Vec3

parameters = {
    # bools

    "loadWind": True,
    "loadOdour": True,

    "fly": "fly6",
    "race": "colony apple",
    "emergence": 20160507,
    "age": 50,
    "windQuad": [-1, -1, 180, 180],  # -1 is open loop
    "odourQuad":[0,1,'c',1], #0 is always off, 1 is always on, c is custom
    "odour1":"models/odour/1.png",
    "odour2":"models/odour/2.png",
    "odour3":"models/odour/3.png",
    "odour4":"models/odour/4.png",

    "loadingString": "rg",
    "trialNo": 1,

    "generateWorld": False,
    "gain": 7.0,
    "maxSpeed": 3.5,

    "loadHUD": False,
    "loadTrajectory": True,
    "loadWorld": True,
    "disabledFly": False,

    "frameRecord": False,
    "replayWorld": False,
    "captureScale": 1,  # scale the window to improve speed
    "captureStart": 0,  # start point of playback
    "playbackIncrement": 1,  # step size of playback
    "recordDur": 10,  # in seconds
    "recordFps": 30,

    "spherePath": "models/sphere.egg",
    "greenTexPath": "models/green.tga",
    # "greenTexPath": "models/greenRed.tga",
    # "redTexPath": "models/greenRed.tga",
    "redTexPath": "models/red.tga",
    "treePath": "models/treeHighB.egg",
    "treeTexPath": "models/BarkBrown.tga",

    "hcp": False,
    "quad": True,
    "posL": (32, 64),
    "posR": (96, 64),
    "bboxDist": 5.5,
    "maxBoutDur": 36000,  # frames

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
    "lockFps": True,  # False,
    "fps": 165,
    "trajectoryUpdateInterval": 165,  # frames between update,

    "sphereScale": 1.9,
    "treeScale": 0.04,

    "maxDistance": 65,
    "camFOV": (178, 140),  # hfov, vfov,

    "worldSize": 257,  # relevant for world boundaries
    "playerInitPos": (64, 32, 3),
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

    "windowWidth": 4320,  # 1440,
    "windowHeight": 2560,
    "bagTopics": "/usb_cam/image_raw /kinefly/image_output /kinefly/flystate /trajectory",

}


