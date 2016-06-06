from panda3d.core import Vec3

parameters = {
    # bools

    "loadHUD": True,
    "loadTrajectory": True,
    "lockFps": False,  # False,
    "frameRecord": False,
    "loadWind":  True,
    "disabledFly": False,
    "generateWorld": False,
    "loadWorld": True,

    "replayWorld": False,
    "captureScale": 1,  # scale the window to improve speed
    "captureStart": 0,  # start point of playback
    "playbackIncrement": 1,  # step size of playback
    "recordDur": 10,  # in seconds
    "recordFps": 30,

    "hcp": False,
    "quad": True,
    "posL": (32, 64),
    "posR": (96, 64),
    "bboxDist": 5.5,
    "maxBoutDur": 36000,  # frames

    "fly": "fly6",
    "race": "colony apple",
    "emergence": 20160307,
    "age": 50,
    "windQuad": [-1, 180, 225, 135],  # -1 is open loop
    "loadingString": "rg",
    "gain": 7.0,
    "maxSpeed": 3.5,
    "trialNo": 1,

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

    "spherePath": "models/sphere.egg",
    "greenTexPath": "models/green.tga",
    # "greenTexPath": "models/greenRed.tga",
    # "redTexPath": "models/greenRed.tga",
    "redTexPath": "models/red.tga",
    "treePath": "models/treeHighB.egg",
    "treeTexPath": "models/BarkBrown.tga",
    "bagTopics": "/usb_cam/image_raw /kinefly/image_output /kinefly/flystate /trajectory",

}



# for key, val in parameters.items():
#     print key+"="+"parameters[\""+key.replace('self.','').strip(" ")+"\"]"
