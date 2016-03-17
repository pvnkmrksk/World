from panda3d.core import Vec3

parameters = {
    # bools

    "loadHUD": True,
    "loadTrajectory": False,
    "lockFps": True,  # False,
    "frameRecord": False,
    "loadWind": False,
    "disabledFly":True,
    "generateWorld":True,
    "loadWorld":True,



    "fly":"test",

    "gain": 5.3,
    "maxSpeed": 7.5,
    "trialNo":1,


    "modelSizeSuffix": "257",
    "modelHeightMap": "models/height" + "257" + "0.png",
    "modelTextureMap": "models/texture" + "257" + "s.png",



    "loadingString":"rg",
    "heightObjects":10,
    "widthObjects":10,
    "lattice":20,
    "origin":Vec3(0,0,0),

    "lrGain":1.0,

    "fps": 60,
    "trajectoryUpdateInterval": 60,  # frames between update,

    "worldSize": 257,  # relevant for world boundaries
    "playerInitPos": (257 / 2, 75, 3),
    "playerInitH": 0,


    "sphereZ": 3,
    "treeZ": 1,

    "speed": 0.0,
    "speedIncrement": 0.05,
    "gainIncrement": 0.02,


    "wbad": 0,
    "wbas": 0,
    "sphereScale": 0.8,
    "treeScale": 0.02,
    "maxDistance": 350,
    "camFOV": 120,
    "camHpr":(0,3,0),
    "frameNum": 0,
    "recordDur": 10,
    "recordFps": 30,
    "windowWidth":2720,
    "windowHeight":768,

    "spherePath": "models/sphere.egg",
    "greenTexPath": "models/green.tga",
    "redTexPath": "models/red.tga",
    "treePath": "models/treeHighB.egg",
    "treeTexPath": "models/BarkBrown.tga",
    "bagTopics": "/usb_cam/image_raw /kinefly/image_output /kinefly/flystate /trajectory",

}



# for key, val in parameters.items():
#     print key+"="+"parameters[\""+key.replace('self.','').strip(" ")+"\"]"
