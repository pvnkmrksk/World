from panda3d.core import Vec3

parameters = {
    # bools

    "loadWind": False,
    "loadOdour":True,
    "loadTrajectory": False,
    "loadNullModels":False,
    "imposeStimulus": False,
    "loadingString": "rg",
    "gain":7,
    "generateWorld": False,

    "DCoffset": -0.002,
    "DCoffsetIncrement": 0.012,

    "fly": "fly14",
    "race": "colony apple",
    "emergence": 20160817,
    "windQuad": [-1, -1, -1, -1],  # -1 is open loop
    "odourQuad":[1,1,1,1], #0 is always off, 1 is always on, c is custom
    # "odourQuad":['s','c','c','c'], #0 is always off, 1 is always on, c is custom
    "odour1":"models/odour/1.png",
    "odour2":"models/odour/2.png",
    "odour3":"models/odour/3.png",
    "odour4":"models/odour/4.png",
    "trialNo": 1,

    "packetFrequency":1,
    "packetDur":4,

    "maxSpeed": 7,

    "loadHUD": False,
    "loadWorld": True,

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

    "beepPath":"models/sounds/beep.wav",

    "quad": True,
    "resetObject" : True,
    "posL": (32, 64),
    "posR": (96, 64),
    "bboxDist": 7.5,
    "maxBoutDur": 9900,  # frames

    "modelSizeSuffix": "257",
    "modelHeightMap": "models/height" + "257" + "0.png",
    "modelTextureMap": "models/texture" + "257" + "_quadrant_t2_sym.png",
    "modelTextureMapNull": "models/texture" + "257" + "_quadrant_t2_sym_null.png",
    "skyMap":'models/sky.egg',
    "skyMapNull":'models/sky_null.egg',

    "origin": (0, 0, 0),

    "lrGain": 1.0,
    "lockFps": True,  # False,
    "fps": 60,
    "trajectoryUpdateInterval": 165,  # frames between update,

    "sphereScale": 1.9,
    "treeScale": 0.1,

    "maxDistance": 65,#90,#65,
    "camFOV": (180, 140),  # hfov, vfov,

    "worldSize": 257,  # relevant for world boundaries
    "playerInitPos": (64, 32, 3), # start in quadrant 3
    # "playerInitPos": (64, 64, 3), # start in quadrant 3
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
    "bagFullTopics": "/rhag_camera/image_raw/compressed /kinefly/image_output /servo_camera/image_raw/compressed /kinefly/flystate /trajectory  ",
    #/vr_camera/image_raw
    "bagTrajTopics": "/kinefly/flystate /trajectory ",


    "durList": [3, 3, 3,3,3,3],
    "headingRate": [0.25,0.5,1,2, 4, 8],

    "fps": 60,  # repeats items with number of frames since all durations in seconds

    "intraTrial": 3,  # duration between two imposed turns within a sweep
    "interTrial": 3,  # duration between two sweeps
    # "interTrial": 12,  # duration between two sweeps
    "startDur": 12,  # duration of the first turn
    "stopDur": 2,  # duration of the last turn
    "stepDur": -1,  # if not in geometric progression gpMode, it is ap mode, linear spaced durations (arithmetic prog)
    "factorDur": 0.65,  # if in gpmode, the common ration of consequetive durations
    # "factorDur": 0.75,  # if in gpmode, the common ration of consequetive durations
    "nSteps": 5,  # number of steps , will be used instead of stopDur if stepmode is true
    "nReps": 20,  # number of times the entire sweep to be repated
    "startHeading": 2.1,  # initial heading rate, will be used if areaMode is false
    "area": 8,  # number of degrees per frame area, will be kept constant , will be used instead of startheading if areaMode is true

    "gpMode": True,  # uses GP instead of AP for durations list gen
    "stepMode": True,  # uses number steps instead of using start and end
    "durListGen": False,  # Generate dur list instead of manually entered
    "headingListGen": False,  # generate heading List ,else use manully entered list
    "areaMode": True,  # Area mode if true, will genreate heading based on given value instead of startHeading
    "signFlip": True,  # flip direction between each turn
    "orderFlip": True,  # flip entire order of sweep

    "mouseMode":False,
    "humanDisplay":False

}


