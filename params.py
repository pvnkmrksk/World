from panda3d.core import Vec3

parameters = {
    # bools

    "loadWind": False,
    "loadOdour": False,
    "loadNullModels":False,
    "fly": "fly8",
    "race": "colony apple",
    "emergence": 20160507,
    "age": 50,
    "windQuad": [180, 180, -1, -1],  # -1 is open loop
    "odourQuad":['s',0,'s',0], #0 is always off, 1 is always on, c is custom
    "odour1":"models/odour/1.png",
    "odour2":"models/odour/2.png",
    "odour3":"models/odour/3.png",
    "odour4":"models/odour/4.png",
    "pulseMode":False,
    "packetFrequency":5,

    "loadingString": "rg",
    "DCoffset": 0.216,
    "DCoffsetIncrement": 0.002,

    "trialNo": 1,

    "generateWorld": True,
    "gain": 3.5,
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
    "bboxDist": 7.5,
    "maxBoutDur": 9900,  # frames

    "modelSizeSuffix": "257",
    "modelHeightMap": "models/height" + "257" + "0.png",
    # "modelTextureMap": "models/texture_green_" + "257" + ".png",
    "modelTextureMap": "models/texture" + "257" + "_quadrant_t2_sym.png",
    "modelTextureMapNull": "models/texture" + "257" + "_quadrant_t2_sym_null.png",
    "skyMap":'models/sky.egg',
    "skyMapNull":'models/sky_null.egg',
    # "modelHeightMap": "models/height" + "257" + "0.png",
    # "modelTextureMap": "models/texture" + "257" + "s.png",rqtpl

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
    "camFOV": (180, 140),  # hfov, vfov,

    "worldSize": 257,  # relevant for world boundaries
    "playerInitPos": (200, 170, 3), # start in quadrant 3
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
    "bagTopics": "/rhag_camera/image_raw/compressed /kinefly/image_output /kinefly/flystate /trajectory /vr_camera/image_raw /servo_camera/image_raw/compressed",
    "bagTrajTopics": "/kinefly/flystate /trajectory ",


    "imposeStimulus":False,
    "durList": [4, 2, 1],
    "headingRate": [2, 4, 8],

    "fps": 165,  # repeats items with number of frames since all durations in seconds

    "intraTrial": 2,  # duration between two imposed turns within a sweep
    "interTrial": 4,  # duration between two sweeps
    # "interTrial": 12,  # duration between two sweeps
    "startDur": 8,  # duration of the first turn
    "stopDur": 0.5,  # duration of the last turn
    "stepDur": -1,  # if not in geometric progression gpMode, it is ap mode, linear spaced durations (arithmetic prog)
    "factorDur": 0.5,  # if in gpmode, the common ration of consequetive durations
    # "factorDur": 0.75,  # if in gpmode, the common ration of consequetive durations
    "nSteps": 5,  # number of steps , will be used instead of stopDur if stepmode is true
    "nReps": 20,  # number of times the entire sweep to be repated
    "startHeading": 2.1,  # initial heading rate, will be used if areaMode is false
    "area": 2,  # number of degrees per frame area, will be kept constant , will be used instead of startheading if areaMode is true

    "gpMode": True,  # uses GP instead of AP for durations list gen
    "stepMode": True,  # uses number steps instead of using start and end
    "durListGen": True,  # Generate dur list instead of manually entered
    "headingListGen": True,  # generate heading List ,else use manully entered list
    "areaMode": True,  # Area mode if true, will genreate heading based on given value instead of startHeading
    "signFlip": True,  # flip direction between each turn
    "orderFlip": True,  # flip entire order of sweep

    "mouseMode":False

}

