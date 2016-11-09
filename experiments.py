from enviroment import *
from playerCon import Player
# from myApp import MyApp as app
from importHelper import datetime, np, helper
parameters = helper.paramsFromGUI()


class Experiments(object):

    def __init__(self, showbase):
        self.sb=showbase
        self.obj = Object(showbase)
        self.terrain = Terrain(showbase)
        self.sky = Sky(showbase)
        self.player = Player()

        self.idxArr = self.randIndexArray()
        print "indexArray: " + str(self.idxArr)
        self.objectPosition = None
        self.instance = None
        self.world = None
        self.newPos = None
        self.trial = 1
        self.firstRun = True
        #self.worldFilename = None

    def getObjects(self, objPath, objScale, objTex):
        tempObj = self.obj.getObjects(objPath, objScale, objTex)


        return tempObj

    def setObjects(self, origin, *objects):
        for idx, obj in enumerate(objects):
            self.obj.moveObjects(origin=origin, obj = obj, position = self.objectPosition[idx], instance=self.instance[idx])


    def hideObject(self, *instance):
        try:
            for i in instance:
                i.setPos(50,50,-50)
        except AttributeError:
            pass

    def randIndexArray(self):
        arr = np.arange(parameters["numObj"])
        if parameters["randPos"] == True:
            np.random.shuffle(arr)
        return arr

    def createTerrain(self, modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString):
        self.terrain.initTerrain(modelHeightMap, modelTextureMapNull, modelTextureMap, loadNullModels)
        self.world = self.terrain.generate(modelSizeSuffix, loadingString)

    def createSky(self, loadNullModels, skyMapNull,skyMap,maxDistance,humanDisplay):
        self.sky.createSky(loadNullModels, skyMapNull, skyMap, maxDistance, humanDisplay)
        
    def resetPosition(self, initH=parameters["playerInitH"], speed=parameters["speed"]):
        self.player.resetPos(self.newPos, initH, speed)
        self.trial += 1
        return self.newPos

    def reachedDestination(self):
        for i in self.objectPosition:

            event = self.player.reachedDestination(i)
            print i
            if event is not False:
                return True
        return False






