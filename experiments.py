from enviroment import *
from helping import helper
import numpy as np
parameters = helper.paramsFromGUI()


class Experiments(object):

    def __init__(self, showbase):
        self.sb=showbase
        self.obj = Object(showbase)
        self.terrain = Terrain(showbase)
        self.sky = Sky(showbase)

        self.idxArr = self.randIndexArray()
        self.objectPosition = None
        self.instance = None
        self.world = None
        #self.worldFilename = None

    def getObjects(self, objPath, objScale, objTex):
        self.tempObj = self.obj.getObjects(objPath, objScale, objTex)
        return self.tempObj

    def setObjects(self, origin, *objects):

        for idx, obj in enumerate(objects):
            self.obj.setObjects(origin=origin, obj = obj, position = self.objectPosition[idx], instance=self.instance)

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



