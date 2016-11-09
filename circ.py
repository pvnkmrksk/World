from experiments import Experiments
import math
import numpy as np
from helping import helper
parameters = helper.paramsFromGUI()

class Circ(Experiments):

    def __init__(self, showbase,objPath1=parameters["spherePath"], objPath2=parameters["treePath"],
                 objScale1=parameters["sphereScale"], objScale2=parameters["treeScale"],
                 objTex1=parameters["greenTexPath"],objTex2=parameters["redTexPath"],origin=parameters["origin"],modelHeightMap=parameters["modelHeightMap"],
                 modelTextureMapNull=parameters["modelTextureMapNull"], modelTextureMap=parameters["modelTextureMap"],
                 loadNullModels=parameters["loadNullModels"],modelSizeSuffix=parameters["modelSizeSuffix"],
                 loadingString=parameters["loadingString"],skyMapNull=parameters["skyMapNull"],skyMap=parameters["skyMap"],
                 maxDistance=parameters["maxDistance"],humanDisplay=parameters["humanDisplay"]):
        super(Circ, self).__init__(showbase)
        self.createTerrain(modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString)
        #self.createSky(loadNullModels,skyMapNull,skyMap,maxDistance,humanDisplay)
        self.obj1 = self.getObjects(objPath1, objScale1, objTex1)
        self.setObjects(origin, self.obj1)

    def setObjects(self, origin, *objects):

        radius = parameters["radius"]
        teta = 360 / parameters["numObj"]
        phi = parameters["phi"]
        fac = self.idxArr[self.trial-1]
        playerPos = parameters['playerInitPos']


        x = playerPos[0] + (math.sin(math.radians(teta * fac + phi)) * radius)
        y = playerPos[1] + (math.cos(math.radians(teta * fac + phi)) * radius)
        z= parameters["sphereZ"]

        self.temp = (x, y, z)
        self.pos = (self.temp,)
        self.objectPosition = self.pos

        if self.firstRun == True:
            instance = self.sb.render.attach_new_node("holder")
            self.instance = (instance,)
            self.firstRun = False

    # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s
        super(Circ, self).setObjects(origin, objects[0])

    def resetPosition(self,initH=parameters["playerInitH"], speed=parameters["speed"]):
        self.newPos = parameters["playerInitPos"]
        super(Circ, self).resetPosition()

        print "trial: " + str(self.trial)
        try:
            self.setObjects(parameters["origin"], self.obj1)
        except IndexError:
            self.trial = 1
            self.idxArr = self.randIndexArray()
            print "new run"
            print "indexArray: " + str(self.idxArr)
            self.setObjects(parameters["origin"], self.obj1)



