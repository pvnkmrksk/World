from experiments import Experiments
import math
import numpy as np
from helping import helper
parameters = helper.paramsFromGUI()

class Circ(Experiments):
    def __init__(self, showbase,objPath, objScale, objTex,origin,modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString,skyMapNull,skyMap,maxDistance,humanDisplay):
        super(Circ, self).__init__(showbase)
        self.createTerrain(modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString)
        #self.createSky(loadNullModels,skyMapNull,skyMap,maxDistance,humanDisplay)
        self.obj1 = self.getObjects(objPath, objScale, objTex)
        self.setObjects(origin,self.obj1)

    def setObjects(self, origin, *objects):

        radius = parameters["radius"]
        teta = 360 / parameters["numObj"]
        phi = parameters["phi"]
        fac = self.idxArr[0]
        playerPos = parameters['playerInitPos']


        x = playerPos[0] + (math.sin(math.radians(teta * fac + phi)) * radius)
        y = playerPos[1] + (math.cos(math.radians(teta * fac + phi)) * radius)
        z= parameters["sphereZ"]

        self.temp = (x, y, z)
        self.pos = np.array([self.temp])

        self.instance = self.sb.render.attach_new_node("holder")
        self.objectPosition = self.pos

    # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s
        super(Circ, self).setObjects(origin, objects[0])

