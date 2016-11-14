from experiments import Experiments
import math
import numpy as np
from helping import helper
import copy
parameters = helper.paramsFromGUI()

#HU
class Lr(Experiments):

    def __init__(self, showbase,objPath1=parameters["spherePath"], objPath2=parameters["treePath"],
                 objScale1=parameters["sphereScale"], objScale2=parameters["treeScale"],
                 objTex1=parameters["greenTexPath"],objTex2=parameters["redTexPath"],origin=parameters["origin"],modelHeightMap=parameters["modelHeightMap"],
                 modelTextureMapNull=parameters["modelTextureMapNull"], modelTextureMap=parameters["modelTextureMap"],
                 loadNullModels=parameters["loadNullModels"],modelSizeSuffix=parameters["modelSizeSuffix"],
                 loadingString=parameters["loadingString"],skyMapNull=parameters["skyMapNull"],skyMap=parameters["skyMap"],
                 maxDistance=parameters["maxDistance"],humanDisplay=parameters["humanDisplay"]):
        super(Lr, self).__init__(showbase)
        self.createTerrain(modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString)
        self.createSky(loadNullModels,skyMapNull,skyMap,maxDistance,humanDisplay)

        if loadingString[0] == "1":
            self.obj1 = self.getObjects(objPath1, objScale1)#todo: self.obj?

        else:
            self.obj1 = None
            print "obj1 None"
        if  loadingString[1] == "1":
            self.obj2 = self.getObjects(objPath2, objScale2)
        else:
            self.obj2 = None
            print "obj2 None"

        self.setObjects(self.obj1, self.obj2)

    def setObjects(self, *objects):

        try:
            case = self.idxArr[self.trial-1]
            print "trial: ", self.trial
            print "case:", case
        except IndexError:
            self.trial = 1
            self.idxArr = self.randIndexArray()
            case = self.idxArr[self.trial - 1]
            print "case:", case

        self.pos1 = parameters["posL"]
        self.pos2 = parameters["posR"]
        self.objectPosition = [self.pos1, self.pos2]#at first, objectPosition will always be pos1 and pos2 for flexible
                                                    # object positioning, even if one instance/object is None.
                                                    # Position change to None occurs later in super-method.


        self.removeObj(objects)# remove tempObj, fixes rg-gg-bug and other render-object-bugs, pass tuple

        if case == 0:
            self.tempObj = copy.copy(objects[1])
            super(Lr, self).setObjects(objects[1], self.tempObj)
            self.tempObjUse= True
        elif case == 1:
           super(Lr, self).setObjects(objects[1], objects[0])
        elif case == 2:
            super(Lr, self).setObjects(objects[0], objects[1])
        else:
            self.tempObj = copy.copy(objects[0])
            super(Lr, self).setObjects(objects[0], self.tempObj)
            self.tempObjUse = True

    # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s

    def resetPosition(self,initH=parameters["playerInitH"], speed=parameters["speed"]):

        self.newPos = parameters["playerInitPos"]
        super(Lr, self).resetPosition()
        self.setObjects(self.obj1, self.obj2)




