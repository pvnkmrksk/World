from experiments import Experiments
import math
import numpy as np
from helping import helper
parameters = helper.paramsFromGUI()

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
        #self.createSky(loadNullModels,skyMapNull,skyMap,maxDistance,humanDisplay)

        if loadingString[0] == "1":
            self.obj1 = self.getObjects(objPath1, objScale1, objTex1)
        else:
            self.obj1 = None
            print "obj1 None"
        if  loadingString[1] == "1":
            self.obj2 = self.getObjects(objPath2, objScale2, objTex2)
        else:
            self.obj2 = None
            print "obj2 None"

        self.setObjects(origin, self.obj1, self.obj2)

    def setObjects(self, origin=parameters["origin"], *objects):

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

        #todo: work on object positioning

        if self.firstRun == True:
            instance1 = self.sb.render.attach_new_node("holder1")
            instance2 = self.sb.render.attach_new_node("holder2")
            self.instance = (instance1, instance2)
            self.firstRun = False

        if case == 0:
            self.pos = (self.pos1, None)
            self.objectPosition = self.pos
            super(Lr, self).setObjects(origin,objects[0], objects[1])
            print "ObjectPos:", self.objectPosition
            print "instance:", self.instance
        elif case == 1:
            self.pos = (None, self.pos2)
            self.objectPosition = self.pos
            super(Lr, self).setObjects(origin,objects[1], objects[0])
            print "ObjectPos:", self.objectPosition
            print "instance:", self.instance
        elif case == 2:
            self.pos = (self.pos1, self.pos2)
            self.objectPosition = self.pos
            super(Lr, self).setObjects(origin,objects[0], objects[0])
            print "ObjectPos:", self.objectPosition
            print "instance:", self.instance
        else:
            self.pos = (None, None)
            self.objectPosition = self.pos
            super(Lr, self).setObjects(origin,objects[1], objects[1])
            print "ObjectPos:", self.objectPosition
            print "instance:", self.instance



    # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s


    def resetPosition(self,initH=parameters["playerInitH"], speed=parameters["speed"]):

        self.newPos = parameters["playerInitPos"]
        super(Lr, self).resetPosition()

        # try:
        #     case = self.idxArr[self.trial-1]
        #     print "trial: " + str(self.trial)
        #     print "case:", case
        # except IndexError:
        #     self.trial = 1
        #     self.idxArr = self.randIndexArray()
        #     case = self.idxArr[self.trial - 1]
        #     print "case:", case


        self.hideObject(self.instance[0], self.instance[1])
        self.setObjects(parameters["origin"], self.obj1, self.obj2)

        # if case == 0:
        #     self.setObjects(parameters["origin"], self.obj1, self.obj2)
        #     print "ObjectPos:", self.objectPosition
        #     print "instance:", self.instance
        # elif case == 1:
        #     self.setObjects(parameters["origin"], self.obj2, self.obj1)
        #     print "ObjectPos:", self.objectPosition
        #     print "instance:", self.instance
        # elif case == 2:
        #     self.setObjects(parameters["origin"], self.obj1, self.obj1)
        #     print "ObjectPos:", self.objectPosition
        #     print "instance:", self.instance
        # else:
        #     self.setObjects(parameters["origin"], self.obj2, self.obj2)
        #     print "ObjectPos:", self.objectPosition
        #     print "instance:", self.instance




