from experiments import Experiments
from helping import helper
import copy
from importHelper import * # that is super dirty, please import only needed stuff
print "\n\n\n\nwee have imported\n\n\n\n"
class Lr(Experiments):

    def __init__(self, showbase, parameters,objPath1=parameters["object1"], objPath2=parameters["object2"],
                 objScale1=parameters["obj1Scale"], objScale2=parameters["obj2Scale"],
                 loadingString=parameters["loadingString"]):

        super(Lr, self).__init__(showbase,parameters)
        self.idxArr = helper.randIndexArray(4, parameters[
            "randPos"])  # creates the indexArray, which controls order of resetPositions
        print "indexArray: " + str(self.idxArr)
        self.createTerrain()
        self.createSky()
        print "loadingString LR:", parameters["loadingString"]
        print "obj1 LR:", parameters["object1"]
        print "obj2 LR:", parameters["object2"]

        # loads objects dependent on the loading string
        # 11 loads obj1, obj2, 10 loads obj1, None, 01 loads None, obj2, 00 loads None, None



        if loadingString[0] == "1":
            self.obj1 = self.getObjects(objPath1, objScale1)
            print "obj1 loaded"
        else:
            self.obj1 = None
            print "obj1 None"
        if loadingString[1] == "1":
            self.obj2 = self.getObjects(objPath2, objScale2)
            print "obj2 loaded"
        else:
            self.obj2 = None
            print "obj2 None"


        # self.sb.initHardware()
        mfg= FieldGen()
        self.of = mfg.odourQuadField(parameters['worldSize']*2,parameters['worldSize']*2,
                                     oq=parameters['odourQuad'],
                                     plot=parameters['plotOdourQuad'])

        if parameters['useOdourMask']:
            self.omCase = {0:np.rot90(imread(parameters['odour1Mask']),2),
                           1:np.rot90(imread(parameters['odour2Mask']),2),
                           2: np.rot90(imread(parameters['odour3Mask']),2),
                           3: np.rot90(imread(parameters['odour4Mask']),2)}

        xO=parameters['worldSize']
        yO=parameters['worldSize']
        self.ofCase={0:self.of[xO:xO+xO,yO:yO+yO],
                     1:self.of[0:xO,yO:yO+yO],
                     2:self.of[0:xO,0:yO],
                     3:self.of[xO:xO+xO,0:yO]}

        self.setObjects(self.obj1, self.obj2)

    def setObjects(self, *objects):
        """
        overwrites setObjects in experiments.py
        creates object-positions for lr-objects dependent on the case
        case: number from idxArr which determines the lr-configuration (10,01,11,00)
        calls super-method after defining the positions
        None-object-errors (AttributeError) are handled by higher methods (if one object is None)
        :param objects: objects to set/move
        """


        # todo: there are unnecessary variable assignments, clean that
        self.case, self.trial, self.runNum = self.generateCase(self.trial, self.runNum)

        self.pos1 = parameters["posL"]
        self.pos2 = parameters["posR"]
        self.objectPosition = [self.pos1, self.pos2]#at first, objectPosition will always be pos1 and pos2 for flexible
                                                    # object positioning, even if one instance/object is None.
                                                    # Position change to None occurs later in super-method.

        self.removeObj(objects)  # remove tempObj, fixes rg-gg-bug and other render-object-bugs, pass tuple

        # case-dependent object-positioning (10,01,11,00)
        # if config is 11 or 00, need to create tempObj-copy of obj and pass that to super
        # if you pass only the same object twice, super will move that one object twice and not create a second object
        # after creating copy, set tempObjUse True, so removeObj() in experiments.py will remove the copy next time
        # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s
        #todo discuss how to decode this with maraian
        if self.case == 0:
            self.tempObj = copy.copy(objects[0])
            self.tempObjUse = True
            super(Lr, self).setObjects(objects[0], self.tempObj)

        elif self.case == 2:
            super(Lr, self).setObjects(objects[0], objects[1])

        elif self.case == 1:
            super(Lr, self).setObjects(objects[1], objects[0])

        elif self.case == 3:
            self.tempObj = copy.copy(objects[1])
            super(Lr, self).setObjects(objects[1], self.tempObj)
            self.tempObjUse = True
        else:
            print "something wrong in setobject of lr"

# if self.case == 0:
#             super(Lr, self).setObjects(objects[1], objects[0])
#
#         elif self.case == 1:
#             self.tempObj = copy.copy(objects[1])
#             super(Lr, self).setObjects(objects[1], self.tempObj)
#             self.tempObjUse = True
#         elif self.case == 2:
#             super(Lr, self).setObjects(objects[0], objects[1])
#         elif self.case == 3:
#             self.tempObj = copy.copy(objects[0])
#             super(Lr, self).setObjects(objects[0], self.tempObj)
#             self.tempObjUse = True
#         else:
#             print "something wrong in setobject of lr"
#

    def updateOdourField(self):
        self.sb.apple.of=self.ofCase[self.case]
        self.sb.haw.of=self.ofCase[self.case]

        if parameters['useOdourMask']:
            self.sb.apple.om=self.omCase[self.case]
            self.sb.haw.om=self.omCase[self.case]

    def resetPosition(self):
        """
        overwrites resetPosition() in experiments.py
        determines new player position
        calls super
        calls setObjects
        """
        super(Lr, self).resetPosition()
        self.setObjects(self.obj1, self.obj2)
        self.updateOdourField()

    def startExperiment(self):
        """
        overwrites startExperiment() in experiment.py
        calls super (new trial), creates fresh idxArray for setup
        sets Obj Position with new trial and idxArr
        """
        super(Lr, self).startExperiment()
        self.idxArr = helper.randIndexArray(4, parameters["randPos"])
        self.setObjects(self.obj1, self.obj2)


    def retryRun(self, newList=False):
        if newList == True:
            self.idxArr = helper.randIndexArray(4, parameters["randPos"])
        super(Lr, self).retryRun()
        self.setObjects(self.obj1, self.obj2)



