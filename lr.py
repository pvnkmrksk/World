from experiments import Experiments
from helping import helper
import copy
parameters = helper.paramsFromGUI()

class Lr(Experiments):

    def __init__(self, showbase, objPath1=parameters["object1"], objPath2=parameters["object2"],
                 objScale1=parameters["obj1Scale"], objScale2=parameters["obj2Scale"],
                 loadingString=parameters["loadingString"]):
        super(Lr, self).__init__(showbase)
        self.idxArr = helper.randIndexArray(4, parameters[
            "randPos"])  # creates the indexArray, which controls order of resetPositions
        print "indexArray: " + str(self.idxArr)
        self.createTerrain()
        self.createSky()

        # loads objects dependent on the loading string
        # 11 loads obj1, obj2, 10 loads obj1, None, 01 loads None, obj2, 00 loads None, None

        if loadingString[0] == "1":
            self.obj1 = self.getObjects(objPath1, objScale1)
        else:
            self.obj1 = None
            print "obj1 None"
        if loadingString[1] == "1":
            self.obj2 = self.getObjects(objPath2, objScale2)
        else:
            self.obj2 = None
            print "obj2 None"

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

        try:
            # get case from idxArr, dependent on trial number
            self.case = self.idxArr[self.trial-1]
            print "trial: ", self.trial
            print "case:", self.case
        except IndexError:
            # if trial is higher than count of digits in idxArr, reset trial and create new idxArr
            # happens if player went through all 4 lr-configurations
            print "new run"
            self.trial = 1
            self.runNum += 1
            print "runNum:", self.runNum
            self.idxArr = helper.randIndexArray(parameters["numObj"], parameters["randPos"])
            print "idxArr:", self.idxArr
            self.case = self.idxArr[self.trial - 1]
            print "trial:", self.trial
            print "case:", self.case

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
        if self.case == 0:
            super(Lr, self).setObjects(objects[1], objects[0])

        elif self.case == 1:
            self.tempObj = copy.copy(objects[1])
            super(Lr, self).setObjects(objects[1], self.tempObj)
            self.tempObjUse = True
        elif self.case == 2:
            super(Lr, self).setObjects(objects[0], objects[1])
        else:
            self.tempObj = copy.copy(objects[0])
            super(Lr, self).setObjects(objects[0], self.tempObj)
            self.tempObjUse = True

    def resetPosition(self):
        """
        overwrites resetPosition() in experiments.py
        determines new player position
        calls super
        calls setObjects
        """
        super(Lr, self).resetPosition()
        self.setObjects(self.obj1, self.obj2)

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



