from experiments import Experiments
import math
from helping import helper
from panda3d.core import Shader
parameters = helper.paramsFromGUI()

class Circ(Experiments):

    def __init__(self, showbase,objPath1=parameters["object1"], objScale1=parameters["obj1Scale"]):

        super(Circ, self).__init__(showbase)
        self.idxArr = helper.randIndexArray(parameters["numObj"]+1, parameters[
            "randPos"])  # creates the indexArray, which controls order of resetPositions
                         # numObj+1 because of negative control, has to be number of positions +1 in GUI
        print "indexArray: " + str(self.idxArr)
        self.createTerrain()
        self.createSky()
        self.obj1 = self.getObjects(objPath1, objScale1)
        self.obj1.setShaderAuto()
        self.setObjects(self.obj1)

    def setObjects(self, *objects):
        """
        overwrites setObjects in experiments.py
        creates object-positions for circ-object dependent on the case
        case: number from idxArr which determines the circ-position
        calls super-method after defining the position
        :param objects: objects to set/move
        """

        try:
            # get case from idxArr, dependent on trial number
            case = self.idxArr[self.trial-1]
            print "trial: ", self.trial
            print "case:", case
        except IndexError:
            # if trial is higher than count of digits in idxArr, reset trial and create new idxArr
            # happens if player went through all circ-positions and negative control
            self.trial = 1
            self.runNum += 1
            self.idxArr = helper.randIndexArray(parameters["numObj"]+1, parameters["randPos"])  # +1 because negControl
            print "new run"
            print "indexArray: " + str(self.idxArr)
            case = self.idxArr[self.trial - 1]
            print "case:", case

        if case == max(self.idxArr):  # negative control
            self.removeObj(objects)  # Always pass tuple to removeObj
            self.objectPosition = [None]
        else:
        # creates the circ-position with radius and angle relative to playerInit
        # phi is phase ("offset" for special positions)
            radius = parameters["radius"]
            teta = 360 / (parameters["numObj"])
            phi = parameters["phi"]
            playerPos = parameters['playerInitPos']

            # converts radius-angle-position to x/y-coordinates
            x = playerPos[0] + (math.sin(math.radians(teta * case + phi)) * radius)
            y = playerPos[1] + (math.cos(math.radians(teta * case + phi)) * radius)
            z= parameters["obj1Z"]

            self.temp = (x, y, z)
            self.objectPosition = [self.temp]

        # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s
        super(Circ, self).setObjects(objects[0])

    def resetPosition(self):
        """
        overwrites resetPosition() in experiments.py
        determines new player position
        calls super
        calls setObjects
        """
        super(Circ, self).resetPosition()
        self.setObjects(self.obj1)

    def startExperiment(self):
        """
        overwrites startExperiment() in experiment.py
        calls super (new trial), creates fresh idxArray for setup
        sets Obj Position with new trial and idxArr
        """
        super(Circ, self).startExperiment()
        self.idxArr = helper.randIndexArray(parameters["numObj"]+1, parameters["randPos"])
        self.setObjects(self.obj1)



