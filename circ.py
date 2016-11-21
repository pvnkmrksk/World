from experiments import Experiments
import math
from helping import helper
parameters = helper.paramsFromGUI()

class Circ(Experiments):

    def __init__(self, showbase,objPath1=parameters["spherePath"], objScale1=parameters["sphereScale"]):

        super(Circ, self).__init__(showbase)
        self.createTerrain()
        self.createSky()

        self.obj1 = self.getObjects(objPath1, objScale1)
        self.setObjects(self.obj1)

    def setObjects(self, *objects):

        try:
            case = self.idxArr[self.trial-1]
            print "trial: ", self.trial
            print "case:", case
        except IndexError:
            self.trial = 1
            self.idxArr = helper.randIndexArray(parameters["numObj"], parameters["randPos"])  # numObj+1 because of
                                                                                                # negative control
            print "new run"
            print "indexArray: " + str(self.idxArr)
            case = self.idxArr[self.trial - 1]
            print "case:", case

        # if case == max(self.idxArr):  # negative control

            # self.removeObj((self.obj1,))  # Always pass tuple to removeObj
        else:

            radius = parameters["radius"]
            teta = 360 / parameters["numObj"]
            phi = parameters["phi"]
            playerPos = parameters['playerInitPos']


            x = playerPos[0] + (math.sin(math.radians(teta * case + phi)) * radius)
            y = playerPos[1] + (math.cos(math.radians(teta * case + phi)) * radius)
            z= parameters["sphereZ"]

            self.temp = (x, y, z)
            self.objectPosition = [self.temp]



            # ATTENTION! objects is a tuple, don't pass the tuple to super fct! pass the object/s
            super(Circ, self).setObjects(objects[0])

    def resetPosition(self):
        self.newPos = parameters["playerInitPos"]
        super(Circ, self).resetPosition()
        self.setObjects(self.obj1)




