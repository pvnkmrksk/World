from experiments import Experiments
from helping import helper
from panda3d.core import Shader
from classes import stimulus
parameters = helper.paramsFromGUI()

class Gain(Experiments):

    def __init__(self, showbase, objPath1=parameters["object1"], objScale1=parameters["obj1Scale"]):

        super(Gain, self).__init__(showbase)

        #todo. change after experiment softcode stimlist
        self.stimGain = stimulus.Stimulus(stimList=[1,2,4,8,16,32])

        self.sb.gain = self.stimGain.stimList[self.trial-1]
        print "gain is: ", self.sb.gain
        self.createTerrain()
        self.createSky()
        self.obj1 = self.getObjects(objPath1, objScale1)
        self.obj1.setShaderAuto()
        self.objectPosition = [(parameters['playerInitPos'][0], parameters['playerInitPos'][1] + parameters['radius'],
                                parameters['obj1Z'])]

        self.setObjects(self.obj1)

    def resetPosition(self):

        super(Gain, self).resetPosition()
        try:
            self.sb.gain = self.stimGain.stimList[self.trial-1]

            print "new gain:", self.sb.gain
        except IndexError:
            # if self.sb.gain is None:
            self.sb.stopBag()
            self.sb.gain = parameters['gain']
            print "gainList is completed"

    def startExperiment(self):
        super(Gain, self).startExperiment()
        self.sb.gain = self.stimGain.stimList[0]




