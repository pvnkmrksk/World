from __future__ import division
from experiments import Experiments
from helping import helper
import copy
from importHelper import * # that is super dirty, please import only needed stuff


class Pf(Experiments):

    def __init__(self, showbase, parameters,objPath1=parameters["object1"],
                 objScale1=parameters["obj1Scale"], loadingString=parameters["loadingString"]):

        super(Pf, self).__init__(showbase,parameters)
        self.createTerrain()
        self.createSky()
        self.stimulus=Stimulus(stimList=[0,1/4,1,4,16],nReps=1,preStimDur=2,stimDur=10,fps=parameters['fps'],genTimeSeries=True)
        self.loadOdour=True #overide parameters for odour

        # self.obj1 = self.getObjects(objPath1, objScale1)
        # self.setObjects(self.obj1)

    def frameUpdateTask(self):
        self.sb.pf=self.stimulus.nextStimFrame()
        self.sb.overRidePf=True

    def resetPosition(self):
        super(Pf,self).resetPosition()
