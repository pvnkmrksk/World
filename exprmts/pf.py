from __future__ import division

from helping.libsOnly import *
from classes.experiment import Experiment
from helping.importHelper import * # that is super dirty, please import only needed stuff


class Pf(Experiment):

    def __init__(self, showbase, parameters,objPath1=parameters["object1"],
                 objScale1=parameters["obj1Scale"], loadingString=parameters["loadingString"]):

        super(Pf, self).__init__(showbase,parameters)
        self.createTerrain()
        self.createSky()
        self.stimulus=Stimulus(stimList=parameters["pfStimList"],nReps=parameters["pfNReps"],
                               preStimDur=parameters["pfPreStimDur"],stimDur=parameters["pfStimDur"],
                               fps=parameters['fps'],genTimeSeries=parameters["pfGenTimeSeries"])
        # self.stimulus=Stimulus(stimList=[0,1/4,1,4,16],nReps=1,preStimDur=5,stimDur=10,fps=parameters['fps'],genTimeSeries=True)
        self.loadOdour=True #overide parameters for odour

        # self.obj1 = self.getObjects(objPath1, objScale1)
        # self.setObjects(self.obj1)

    def frameUpdateTask(self):
        try:
            self.sb.pf=self.stimulus.nextStimFrame()
        except IndexError:
            self.resetPosition()
            self.frameUpdateTask()
        except NextStimStartException:
            #update phase of tunnels to zero so that it fires at start of stim.
            self.sb.haw.phase = 0
            self.sb.apple.phase = 0
        except NextPreStimStartException:
            super(Pf,self).resetPosition()

        self.sb.overRidePf=True


    def startExperiment(self):
        print "starting now"
        super(Pf,self).startExperiment()
        self.resetPosition()

    def resetPosition(self):
        super(Pf,self).resetPosition()
        self.runNum+=1
        self.stimulus.currentIndex=0
        self.stimulus.currentFrame=0
        self.stimulus.tsg()
