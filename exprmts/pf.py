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
                               postStim=parameters["pfPostStim"],postStimDur=parameters["pfPostStimDur"],
                               fps=parameters['fps'],genTimeSeries=parameters["pfGenTimeSeries"],mode=parameters['pfMode'])
        # self.stimulus=Stimulus(stimList=[0,1/4,1,4,16],nReps=1,preStimDur=5,stimDur=10,fps=parameters['fps'],genTimeSeries=True)
        self.loadOdour=True #overide parameters for odour
        self.sb.overRidePf=True
        if parameters['useOdourMask']:
            self.initField()
        # self.obj1 = self.getObjects(objPath1, objScale1)
        # self.setObjects(self.obj1)

    def frameUpdateTask(self):
        if self.isFlying:
            try:
                self.sb.pf=self.stimulus.nextStimFrame()
            except IndexError:
                self.resetPosition()
                # self.frameUpdateTask()
            except NextStimStartException:
                #update phase of tunnels to zero so that it fires at start of stim.
                self.sb.odour1.phase = 0
                self.sb.odour2.phase = 0
            except NextPreStimStartException:
                super(Pf,self).resetPosition()
        else:
            #turn off valves if not flying
            self.sb.pf=0
            # print "i am not flying",self.stimulus.currentFrame


    def startExperiment(self):
        print "starting now"

        #set it to closed loop
        self.sb.keyMap["closed"]=1

        #reset init pos and stim conditions
        self.resetPosition()

        #set runum and trial to init values
        self.runNum = 1
        self.trial = 1

        # prevent initial prestimexception
        self.prevStimState = -1

        #prevent reset from happening
        self.sb.reset=False

        #prevent initial open loop,closed loop, closed loop with speed being used
        self.sb.decayTime=-1

    def resetPosition(self):
        super(Pf,self).resetPosition()
        self.runNum+=1
        self.stimulus.currentIndex=0
        self.stimulus.currentFrame=0
        self.stimulus.tsg()
        self.case=self.stimulus.currentIndex

        if parameters['useOdourMask']:
            self.updateOdourField()


    def retryPosition(self):
        self.stimChange()
        self.stimulus.repeatCurrentStim()

    def prevStim(self):
        self.stimChange()
        self.stimulus.previousStim()

    def nextStim(self):
        self.stimChange()
        self.stimulus.nextStim()

    def currStim(self):
        self.retryPosition()

    def stimChange(self):
        super(Pf,self).resetPosition()
        self.trial-=1
        self.sb.maxBoutDur = 0
        self.case=self.stimulus.currentIndex

