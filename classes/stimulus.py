import numpy as np
import pandas as pd
class Stimulus(object):
    def __init__(self, stimList=[1,2,3,4], nReps=1, mode='ordered', genTimeSeries=False,preStim=0, preStimDur=0, stimDur=1, fps=165, ):
        # pass
        self.stimList=self.stimListGen(np.array(stimList),nReps,mode)
        self.preStim=preStim
        self.stimDur=stimDur
        self.preStimDur=preStimDur
        self.currentIndex=0
        self.fps=fps
        self.currentFrame = 0
        self.genTimeSeries=genTimeSeries
        self.timeSeries=self.tsg()

    def stimDfGen(self,stimList,nReps,mode,genTimeSeries,stimDur,fps):
        self.stimDf=pd.DataFrame(columns=['trial','stimList','timeSeries'])

    def tsg(self):
        ts=self.timeSeriesGen(self.preStim,self.currentStim(),self.preStimDur,self.stimDur,self.fps,self.genTimeSeries)
        return ts

    def stimListGen(self,stimList,nReps,mode):
        if mode=='randomRep':
            stimListRep=[]
            for i in range(nReps):
                np.random.shuffle(stimList)
                stimListRep=np.append(stimListRep,stimList)
        elif mode=='ordered':
            stimListRep=np.tile(stimList,nReps)
        print "stimlist rep is",stimListRep
        return stimListRep

    def timeSeriesGen(self,preStim,stim,preStimDur,stimDur,fps,genTimeSeries):
        if genTimeSeries:
            self.currentFrame=0
            ts = np.append(np.tile(preStim, int(preStimDur * fps)), np.tile(stim, int((stimDur - preStimDur) * fps)))
            return ts
        else:
            return None


    def nextStim(self):
        self.currentIndex+=1
        try:
            self.timeSeries = self.tsg()
        except IndexError:
            print "stim Complete"
            return None
        print "\n\nthe Stim right now is",self.currentStim()
        return self.currentStim()

    def currentStim(self):
        try:
            cs=self.stimList[self.currentIndex]
        except IndexError:
            print 'stim Complete'
            self.currentIndex=0
            return self.currentStim() #recursion baby!
        return cs

    def previousStim(self):
        self.currentIndex-=1
        try:
            self.timeSeries = self.tsg()
        except IndexError:
            print "stim complete"
            return None
        return self.currentStim()


        # pass
    def repeatCurrentStim(self):
        self.timeSeries=self.tsg()

        return self.currentStim()

        # pass
    def restartStim(self):
        self.currentIndex=0
        self.timeSeries=self.tsg()

        return self.currentStim()
        # pass


    def nextStimFrame(self):
        if self.timeSeriesGen:
            try:
                cf=self.timeSeries[self.currentFrame]
                self.currentFrame+=1
            except IndexError:
                self.nextStim()
                cf=self.timeSeries[self.currentFrame]

            return cf
        else:
            return None



if __name__ =='__main__':

    s=Stimulus(nReps=5,mode='randomRep')
    while True:
        n=s.nextStim()
        print n
        if n is None:
            break

