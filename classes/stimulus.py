import numpy as np
class Stimulus(object):
    def __init__(self, stimList=[1,2,3,4], nReps=1, mode='ordered', timeSeries=False, stimDur=1, fps=165, ):
        # pass
        self.stimList=self.stimListGen(np.array(stimList),nReps,mode)
        self.stimDur=stimDur
        self.currentIndex=0
        self.timeSeries=timeSeries
        self.fps=fps

    def stimListGen(self,stimList,nReps,mode):
        if mode=='randomRep':
            for i in range(nReps):
                stimListRep=np.append(stimListRep,np.random.shuffle(stimList))
        elif mode=='ordered':
            stimListRep=np.tile(stimList,nReps)
        print "stimlist rep is",stimListRep
        return stimListRep

    def nextStim(self):
        self.currentIndex+=1
        return self.currentStim()

    def previousStim(self):
        self.currentIndex-=1
        return self.currentStim()


        # pass
    def repeatCurrentStim(self):
        return self.currentStim()

        # pass
    def restartStim(self):
        self.currentIndex=0
        return self.currentStim()
        # pass
    def currentStim(self):
        return self.stimList[self.currentIndex]

    def nextFrame(self):
        if self.timeSeries:
            pass

    def pauseStim(self):
        pass

        pass
    def playStim(self):
        pass
    def stopStim(self):
        pass

s=Stimulus()
s.nextStim()