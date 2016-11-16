import numpy as np
class Stimulus(object):
    def __init__(self, stimList=[1,2,3,4], nReps=1, mode='ordered', genTimeSeries=False, stimDur=1, fps=165, ):
        # pass
        self.stimList=self.stimListGen(np.array(stimList),nReps,mode)
        self.stimDur=stimDur
        self.currentIndex=-1
        self.fps=fps
        self.currentFrame = 0
        self.genTimeSeries=genTimeSeries
        self.timeSeries=self.timeSeriesGen(self.currentStim(),self.stimDur,self.fps,self.genTimeSeries)


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

    def timeSeriesGen(self,stim,stimDur,fps,genTimeSeries):
        if genTimeSeries:
            self.currentFrame=0
            return np.tile(stim,int(stimDur*fps))
        else:
            return None

    def nextStim(self):
        self.currentIndex+=1
        try:
            self.timeSeries=self.timeSeriesGen(self.currentStim(),self.stimDur,self.fps,self.genTimeSeries)
        except IndexError:
            print "stim Complete"
            return None
        print self.currentStim()
        return self.currentStim()
    def previousStim(self):
        self.currentIndex-=1
        try:
            self.timeSeries=self.timeSeriesGen(self.currentStim(),self.stimDur,self.fps,self.genTimeSeries)
        except IndexError:
            print "stim complete"
            return None
        return self.currentStim()


        # pass
    def repeatCurrentStim(self):
        self.timeSeries=self.timeSeriesGen(self.currentStim(),self.stimDur,self.fps,self.genTimeSeries)

        return self.currentStim()

        # pass
    def restartStim(self):
        self.currentIndex=0
        self.timeSeries=self.timeSeriesGen(self.currentStim(),self.stimDur,self.fps,self.genTimeSeries)

        return self.currentStim()
        # pass

    def currentStim(self):
        return self.stimList[self.currentIndex]
    def nextFrame(self):
        if self.timeSeriesGen:
            try:
                cf=self.timeSeries[self.currentFrame]
                self.currentFrame+=1
            except IndexError:
                self.nextStim()
                cf=self.timeSeries[self.currentFrame]

            return cf


if __name__ =='__main__':

    s=Stimulus(nReps=5,mode='randomRep')
    while True:
        n=s.nextStim()
        print n
        if n is None:
            break

