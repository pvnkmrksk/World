import numpy as np
import pandas as pd
class Stimulus(object):
    def __init__(self, stimList=[1,2,3,4], nReps=1, mode='ordered', genTimeSeries=False,preStim=0, preStimDur=0, stimDur=1, fps=165, ):
        # pass
        self.stimList=self.stimListGen(np.array(stimList),nReps,mode)
        self.preStim=preStim
        self.stimDur=stimDur
        self.preStimDur=preStimDur
        self.fps=fps
        self.genTimeSeries=genTimeSeries

        self.currentIndex=0
        self.currentFrame = 0
        self.prevStimState=None
        self.tsg()#timeSeriesGen and assignment

    def stimDfGen(self,stimList,nReps,mode,genTimeSeries,stimDur,fps):
        self.stimDf=pd.DataFrame(columns=['trial','stimList','timeSeries'])

    def tsg(self):
        '''
        timeSeriesGen tsg calls the standard tsg with the standard params and assigns to object variables
        Returns:

        '''
        self.timeSeries,self.timeSeriesState=self.timeSeriesGen(self.preStim,self.currentStim(),self.preStimDur,self.stimDur,self.fps,self.genTimeSeries)

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
            doubleWhammy=True

            ps=np.tile(preStim, int(preStimDur * fps))
            if doubleWhammy:
                f=20
                d=0.055
                ps=np.append(ps,np.tile(f,int(fps*d)))


            ts = np.append(ps, np.tile(stim, int((stimDur) * fps)))

            #timeseries state is -1 during pre stim and stimpf during stimulus
            tsState = np.append(np.tile(-1, int(preStimDur * fps)), np.tile(stim, int((stimDur) * fps)))

            return ts,tsState
        else:
            return None,None


    def nextStim(self):
        self.currentIndex+=1
        try:
            self.tsg()
        except IndexError:
            print "stim Complete"
            raise
        print "\n\nthe Stim right now is",self.currentStim()
        return self.currentStim()

    def currentStim(self):
        try:
            cs=self.stimList[self.currentIndex]
        except IndexError:
            print 'cstim Complete'
            raise #sends a reset event exception upostream
            self.currentIndex=0
            return self.currentStim() #recursion baby!
        return cs

    def previousStim(self):
        self.currentIndex-=1
        try:
            self.tsg()
        except IndexError:
            print "stim complete"
            return None
        return self.currentStim()


        # pass
    def repeatCurrentStim(self):
        self.tsg()

        return self.currentStim()

        # pass
    def restartStim(self):
        self.currentIndex=0
        self.tsg()

        return self.currentStim()
        # pass


    def nextStimFrame(self):
        if self.timeSeriesGen:
            try:
                cf=self.timeSeries[self.currentFrame]
                self.stimState=self.timeSeriesState[self.currentFrame]

                #
                if self.prevStimState!=-1 and self.stimState==-1:
                    self.prevStimState = self.stimState
                    raise NextPreStimStartException

                elif self.prevStimState==-1 and self.stimState!=-1:
                    self.prevStimState = self.stimState #reset it now, else will be in endlessloop of raised exception
                    raise NextStimStartException

                self.prevStimState=self.stimState
                self.currentFrame+=1

            except IndexError:
                self.nextStim()
                cf=self.timeSeries[self.currentFrame]
                # raise NextStimStartException

            return cf
        else:
            return None

class NextStimStartException(Exception):
    pass
class NextPreStimStartException(Exception):
    pass

if __name__ =='__main__':

    s=Stimulus(nReps=5,mode='randomRep')
    while True:
        n=s.nextStim()
        print n
        if n is None:
            break

