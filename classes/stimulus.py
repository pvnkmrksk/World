import numpy as np
class Stimulus():
    def __init__(self):
        pass
    def stimulusListGen(self,nSteps,mode,constantArea,area,stopDur,startDur,stepDur,factorDur,areaMode,startHeading,durListGen,gpMode,headingListGen,interTrial,fps,intraTrial,signFlip,orderFlip,nReps ):
        '''
        """
        If stepMode is true, the stimulus will have than many steps
         Else it calculates, the number of steps needed to reach Start to stop in step ratio of factorDur

        If areaMode is true, the stimulus will all have area equal to area given
         Else it calculates area from startHeading rate and startDuration

        If durListGen is true, it generates the list of durations, else the entered list of durations is used
            In gp Mode, the list forms a gp with common ratio of factorDur
            In ap Mode, the list forms a ap with common difference of stepDur

        If headingListGen is true, it generates the list of heading rates, else the entered list is used
            It generates it by dividing durlist form area, this is because, area, which is total rotation imposed every stimulus is kept constant (ex, 720degrees)



        Returns:

        """

        Args:
            stepMode:  the stimulus will have nSteps steps
            Else it calculates, the number of steps needed to reach Start to stop in step ratio of factorDur

            nSteps: number of steps in the sweep if stepMode is True
            stopDur:
            startDur:
            stepDur:
            factorDur:
            areaMode:
            startHeading:
            durListGen:
            gpMode:
            headingListGen:
            interTrial:
            fps:
            intraTrial:
            signFlip:
            orderFlip:
            nReps:

        Returns:

        '''
        # print "startDur is", nSteps


        if not constantArea:
            area = startHeading * startDur

        if nSteps == 'auto':
            # todo.fix this calculates steps for gp mode. Missing ap mode
            if mode == 'gp':
                nSteps = np.log10(stopDur / startDur) / np.log10(factorDur)
        else:
            nSteps -= 1  # because power estimation will increment it because factor^0 is also included

        if durListGen:
            if mode=='gp':
                # print "GP mode is True"
                durList = np.array([startDur * factorDur ** x
                                    for x in range(int(nSteps + 1))])
            elif mode=='ap':
                durList = np.array(range(startDur, stopDur, stepDur))
                # print "GP mode is False"


        if headingListGen:
            headingRate = area / durList
            print "Imposed turn in degrees/frame is,", durList * headingRate

        print "\nDurlist is", durList
        print "Heading is", headingRate

        i = 0
        timeSeries = (np.zeros(int(interTrial * fps)))

        for dur in durList:
            try:
                assert len(durList) == len(
                    headingRate)  # they should be equal unless there is a bug or typo

            except AssertionError:
                print "gain and dur of smae lengty"

            timeSeries = np.append(timeSeries,
                                                 headingRate[i] * np.ones(int(dur * fps)))
            timeSeries = np.append(timeSeries,
                                                 np.zeros(int(intraTrial * fps)))

            if signFlip:
                timeSeries = np.append(timeSeries,
                                                     -headingRate[i] * np.ones(
                                                         int(dur * fps)))
                timeSeries = np.append(timeSeries,
                                                     np.zeros(int(intraTrial * fps)))

            i += 1

        timeSeries = np.append(timeSeries,
                                             np.zeros(int(interTrial * fps)))

        if orderFlip:
            timeSeries = np.append(timeSeries, np.flipud(timeSeries))
        timeSeries = np.tile(timeSeries, nReps)

        # plt.plot(timeSeries)
        # plt.show()


        return timeSeries