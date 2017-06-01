from environment import *
from helping.importHelper import helper
from player import Player


class Experiment(object):

    def __init__(self, showbase, parameters):

        parameters = parameters
        self.sb=showbase  # gets ShowBase from MyApp()(can't open 2 ShowBases)
        self.obj = Object(showbase)  # new object-handler from environment.py
        self.terrain = Terrain(showbase)  # new terrain-handler from environment.py
        self.sky = Sky(showbase)  # new sky- and -light-handler from environment.py
        self.player = Player(showbase)  # new player-handler from player.py
        self.objectPosition = None  # list of positions of objects
        self.world = None  # the terrain-object, used by player-positioning etc.
        self.newPos = parameters["playerInitPos"]  # player Position
        self.trial = 1  # trial number, gets increased every time a reset-event happens
        self.tempObjUse = False  # Bool, true if one instance/object is copied
        self.tempObj = None  # copied instance, will have value if needed
        self.runNum = 1
        self.case = 0
        self.isFlying = True
        self.replay = parameters["replayWorld"]
        self.loadOdour=False

    def getObjects(self, objPath, objScale):
        """
        calls getObjects from environment.py
        :param objPath: file path of loaded object
        :param objScale: scale of loaded object
        :return: received object
        """

        obj = self.obj.getObjects(objPath, objScale)
        return obj

    def setObjects(self, *objects):
        """
        calls moveObjects in environment.py for every passed object
        updates self.objectPosition after placing. Important if one object is None, objPos will be None at this place
        necessary for reached destination, since reachedDestination works with self.objectPosition
        :param objects: tuple of objects to set
        """

        for idx, obj in enumerate(objects):
            self.obj.moveObjects(position=self.objectPosition[idx], obj=obj)
        self.updateObjPos(objects)
        print "updatedPos:", self.objectPosition



    def createTerrain(self):
        """
        calls initTerrain in environment.py
        assigns self.world to created terrain
        """

        self.world = self.terrain.initTerrain()
        from direct.gui.OnscreenImage import OnscreenImage
        #self.patch1 = OnscreenImage(image='models/patch.png', pos=(-1.23, 0, -0.113),scale=(0.479,1,0.176))
        #self.patch2 = OnscreenImage(image='models/patch.png', pos=(1.23, 0, -0.113),scale=(0.479,1,0.176))

        # self.world = self.terrain.generate()#todo: is that right? or even necessary?

    def createSky(self):
        """
        calls createSky in environment.py
        """

        self.sky.createSky()

    def resetPosition(self):
        """
        initH: heading of player
        speed: do I have to explain that?
        calls resetPos in playerCon
        increases self.trial
        :return: new player Position (necessary?)
        """

        if not self.replay:
            print 'i am getting reset'
            self.player.resetPos(self.newPos)
            self.trial += 1
            return self.newPos
        else:
            return

    def retryPosition(self):
        """
        just resets Position without increasing trial
        :return: new player Position (necessary?)
        """
        self.player.resetPos(self.newPos)
        self.sb.boutFrame = 0 #todo.remove redundant right?
        return self.newPos

    def retryRun(self, newList=False):
        self.trial = 1
        self.player.resetPos(self.newPos)

    def reachedDestination(self):
        """
        calls reachedDestination in player.py
        gets called in myApp.py
        checks every frame if player is in bbox at objPos
        :return: True in case player is at objPos, False if not
        """

        for i in self.objectPosition:
            event = self.player.reachedDestination(i)
            if event is not False:
                return True
        return False

    def updateObjPos(self, obj):
        """
        needs tuple of objects
        writes positions of objects in self.objectPos
        if object is None, AttributeError assigns None as position
        """

        for idx, item in enumerate(obj):
            try:
                pos = item.getPos()
            except AttributeError:
                pos = None
            self.objectPosition[idx] = pos

    def removeObj(self, obj):
        # todo: call this in experiments setObjects? and not in experiment classes?

        """
        :param: obj, object ro remove/derender
        detaches render nodes from objects
        removes tempObj from scene graph
        if object is None (i.e. Lr: 10), AttributeError
        safe way to prevent any kind of render-bugs (i.e. the rg-gg-bug in Lr)
        """

        if self.tempObjUse:
            try:
                self.tempObj.remove_node()
                self.tempObjUse = False
            except AttributeError:
                self.tempObjUse = False
                print "tempObj None at removeObj"

        for item in obj:
            try:
                item.detachNode()
            except AttributeError:
                print "Obj None at removeObj"

    def startExperiment(self):
        """
        resets trial and runNum to 1
        creates new idxArr
        sets playerPos to init
        object positioning occurs in circ, lr...
        bag recording starts in myApp
        everything is ready for an experiment!

        """
        self.trial = 1
        self.runNum = 1
        self.player.resetPos(self.newPos)
        # self.sb.decayTime=-1


    def badFly(self):
        self.isFlying = False
        self.sb.maxBoutDur = 0
        print "isFlying:", self.isFlying

    def goodFly(self):
        self.isFlying = True
        self.sb.maxBoutDur = parameters["maxBoutDur"]
        self.retryPosition()
        print "isFlying:", self.isFlying

    def generateCase(self, trial, runNum, addArrNum=0):
        """

        Args:
            trial: pass the current trial number
            runNum: pass the current runNum
            addArrNum: value to add to randIndexArr if number of cases has to be bigger than number of objects
            (ie. negative control in Circ)
        Gets the new case out of self.idxArr, depending on the current trial. If the array is finished, it sets
        trial to 1, increases the runNum, creates a new idxArr and gets the new case out of that new array.
        If world is in replay mode, it only prints and returns the stored values.

        Returns: new case old/new trial, old/new runNum

        """

        if not self.replay:
            try:
                # get case from idxArr, dependent on trial number
                case = self.idxArr[trial-1]
                print "runNum:", runNum

                print "trial: ", trial
                print "case:", case
                return case, trial, runNum
            except IndexError:
                # if trial is higher than count of digits in idxArr, reset trial and create new idxArr
                # happens if player went through all 4 lr-configurations
                print "new run"
                trial = 1
                runNum += 1
                print "runNum:", runNum
                self.idxArr = helper.randIndexArray(parameters["numObj"]+addArrNum, parameters["randPos"])
                print "idxArr:", self.idxArr
                case = self.idxArr[trial - 1]
                print "trial:", trial
                print "case:", case
                return case, trial, runNum
        else:
            case = self.case
            trial = self.trial
            runNum = self.runNum
            print "runNum:", runNum
            print "trial: ", trial
            print "case:", case

            return case, trial, runNum


    # def replayUpdate(self, case, trial, runNum):
    #     self.case = case
    #     self.trial = trial
    #     self.runNum = runNum


    def frameUpdateTask(self):
        '''
        Does nothing at root, can be overrided if needed,
        Will ideally be called in updateTask to run once a frame
        Returns:

        '''
        pass

    def prevStim(self):
        '''go to previous case'''
        pass
    def nextStim(self):
        '''go to next case'''
        pass
    def currStim(self):
        '''repeat curr stim'''
        pass
    def initField(self):
        mfg= FieldGen()
        xO = parameters['worldSize']
        yO = parameters['worldSize']

        if parameters['loadOdour']:
            self.of = mfg.odourQuadField(parameters['worldSize']*2,parameters['worldSize']*2,
                                         oq=parameters['odourQuad'],
                                         plot=parameters['plotOdourQuad'])


            self.ofCase={0:self.of[xO:xO+xO,yO:yO+yO],
                         1:self.of[0:xO,yO:yO+yO],
                         2:self.of[0:xO,0:yO],
                         3:self.of[xO:xO+xO,0:yO]}


            if parameters['useOdourMask']:
                self.omCase = {0:np.rot90(imread(parameters['odour1Mask']),3),
                               1:np.rot90(imread(parameters['odour2Mask']),3),
                               2: np.rot90(imread(parameters['odour3Mask']),3),
                               3: np.rot90(imread(parameters['odour4Mask']),3)}
                self.sb.omCase=self.omCase

        else:
            self.of=self.omCase=self.ofCase=None

        if parameters['loadWind']:
            self.wf=mfg.windField(parameters['worldSize']*2,parameters['worldSize']*2,
                                         wq=parameters['windQuad'],
                                         plot=parameters['plotOdourQuad'])
            self.wfCase={0:self.wf[xO:xO+xO,yO:yO+yO],
                         1:self.wf[0:xO,yO:yO+yO],
                         2:self.wf[0:xO,0:yO],
                         3:self.wf[xO:xO+xO,0:yO]}

        else:
            self.wf=self.wfCase=None



    def updateOdourField(self):
        try:
            if parameters['loadOdour']:
                self.sb.odour2.of = self.ofCase[self.case]
                self.sb.odour1.of = self.ofCase[self.case]

                if parameters['useOdourMask']:
                    self.sb.odour2.om = self.omCase[self.case]
                    self.sb.odour1.om = self.omCase[self.case]
        except KeyError:
            print "only 4 odour quad given, FIX IT"
    def updateWindField(self):
        if parameters['loadWind']:
            self.sb.windField=self.wfCase[self.case]







