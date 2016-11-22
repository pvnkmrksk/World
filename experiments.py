from enviroment import *
from playerCon import Player
from importHelper import helper
parameters = helper.paramsFromGUI()


class Experiments(object):

    def __init__(self, showbase):
        self.sb=showbase  # gets ShowBase from MyApp()(can't open 2 ShowBases)
        self.obj = Object(showbase)  # new object-handler from enviroment.py
        self.terrain = Terrain(showbase)  # new terrain-handler from enviroment.py
        self.sky = Sky(showbase)  # new sky- and -light-handler from enviroment.py
        self.player = Player()  # new player-handler from playerCon.py
        self.objectPosition = None  # list of positions of objects
        self.world = None  #the terrain-object, used by player-positioning etc.
        self.newPos = None  # player Position
        self.trial = 1  # trial number, gets increased everytime a reset-event happens
        self.tempObjUse = False  # Bool, true if one instance/object is copied
        self.tempObj = None  # copied instance, will have value if needed

    def getObjects(self, objPath, objScale):
        """
        calls getObjects from enviroment.py
        :param objPath: file path of loaded object
        :param objScale: scale of loaded object
        :return: received object
        """

        obj = self.obj.getObjects(objPath, objScale)
        return obj

    def setObjects(self, *objects):
        """
        calls moveObjects in enviroment.py for every passed object
        updates self.objectPosition after placing. Important if one object is None, objPos will be None at this place
        necessary for reached destination, since reachedDestination works with self.objectPosition
        :param objects: tuple of objects to set
        """

        for idx, obj in enumerate(objects):
            self.obj.moveObjects(position = self.objectPosition[idx], obj=obj)
        self.updateObjPos(objects)
        print "updatedPos:", self.objectPosition



    def createTerrain(self):
        """
        calls initTerrain in enviroment.py
        assigns self.world to created terrain
        """

        self.world = self.terrain.initTerrain()
        # self.world = self.terrain.generate()#todo: is that right? or even necessary?

    def createSky(self):
        """
        calls createSky in enviroment.py
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

        self.player.resetPos(self.newPos)
        self.trial += 1
        return self.newPos

    def retryPosition(self):
        """
        just resets Position without increasing trial
        :return: new player Position (necessary?)
        """
        self.player.resetPos(self.newPos)
        return self.newPos

    def reachedDestination(self):
        """
        calls reachedDestination in playerCon.py
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
        """
        :param: obj, object ro remove/derender
        detaches render nodes from objects
        removes tempObj from scene graph
        if object is None (i.e. Lr: 10), AttributeError
        safe way to prevent any kind of render-bugs (i.e. the rg-gg-bug in Lr)
        """

        if self.tempObjUse == True:
            try:
                self.tempObj.remove_node()
                self.tempObjUse = False
            except AttributeError:
                pass
        for item in obj:
            try:
                item.detachNode()
            except AttributeError:
                pass










