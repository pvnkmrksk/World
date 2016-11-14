from enviroment import *
from playerCon import Player
from importHelper import np, helper
parameters = helper.paramsFromGUI()


class Experiments(object):

    def __init__(self, showbase):
        self.sb=showbase
        self.obj = Object(showbase)
        self.terrain = Terrain(showbase)
        self.sky = Sky(showbase)
        self.player = Player()

        self.idxArr = self.randIndexArray()
        print "indexArray: " + str(self.idxArr)
        self.objectPosition = None #list of positions of objects (instances)
        self.instance = None #not neccessary, just in case you want to know
        self.world = None #
        self.newPos = None #player Position
        self.trial = 1 #trial number
        self.tempObjUse = False #Bool, true if one instance/object is copied
        self.tempObj = None #copied instance, will have value if needed
        #self.worldFilename = None

    def getObjects(self, objPath, objScale):
        obj = self.obj.getObjects(objPath, objScale)
        return obj

    def setObjects(self, *objects):

        for idx, obj in enumerate(objects):
            self.obj.moveObjects(position = self.objectPosition[idx], obj=obj)
        self.updateObjPos(objects)
        print "updatedPos:", self.objectPosition

    def randIndexArray(self):
        '''
        creates index with range numObj
        if randPos = True(parameters), shuffles array
        neccessary for random object positioning
        '''
        arr = np.arange(parameters["numObj"])
        if parameters["randPos"] == True:
            np.random.shuffle(arr)
        return arr

    def createTerrain(self, modelHeightMap, modelTextureMapNull, modelTextureMap,loadNullModels,modelSizeSuffix, loadingString):
        '''
        calls initTerrain in enviroment.py
        assigns self.world to created terrain
        '''
        self.world = self.terrain.initTerrain(modelHeightMap, modelTextureMapNull, modelTextureMap, loadNullModels)
        # ?self.world = self.terrain.generate(modelSizeSuffix, loadingString)#todo: is that right? or even neccessary?

    def createSky(self, loadNullModels, skyMapNull,skyMap,maxDistance,humanDisplay):
        '''
          calls  createSky in enviroment.py
               '''
        self.sky.createSky(loadNullModels, skyMapNull, skyMap, maxDistance, humanDisplay)
        
    def resetPosition(self, initH=parameters["playerInitH"], speed=parameters["speed"]):
        self.player.resetPos(self.newPos, initH, speed)
        self.trial += 1
        return self.newPos

    def reachedDestination(self):
        for i in self.objectPosition:
            event = self.player.reachedDestination(i)
            if event is not False:
                return True
        return False

    def updateObjPos(self, obj):
        '''
                        needs tuple of objects
                        writes positions of objects in self.objectPos
                        if object is None, AttributeError assigns None as position

                        '''
        for idx, item in enumerate(obj):
            try:
                pos = item.getPos()
            except AttributeError:
                pos = None
            self.objectPosition[idx] = pos

    def removeObj(self, obj):
        '''
                detaches render nodes from objects
                removes tempObj from scene graph
                if object is None (i.e. Lr: 10), AttributeError
                safe way to prevent any kind of render-bugs (i.e. the rg-gg-bug in Lr)

                '''

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










