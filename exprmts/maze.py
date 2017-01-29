import copy

from classes.experiment import Experiment
from helping.importHelper import * # that is super dirty, please import only needed stuff


class Maze(Experiment):

    def __init__(self, showbase, parameters,objPath1=parameters["object1"],
                 objScale1=parameters["obj1Scale"], loadingString=parameters["loadingString"]):

        super(Maze, self).__init__(showbase,parameters)

        self.createTerrain()
        self.createSky()
        self.obj1 = self.getObjects(objPath1, objScale1)
        self.setObjects(self.obj1)

    def setObjects(self, *objects):
        tempPosList = []
        playerPos = parameters['playerInitPos']
        np.random.seed(0)
        sc=50
        rand=(np.random.rand(parameters['numObj'],2)-0.5)*sc
        print "rand is", rand

        for i in range(parameters['numObj']):
            x = (playerPos[0] + rand[i,0])
            y = (playerPos[1] + rand[i,1])
            z = parameters["obj1Z"]

            pos = (x,y,z)
            self.objectPosition = [pos]
            self.tempObj = copy.copy(objects[0])
            super(Maze, self).setObjects(self.tempObj)
            tempPosList.append(pos)
            parameters['mazePos']=tempPosList
            # self.objectPosition = tempPosList


    # def setObjects(self, *objects):
    #     tempPosList = []
    #     for i in range(5):
    #         p = 0
    #         playerPos = parameters['playerInitPos']
    #
    #         if i%2 == 1:
    #             while p <= 4:
    #
    #                 x = playerPos[0] - 7  + p*4
    #                 y = playerPos[1] - 10 + i*2
    #                 z = parameters["obj1Z"]
    #
    #                 pos = (x,y,z)
    #                 self.objectPosition = [pos]
    #                 self.tempObj = copy.copy(objects[0])
    #                 super(Maze, self).setObjects(self.tempObj)
    #                 tempPosList.append(pos)
    #
    #                 p += 1
    #
    #         elif i%2 == 0:
    #             while p <=5:
    #
    #                 x = playerPos[0] - 8 + p * 4
    #                 y = playerPos[1] - 10 + i * 2
    #
    #                 if x is playerPos[0] and y is playerPos[1]:
    #                     continue
    #                 z = parameters["obj1Z"]
    #
    #                 pos = (x, y, z)
    #                 self.objectPosition = [pos]
    #                 self.tempObj = copy.copy(objects[0])
    #                 super(Maze, self).setObjects(self.tempObj)
    #                 tempPosList.append(pos)
    #
    #
    #                 p += 1
    #         # self.objectPosition = tempPosList
    #
    #



