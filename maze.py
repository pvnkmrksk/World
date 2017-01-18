from experiments import Experiments
from helping import helper
import copy
from importHelper import * # that is super dirty, please import only needed stuff
class Maze(Experiments):

    def __init__(self, showbase, parameters,objPath1=parameters["object1"],
                 objScale1=parameters["obj1Scale"], loadingString=parameters["loadingString"]):

        super(Maze, self).__init__(showbase,parameters)

        self.createTerrain()
        self.createSky()
        self.obj1 = self.getObjects(objPath1, objScale1)
        self.setObjects(self.obj1)

    def setObjects(self, *objects):
        tempPosList = []
        for i in range(21):
            p = 0
            playerPos = parameters['playerInitPos']

            if i%2 == 1:
                while p <= 16:

                    x = playerPos[0] - 30  + p*4
                    y = playerPos[1] - 20 + i*2
                    z = parameters["obj1Z"]

                    pos = (x,y,z)
                    self.objectPosition = [pos]
                    self.tempObj = copy.copy(objects[0])
                    super(Maze, self).setObjects(self.tempObj)
                    tempPosList.append(pos)

                    p += 1

            elif i%2 == 0:
                while p <=17:

                    x = playerPos[0] - 32 + p * 4
                    y = playerPos[1] - 20 + i * 2

                    if x is playerPos[0] and y is playerPos[1]:
                        continue
                    z = parameters["obj1Z"]

                    pos = (x, y, z)
                    self.objectPosition = [pos]
                    self.tempObj = copy.copy(objects[0])
                    super(Maze, self).setObjects(self.tempObj)
                    tempPosList.append(pos)


                    p += 1
            # self.objectPosition = tempPosList





