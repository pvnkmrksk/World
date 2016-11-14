from helping.helper import paramsFromGUI
import numpy as np
import math
parameters=paramsFromGUI()

class ObjectControl():
    # class of experiment-class
    def __init__(self,showbase):
        self.sb=showbase
    #     self.objectPosition = None
    #
    # def getObjects(self):
    #
    #     if len(parameters['loadingString']) == 2:
    #         if parameters['loadingString'][0] == "1":
    #             self.obj1 = self.sb.loader.loadModel(parameters["spherePath"])
    #             self.greenTex = self.sb.loader.loadTexture(parameters["greenTexPath"])
    #             self.obj1.setTexture(self.greenTex)
    #             self.obj1.setScale(parameters['sphereScale'])
    #         else:
    #             self.obj1 = None
    #             print "obj1 None"
    #         if  parameters['loadingString'][1] == "1":
    #             self.obj2 = self.sb.loader.loadModel(parameters["treePath"])
    #             self.redTex = self.sb.loader.loadTexture(parameters["redTexPath"])
    #             self.obj2.setTexture(self.redTex)
    #             self.obj2.setScale(parameters['treeScale'])
    #         else:
    #             self.obj2 = None
    #             print "obj2 None"
    #         self.objTup = (self.obj1, self.obj2)
    #         return self.objTup
    #     else:
    #         self.obj1 = self.sb.loader.loadModel(parameters["spherePath"])
    #         self.greenTex = self.sb.loader.loadTexture(parameters["greenTexPath"])
    #         self.obj1.setTexture(self.greenTex)
    #         self.obj1.setScale(parameters['sphereScale'])
    #         self.objTup = (self.obj1,)
    #         return self.objTup
    #
    #
    # def setObjPositions(self, fac):
    #     if len(parameters['loadingString']) == 2:
    #         self.setTwoPos(self.obj1, self.obj2)
    #         print "setTwoPos successful"
    #     elif parameters['loadingString'] == "circ":
    #         self.setCircPos(obj=self.obj1, fac=fac)
    #         print "setCirc successful"
    #
    # def setTwoPos(self, obj1, obj2):
    #     offset = ((int(parameters["worldSize"]) - 1) / 2) + 1
    #
    #     quad3PosL=parameters["posL"]
    #     quad3PosR=parameters["posR"]
    #
    #     quad4PosL=(parameters["posL"][0]+offset,parameters["posL"][1])
    #     quad4PosR=(parameters["posR"][0]+offset,parameters["posR"][1])
    #
    #     quad2PosL=(parameters["posL"][0],parameters["posL"][1]+offset)
    #     quad2PosR=(parameters["posR"][0],parameters["posR"][1]+offset)
    #
    #     quad1PosL=(parameters["posL"][0]+offset,parameters["posL"][1]+offset)
    #     quad1PosR=(parameters["posR"][0]+offset,parameters["posR"][1]+offset)
    #
    #     self.pos1 = np.array([quad1PosR,quad2PosL,quad3PosL,quad3PosR])
    #     self.pos2 = np.array([quad1PosL,quad2PosR,quad4PosL,quad4PosR])
    #
    #     if not obj1:
    #         pass
    #     else:
    #         self.obj1.setPos(tuple(parameters["origin"]))
    #         for i in range(self.pos1.shape[0]):
    #             placeholder1 = self.sb.render.attach_new_node("holder1")
    #             placeholder1.setPos(self.pos1[i][0], self.pos1[i][1], parameters["sphereZ"])
    #             obj1.instanceTo(placeholder1)
    #
    #     if not obj2:
    #         pass
    #     else:
    #         self.obj2.setPos(tuple(parameters["origin"]))
    #         for i in range(self.pos2.shape[0]):
    #             placeholder2 = self.sb.render.attach_new_node("holder2")
    #             placeholder2.setPos(self.pos2[i][0], self.pos2[i][1], parameters["treeZ"])
    #             obj2.instanceTo(placeholder2)
    #     self.pos = np.append(self.pos1, self.pos2, axis=0)
    #     self.objectPosition = self.pos
    #
    # def setCircPos(self, obj, fac):
    #
    #     r = parameters["radius"]
    #     teta = 360 / parameters["numObj"]
    #     phi = parameters["phi"]
    #     x = parameters['playerInitPos'][0] + (math.sin(math.radians(teta * fac+phi)) * r)
    #     y = parameters['playerInitPos'][1] + (math.cos(math.radians(teta * fac+phi)) * r)
    #     self.temp = (x,y,parameters["sphereZ"])
    #     self.pos = np.array([self.temp])
    #     obj.setPos(tuple(parameters['origin']))
    #     self.placeholder = self.sb.render.attach_new_node("holder")
    #     self.placeholder.setPos(self.pos[0][0],self.pos[0][1],self.pos[0][2])
    #     obj.instanceTo(self.placeholder)
    #     self.objectPosition = self.pos
    #
    # def moveObj(self, obj, fac):
    #
    #     r = parameters["radius"]
    #     teta = 360/parameters["numObj"]
    #     phi=parameters["phi"]
    #     x = parameters['playerInitPos'][0] + (math.sin(math.radians(teta*fac+phi)) * r)
    #     y = parameters['playerInitPos'][1] + (math.cos(math.radians(teta*fac+phi)) * r)
    #     self.temp = (x, y, parameters["sphereZ"])
    #     self.pos = np.array([self.temp])
    #     obj.setPos(tuple(parameters['origin']))
    #     self.placeholder.setPos(self.pos[0][0], self.pos[0][1], self.pos[0][2])
    #     obj.instanceTo(self.placeholder)
    #     self.objectPosition = self.pos#todo: maybe one step too much, but is this more explicit?

