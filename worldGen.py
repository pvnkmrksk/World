from __future__ import division

from direct.showbase.ShowBase import ShowBase  # import the bits of panda
from panda3d.core import GeoMipTerrain, SamplerState  # that we need
import numpy as np
import matplotlib.pyplot as plt
import json_tricks as json
import sys
import math

from helping.helper import paramsFromGUI
parameters=paramsFromGUI()
if parameters["replayWorld"]:
    import easygui
    import pandas as pd

    # replayPath = easygui.fileopenbox(multiple=False, filetypes=["*.pickle"])
    replayPath="/home/behaviour/catkin/src/beginner/scripts/panda/world/bags/fly4/fly4_quad_rg_gain7.0_speed_3.5_" \
               "trial_1_2016-04-13__23:31:35.bag_df.pickle"

    print replayPath
    df = pd.read_pickle(replayPath)

    #change current parameters to parameters saved in dataframe
    parameters = None

    try:
        parameters = json.loads(df.metadata__data.values[1])

    except:
        parameters = json.loads(df.metadata__data.values[0])
        print "using exceprion"

    parameters["replayWorld"]=True

    print "allooo \n \n \n \n "

class WorldGen(ShowBase):  # our 'class'
    def __init__(self):


        # if __name__ =='__main__':

        ShowBase.__init__(self)  # initialise

        # self.initTerrain()
        self.generate()

        # self.quadPositionGenerator()
        #self.initModels()
        # self.getObjects()
        #self.initPositions()
        # self.setObjPositions()

        # if parameters["loadingString"] == 'circ':
        #     # self.setObjects()
        #     for i in range(0,360,30):
        #         self.setCircPos(dummy=self.redSphere, scale=parameters['sphereScale'], z=3, teta=i, r=50, tex=self.redTex)
        #     print "circ successfull"
        # else:
        #
        #     self.loadingStringParser(parameters["loadingString"])
        #
        # # self.plotPosition()
        #     if self.evenLoad:
        #        self.evenInstance(self.evenObj, scale=self.evenScale, tex=self.evenTex,z=self.evenZ)
        #     if self.oddLoad:
        #        self.oddInstance(self.oddObj, scale=self.oddScale, tex=self.oddTex,z=self.oddZ)

        # print " \n \n \n Generate is about to exit \n \n \n"




        # print "kill is ", parameters["killWorldGen"]
        # parameters["killWorldGen"]=False
        print " \n \n \n World Generate is about to exit \n \n \n"
        sys.exit()

    # def initTerrain(self):
    #     self.terrain = GeoMipTerrain("worldTerrain")  # create a self.terrain
    #     self.terrain.setHeightfield(parameters["modelHeightMap"])  # set the height map
    #     if parameters["loadNullModels"]:#if null, then create uniform back and sky
    #         self.terrain.setColorMap(parameters["modelTextureMapNull"])  # set the colour map
    #     else:
    #             self.terrain.setColorMap(parameters['modelTextureMap'])
    #         # try:
    #         #     self.tex = self.loader.loadTexture(parameters["modelTextureMap"])
    #         #     # self.tex.setMinfilter(SamplerState.FT_linear)
    #         #     # self.tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
    #         #     print "texture successfully loaded"
    #         #
    #         # except:
    #         #     print "Texture loading failed"
    #         # try:
    #         #     self.terrain.setColorMap(self.tex)  # set the colour map
    #         #     print "Texture successfully applied"
    #         # except:
    #         #     print "Applying texture failed"
    #     self.terrain.setBruteforce(True)  # level of detail
    #     self.root = self.terrain.getRoot()  # capture root
    #     self.root.reparentTo(self.render)  # render from root
    #     self.root.setSz(0.2)  # maximum height
    #     self.terrain.generate()  # generate

    #
    # def initModels(self):
    #     self.tree = self.loader.loadModel(parameters["treePath"])
    #     self.greenSphere = self.loader.loadModel(parameters["spherePath"])
    #     self.redSphere = self.loader.loadModel(parameters["spherePath"])
    #
    #     self.treeTex = self.loader.loadTexture(parameters["treeTexPath"])
    #     self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
    #     self.redTex = self.loader.loadTexture(parameters["redTexPath"])


    # def initPositions(self):
    #     self.evenObj = self.oddObj = self.evenScale = self.oddScale = \
    #             self.evenTex = self.oddTex = self.evenLoad = self.oddLoad =self.evenZ=self.oddZ= \
    #             self.evenPlotColor=self.oddPlotColor=self.evenPlotMarker=self.oddPlotMarker=None
    #
    #     if parameters["quad"]:
    #
    #         self.odd,self.even=self.quadPositionGenerator()

        # print "odd is",self.odd
        # print "even is ", self.even


    # def quadPositionGenerator(self):
    #     offset=((int(parameters["worldSize"])-1)/2)+1
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
    #     #identical visual stim
    #
    #     # odd=np.array([quad1PosR,quad2PosR,quad3PosR,quad4PosR])
    #     # even=np.array([quad1PosL,quad2PosL,quad3PosL,quad4PosL])
    #
    #     odd=np.array([quad1PosR,quad2PosL,quad3PosL,quad3PosR])
    #     even=np.array([quad1PosL,quad2PosR,quad4PosL,quad4PosR])
    #
    #     # print offset
    #     # print "even is ",odd
    #     print "even is ", even
    #     return odd,even

    # def loadingStringParser(self, loadingString):
    #     """
    #     Args:
    #         loadingString:2 letter string,
    #         r - red sphere
    #         g - green sphere
    #         t - tree
    #         0 - None
    #
    #         1st letter is even instance
    #         2nd letter is odd instance
    #
    #     Returns:
    #
    #     """
    #     if len(loadingString) > 2:
    #         print "only 2 objects in world"
    #
    #     j = 0
    #     for i in loadingString:
    #
    #         if i == "t":
    #             obj = self.tree
    #             tex = self.treeTex
    #             scale = parameters["treeScale"]
    #             z=parameters["treeZ"]
    #
    #             plotColor="k"
    #             plotMarker="s"
    #             load = True
    #
    #         elif i == "r":
    #             obj = self.redSphere
    #             tex = self.redTex
    #             scale = parameters["sphereScale"]
    #             z=parameters["sphereZ"]
    #
    #             plotColor="r"
    #             plotMarker="o"
    #             load = True
    #         elif i == "g":
    #             obj = self.greenSphere
    #             tex = self.greenTex
    #             scale = parameters["sphereScale"]
    #             z=parameters["sphereZ"]
    #
    #             plotColor="g"
    #             plotMarker="o"
    #             load = True
    #         else:
    #             obj = None
    #             tex = Nonesssss
    #             scale = None
    #             z=None
    #
    #             plotColor=None
    #             plotMarker=None
    #             load = False
    #
    #         if j == 0:
    #             self.evenObj = obj
    #             self.evenTex = tex
    #             self.evenScale = scale
    #             self.evenZ=z
    #
    #             self.evenPlotColor=plotColor
    #             self.evenPlotMarker=plotMarker
    #             self.evenLoad = load
    #         else:
    #             self.oddObj = obj
    #             self.oddTex = tex
    #             self.oddScale = scale
    #             self.oddZ=z
    #
    #             self.oddPlotColor=plotColor
    #             self.oddPlotMarker=plotMarker
    #             self.oddLoad = load
    #         j += 1

    # def oddInstance(self, dummy, scale, tex,z):
    #     dummy.setPos(tuple(parameters["origin"]))
    #     dummy.setScale(scale)
    #     dummy.setTexture(tex)
    #
    #     for i in range(self.odd.shape[0]):
    #         self.oddPlaceholder = self.render.attach_new_node("odd Holder")
    #         self.oddPlaceholder.setPos(self.odd[i][0], self.odd[i][1], z)
    #         dummy.instanceTo(self.oddPlaceholder)
    #
    # def evenInstance(self, dummy, scale, tex,z):
    #     dummy.setPos(tuple(parameters["origin"]))
    #     dummy.setScale(scale)
    #     dummy.setTexture(tex)
    #
    #     for i in range(self.even.shape[0]):
    #         evenPlaceholder = self.render.attach_new_node("even Holder")
    #         evenPlaceholder.setPos(self.even[i][0], self.even[i][1], z)
    #         dummy.instanceTo(evenPlaceholder)

    # def getObjects(self):
    #     if len(parameters['loadingString']) == 2:
    #         if parameters['loadingString'][0] == "1":
    #             self.obj1 = self.loader.loadModel(parameters["spherePath"])
    #             self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
    #             self.obj1.setTexture(self.greenTex)
    #             self.obj1.setScale(parameters['sphereScale'])
    #         else:
    #             self.obj1 = None
    #             print "obj1 None"
    #         if  parameters['loadingString'][1] == "1":
    #             self.obj2 = self.loader.loadModel(parameters["treePath"])
    #             self.redTex = self.loader.loadTexture(parameters["redTexPath"])
    #             self.obj2.setTexture(self.redTex)
    #             self.obj2.setScale(parameters['treeScale'])
    #         else:
    #             self.obj2 = None
    #             print "obj2 None"
    #         return self.obj1, self.obj2
    #     else: #  parameters['loadingString'] == "circ":
    #         self.obj1 = self.loader.loadModel(parameters["spherePath"])
    #         self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
    #         self.obj1.setTexture(self.greenTex)
    #         self.obj1.setScale(parameters['sphereScale'])
    #         print "circ successfull"
    #         return self.obj1

    # def setObjPositions(self):
    #     if len(parameters['loadingString']) == 2:
    #         self.setTwoPos(self.obj1, self.obj2)
    #         print "setTwoPos successful"
    #     elif parameters['loadingString'] == "circ":
    #         self.setCircPos(obj=self.obj1, teta=0, r=50)
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
    #     if not obj1:
    #         pass
    #     else:
    #         self.obj1.setPos(tuple(parameters["origin"]))
    #         for i in range(self.pos1.shape[0]):
    #             placeholder1 = self.render.attach_new_node("holder1")
    #             placeholder1.setPos(self.pos1[i][0], self.pos1[i][1], parameters["sphereZ"])
    #             obj1.instanceTo(placeholder1)
    #
    #     if not obj2:
    #         pass
    #     else:
    #         self.obj2.setPos(tuple(parameters["origin"]))
    #         for i in range(self.pos2.shape[0]):
    #             placeholder2 = self.render.attach_new_node("holder2")
    #             placeholder2.setPos(self.pos2[i][0], self.pos2[i][1], parameters["treeZ"])
    #             obj2.instanceTo(placeholder2)
    #
    #
    # def setCircPos(self, obj, teta, r):
    #     x = parameters['playerInitPos'][0] + (math.sin(math.radians(teta))*r)
    #     y = parameters['playerInitPos'][1] + (math.cos(math.radians(teta))*r)
    #     pos = (x,y,parameters["sphereZ"])
    #     obj.setPos(tuple(parameters['origin']))
    #     placeholder = self.render.attach_new_node("holder")
    #     placeholder.setPos(pos)
    #     obj.instanceTo(placeholder)

    def plotPositions(self):#todo: take care of that
        self.tree=self.greenSphere=self.redSphere=self.treeTex=self.greenTex=self.redTex=None
        self.loadingStringParser(parameters["loadingString"])
        plt.scatter(self.odd[:,0],self.odd[:,1],color=self.oddPlotColor,marker=self.oddPlotMarker,s=80)
        plt.scatter(self.even[:,0],self.even[:,1],color=self.evenPlotColor,marker=self.evenPlotMarker,s=80)#,marker='|',color='g')



    # def worldNameGen(self):
    #     self.worldFilename = "models/world_" + "size:" + parameters["modelSizeSuffix"] \
    #                          + "_obj:"   + parameters["loadingString"] + ".bam"
    #     print "world file name is ",self.worldFilename

    def generate(self):

        # self.worldNameGen()
        self.worldFilename = "models/world_" + "size:" + parameters["modelSizeSuffix"] \
                             + "_obj:" + parameters["loadingString"] + ".bam"
        print "world file name is ", self.worldFilename
        self.render.writeBamFile(self.worldFilename)
            # create 3D model


if __name__ == '__main__':
    app = WorldGen()  # our 'object'
    app.run()
