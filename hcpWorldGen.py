from __future__ import division

from direct.showbase.ShowBase import ShowBase  # import the bits of panda
from panda3d.core import GeoMipTerrain  # that we need
import numpy as np
import matplotlib.pyplot as plt
from params import parameters



class WorldGen(ShowBase):  # our 'class'
    def __init__(self):


        if __name__ =='__main__':

            ShowBase.__init__(self)  # initialise

            self.initTerrain()
            self.initModels()
            self.initPositions()
            self.loadingStringParser(parameters["loadingString"])

            # self.plotPosition()
            if self.evenLoad:
                self.evenInstance(self.evenObj, scale=self.evenScale, tex=self.evenTex,z=self.evenZ)
            if self.oddLoad:
                self.oddInstance(self.oddObj, scale=self.oddScale, tex=self.oddTex,z=self.oddZ)

            # print " \n \n \n Generate is about to exit \n \n \n"

            self.generate()

            # print "kill is ", parameters["killWorldGen"]
            # parameters["killWorldGen"]=False
            if parameters["killWorldGen"]:
                import sys
                print " \n \n \n World Generate is about to exit \n \n \n"
                sys.exit()

    def initTerrain(self):
        self.terrain = GeoMipTerrain("worldTerrain")  # create a self.terrain
        self.terrain.setHeightfield(parameters["modelHeightMap"])  # set the height map
        self.terrain.setColorMap(parameters["modelTextureMap"])  # set the colour map
        self.terrain.setBruteforce(True)  # level of detail
        self.root = self.terrain.getRoot()  # capture root
        self.root.reparentTo(self.render)  # render from root
        self.root.setSz(0.2)  # maximum height
        self.terrain.generate()  # generate

    def initModels(self):
        self.tree = self.loader.loadModel(parameters["treePath"])
        self.greenSphere = self.loader.loadModel(parameters["spherePath"])
        self.redSphere = self.loader.loadModel(parameters["spherePath"])

        self.treeTex = self.loader.loadTexture(parameters["treeTexPath"])
        self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
        self.redTex = self.loader.loadTexture(parameters["redTexPath"])



    def plotPositions(self):
        self.tree=self.greenSphere=self.redSphere=self.treeTex=self.greenTex=self.redTex=None
        self.loadingStringParser(parameters["loadingString"])
        plt.scatter(self.odd[:,0],self.odd[:,1],color=self.oddPlotColor,marker=self.oddPlotMarker)
        plt.scatter(self.even[:,0],self.even[:,1],color=self.evenPlotColor,marker=self.evenPlotMarker)#,marker='|',color='g')


    def initPositions(self):

        self.evenObj = self.oddObj = self.evenScale = self.oddScale = \
            self.evenTex = self.oddTex = self.evenLoad = self.oddLoad =self.evenZ=self.oddZ= \
            self.evenPlotColor=self.oddPlotColor=self.evenPlotMarker=self.oddPlotMarker=None

        self.odd, self.even = self.positionListGenerator(heightObjects=parameters["heightObjects"],
                                                         widthObjects=parameters["widthObjects"],
                                                         lattice=parameters["lattice"])


        # print "odd is",self.odd
        # print "even is ", self.even



    def loadingStringParser(self, loadingString):
        """
        Args:
            loadingString:2 letter string,
            r - red sphere
            g - green sphere
            t - tree
            0 - None

            1st letter is even instance
            2nd letter is odd instance

        Returns:

        """
        if len(loadingString) > 2:
            print "only 2 objects in world"

        j = 0
        for i in loadingString:

            if i == "t":
                obj = self.tree
                tex = self.treeTex
                scale = parameters["treeScale"]
                z=parameters["treeZ"]

                plotColor="k"
                plotMarker="s"
                load = True

            elif i == "r":
                obj = self.redSphere
                tex = self.redTex
                scale = parameters["sphereScale"]
                z=parameters["sphereZ"]

                plotColor="r"
                plotMarker="o"
                load = True
            elif i == "g":
                obj = self.greenSphere
                tex = self.greenTex
                scale = parameters["sphereScale"]
                z=parameters["sphereZ"]

                plotColor="g"
                plotMarker="o"
                load = True
            else:
                obj = None
                tex = None
                scale = None
                z=None

                plotColor=None
                plotMarker=None
                load = False

            if j == 0:
                self.evenObj = obj
                self.evenTex = tex
                self.evenScale = scale
                self.evenZ=z

                self.evenPlotColor=plotColor
                self.evenPlotMarker=plotMarker
                self.evenLoad = load
            else:
                self.oddObj = obj
                self.oddTex = tex
                self.oddScale = scale
                self.oddZ=z

                self.oddPlotColor=plotColor
                self.oddPlotMarker=plotMarker
                self.oddLoad = load
            j += 1

    def oddInstance(self, dummy, scale, tex,z):
        dummy.setPos(parameters["origin"])
        dummy.setScale(scale)
        dummy.setTexture(tex)

        for i in range(self.odd.shape[0]):
            self.oddPlaceholder = self.render.attach_new_node("odd Holder")
            self.oddPlaceholder.setPos(self.odd[i][0], self.odd[i][1], z)
            dummy.instanceTo(self.oddPlaceholder)

    def evenInstance(self, dummy, scale, tex,z):
        dummy.setPos(parameters["origin"])
        dummy.setScale(scale)
        dummy.setTexture(tex)

        for i in range(self.even.shape[0]):
            evenPlaceholder = self.render.attach_new_node("even Holder")
            evenPlaceholder.setPos(self.even[i][0], self.even[i][1], z)
            dummy.instanceTo(evenPlaceholder)

    def positionListGenerator(self, heightObjects, widthObjects, lattice):
        self.posList = np.zeros([heightObjects * widthObjects, 2])
        self.alternate = False
        index = 0

        for i in range(0, heightObjects):
            y = i * lattice * (3 ** 0.5) / 2
            self.alternate = not self.alternate

            for j in range(0, widthObjects):
                if self.alternate:
                    x = j * lattice + (lattice / 2)
                else:
                    x = j * lattice

                self.posList[index, :] = [x, y]
                index += 1

        # self.posList[:,0]=np.linspace(0,100,num=(self.posList.shape)[0])
        # self.posList[:,1]=np.linspace(0,100,num=(self.posList.shape)[0])

        self.oddPosList = self.posList[1::2]
        self.evenPosList = self.posList[0::2]

        # print "odd is ",self.oddPosList
        # print "even is", self.evenPosList

        return self.oddPosList, self.evenPosList

    def worldNameGen(self):
        self.worldFilename = "models/world_" + "size:" + parameters["modelSizeSuffix"] \
                             + "_obj:"   + parameters["loadingString"] + "_num:" \
                             + str(parameters["widthObjects"])      + "x" \
                             + str(parameters["heightObjects"])+"_lattice:"\
                             +str(parameters["lattice"]) + ".bam"
        print "world file name is ",self.worldFilename

    def generate(self):

        self.worldNameGen()
        self.render.writeBamFile(self.worldFilename)
            # create 3D model


if __name__ == '__main__':
    app = WorldGen()  # our 'object'
    app.run()
