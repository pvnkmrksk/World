from __future__ import division

from direct.showbase.ShowBase import ShowBase  # import the bits of panda
from panda3d.core import GeoMipTerrain  # that we need
import numpy as np
import matplotlib.pyplot as plt
from params import parameters


class MyApp(ShowBase):  # our 'class'
    def __init__(self):
        ShowBase.__init__(self)  # initialise
        terrain = GeoMipTerrain("worldTerrain")  # create a terrain
        terrain.setHeightfield(parameters["modelHeightMap"])  # set the height map
        terrain.setColorMap(parameters["modelTextureMap"])  # set the colour map
        terrain.setBruteforce(True)  # level of detail
        self.root = terrain.getRoot()  # capture root
        self.root.reparentTo(self.render)  # render from root
        self.root.setSz(0)  # maximum height
        terrain.generate()  # generate

        self.evenObj = self.oddObj = self.evenScale = self.oddScale = \
            self.evenTex = self.oddTex = self.evenLoad = self.oddLoad = None

        self.odd, self.even = self.positionListGenerator(heightObjects=parameters["heightObjects"],
                                                         widthObjects=parameters["widthObjects"],
                                                         lattice=parameters["lattice"])

        self.tree = self.loader.loadModel(parameters["treePath"])
        self.greenSphere = self.loader.loadModel(parameters["spherePath"])
        self.redSphere = self.loader.loadModel(parameters["spherePath"])

        self.treeTex = self.loader.loadTexture(parameters["treeTexPath"])
        self.greenTex = self.loader.loadTexture(parameters["greenTexPath"])
        self.redTex = self.loader.loadTexture(parameters["redTexPath"])

        self.loadingStringParser(parameters["loadingString"])

        if self.evenLoad:
            self.evenInstance(self.evenObj, scale=self.evenScale, tex=self.evenTex)
        if self.oddLoad:
            self.oddInstance(self.oddObj, scale=self.oddScale, tex=self.oddTex)

        self.generate()

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
                load = True

            elif i == "r":
                obj = self.redSphere
                tex = self.redTex
                scale = parameters["sphereScale"]
                load = True
            elif i == "g":
                obj = self.greenSphere
                tex = self.greenTex
                scale = parameters["sphereScale"]
                load = True
            else:
                obj = None
                tex = None
                scale = None
                load = False

            if j == 0:
                self.evenObj = obj
                self.evenTex = tex
                self.evenScale = scale
                self.evenLoad = load
            else:
                self.oddObj = obj
                self.oddTex = tex
                self.oddScale = scale
                self.oddLoad = load
            j += 1

    def oddInstance(self, dummy, scale, tex):
        dummy.setPos(parameters["origin"])
        dummy.setScale(scale)
        dummy.setTexture(tex)

        for i in range(self.odd.shape[0]):
            self.oddPlaceholder = self.render.attach_new_node("odd Holder")
            self.oddPlaceholder.setPos(self.odd[i][0], self.odd[i][1], 1)
            dummy.instanceTo(self.oddPlaceholder)

    def evenInstance(self, dummy, scale, tex):
        dummy.setPos(parameters["origin"])
        dummy.setScale(scale)
        dummy.setTexture(tex)

        for i in range(self.even.shape[0]):
            evenPlaceholder = self.render.attach_new_node("even Holder")
            evenPlaceholder.setPos(self.even[i][0], self.even[i][1], 1)
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

        self.oddPosList = self.posList[1::2]
        self.evenPosList = self.posList[0::2]

        return self.oddPosList, self.evenPosList

    def worldNameGen(self):
        self.worldFilename = "models/world_" + "size:" + parameters["modelSizeSuffix"] + "_obj:" \
                             + parameters["loadingString"] + "_num:" + str(parameters["widthObjects"]) \
                             + "x" + str(parameters["heightObjects"]) + ".bam"

        print self.worldFilename

    def generate(self):

        if parameters["generateWorld"]:
            self.worldNameGen()
            self.root.writeBamFile(self.worldFilename)
            # create 3D model


if __name__ == '__main__':
    app = MyApp()  # our 'object'
    app.run()
