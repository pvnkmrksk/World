from panda3d.core import GeoMipTerrain
from importHelper import *

class Sky():

    def __init__(self, showbase):
        self.sb = showbase

    def createSky(self, loadNullModels, skyMapNull,skyMap,maxDistance,humanDisplay):
        """
        load fog
        load sky
        setup lights
        Returns:

        """
        # Fog to hide a performance tweak:
        #What is happening?
        colour = (0.0, 0.0, 0.0)
        expfog = Fog("scene-wide-fog")
        expfog.setColor(*colour)
        expfog.setExpDensity(0.004)
        self.sb.render.setFog(expfog)
        self.sb.setBackgroundColor(*colour)

        # Our sky
        if parameters["loadNullModels"]:  # if null, then create uniform back and sky
            self.skysphere = self.sb.loader.loadModel(parameters["skyMapNull"])
        else:
            self.skysphere = self.sb.loader.loadModel(parameters["skyMap"])

        self.skysphere.setEffect(CompassEffect.make(self.sb.render))
        self.skysphere.setScale(parameters["maxDistance"])  # bit less than "far"
        self.skysphere.setZ(-3)
        # NOT render - you'll fly through the sky!:


        # Our lighting
        # ambientLight = AmbientLight("ambientLight")
        # ambientLight.setColor(Vec4(.6, .6, .6, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(Vec3(-1, -1, -1))
        directionalLight.setColor(Vec4(1, 1, 1, 1))
        directionalLight.setSpecularColor(Vec4(1, 1, 1, 1))

        directionalLight2 = DirectionalLight("directionalLight")
        directionalLight2.setDirection(Vec3(-1, 1, -1))
        directionalLight2.setColor(Vec4(1, 1, 1, 1))
        directionalLight2.setSpecularColor(Vec4(1, 1, 1, 1))
        directionalLight3 = DirectionalLight("directionalLight")
        directionalLight3.setDirection(Vec3(1, -1, -1))
        directionalLight3.setColor(Vec4(1, 1, 1, 1))
        directionalLight3.setSpecularColor(Vec4(1, 1, 1, 1))

        directionalLight4 = DirectionalLight("directionalLight")
        directionalLight4.setDirection(Vec3(1, 1, -1))
        directionalLight4.setColor(Vec4(1, 1, 1, 1))
        directionalLight4.setSpecularColor(Vec4(1, 1, 1, 1))

        # render.setLight(render.attachNewNode(ambientLight))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight2))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight3))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight4))


class Terrain():

    def __init__(self, showbase):
        self.sb = showbase

    def initTerrain(self, modelHeightMap, modelTextureMapNull, modelTextureMap, loadNullModels=False,):
        # self.terrain = GeoMipTerrain("worldTerrain")  # create a self.terrain
        # self.terrain.setHeightfield(modelHeightMap)  # set the height map
        # if loadNullModels:  # if null, then create uniform back and sky
        #     self.terrain.setColorMap(modelTextureMapNull)  # set the colour map
        # else:
        #     self.terrain.setColorMap(modelTextureMap)
        #     # try:
        #     #     self.tex = self.loader.loadTexture(parameters["modelTextureMap"])
        #     #     # self.tex.setMinfilter(SamplerState.FT_linear)
        #     #     # self.tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
        #     #     print "texture successfully loaded"
        #     #
        #     # except:
        #     #     print "Texture loading failed"
        #     # try:
        #     #     self.terrain.setColorMap(self.tex)  # set the colour map
        #     #     print "Texture successfully applied"
        #     # except:
        #     #     print "Applying texture failed"
        # self.terrain.setBruteforce(True)  # level of detail
        # self.root = self.terrain.getRoot()  # capture root
        # self.root.reparentTo(self.sb.render)  # render from root
        # self.root.setSz(0.2)  # maximum height
        # self.terrain.generate()  # generate
        self.environ = self.sb.loader.loadModel(parameters["modelTextureMap"])
        self.environ.reparentTo(self.sb.render)
        self.environ.setPos(0, 0, 0)
        return self.environ

    def generate(self, modelSizeSuffix, loadingString):

        self.worldFilename = "models/world_" + "size:" + modelSizeSuffix \
                             + "_obj:" + loadingString + ".bam"
        print "world file name is ", self.worldFilename
        self.sb.render.writeBamFile(self.worldFilename)
        self.world = self.sb.loader.loadModel(self.worldFilename)  # loads the world_size
        self.world.reparentTo(self.sb.render)
        return self.world
            # create 3D model

class Object():

    def __init__(self, showbase):
        self.sb = showbase

    def getObjects(self, objPath, objScale):

        self.obj = self.sb.loader.loadModel(objPath)
        self.obj.setScale(objScale)
        return self.obj


    def moveObjects(self, position, obj):

        try:
            obj.setPos(position[0], position[1], position[2])
            obj.reparentTo(self.sb.render)
        except AttributeError:
            pass


