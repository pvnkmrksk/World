from importHelper import *

class Sky():

    def __init__(self, showbase):
        self.sb = showbase

    def createSky(self, loadNullModels=parameters["loadNullModels"], skyMapNull=parameters["skyMapNull"],skyMap=parameters["skyMap"],
                 maxDistance=parameters["maxDistance"],humanDisplay=parameters["humanDisplay"]):
        """
        load fog
        load sky
        setup lights
        Returns:

        """
        # Fog to hide a performance tweak:
        # What is happening?
        colour = (0.0, 0.0, 0.0)
        expfog = Fog("scene-wide-fog")
        expfog.setColor(*colour)
        expfog.setExpDensity(0.004)
        self.sb.render.setFog(expfog)
        self.sb.setBackgroundColor(*colour)

        # Our sky
        if loadNullModels:  # if null, then create uniform back and sky
            self.skysphere = self.sb.loader.loadModel(skyMapNull)
        else:
            self.skysphere = self.sb.loader.loadModel(skyMap)

        self.skysphere.setEffect(CompassEffect.make(self.sb.render))
        self.skysphere.setScale(maxDistance)  # bit less than "far"
        self.skysphere.setZ(-3)
        self.skysphere.reparentTo(self.sb.render)
        # NOT render - you'll fly through the sky!:




class Terrain():
    """
    handles the terrain
    """

    def __init__(self, showbase):
        self.sb = showbase
        # Our lighting
        self.ambientLight = AmbientLight("ambientLight")
        self.ambientLight.setColor(Vec4(0.3, 0.3, 0.3, 1))
        self.ambientNode = self.sb.render.attachNewNode(self.ambientLight)
        #
        self.directionalLight = DirectionalLight("directionalLight")
        self.directionalLight.setDirection(Vec3(-1, 1, -1))
        self.directionalLight.setColor(Vec4(1, 1, 1, 1))
        self.directionalLight.setSpecularColor(Vec4(1, 1, 1, 1))
        self.directNode = self.sb.render.attachNewNode(self.directionalLight)
        #
        directionalLight2 = DirectionalLight("directionalLight2")
        directionalLight2.setDirection(Vec3(-1, 1, -1))
        directionalLight2.setColor(Vec4(1, 1, 1, 1))
        directionalLight2.setSpecularColor(Vec4(1, 1, 1, 1))
        # #
        # directionalLight3 = DirectionalLight("directionalLight3")
        # directionalLight3.setDirection(Vec3(1, -1, -1))
        # directionalLight3.setColor(Vec4(1, 1, 1, 1))
        # directionalLight3.setSpecularColor(Vec4(1, 1, 1, 1))
        #
        # directionalLight4 = DirectionalLight("directionalLight4")
        # directionalLight4.setDirection(Vec3(1, 1, -1))
        # directionalLight4.setColor(Vec4(1, 1, 1, 1))
        # directionalLight4.setSpecularColor(Vec4(1, 1, 1, 1))
        #
        self.sb.render.setLight(self.ambientNode)
        self.sb.render.setLight(self.directNode)
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight2))
        # self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight3))
        # self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight4))

    def initTerrain(self, modelHeightMap = parameters["modelHeightMap"], modelTextureMapNull = parameters["modelTextureMapNull"],
                    modelTextureMap = parameters["modelTextureMap"], loadNullModels = parameters["loadNullModels"], worldSize = parameters["worldSize"]):
        """
        loads terrain-model, translates it and dumps it into bam-file
        :param modelHeightMap:
        :param modelTextureMapNull:
        :param modelTextureMap: filepath of terrain-model
        :param loadNullModels:
        :param worldSize: length/width of terrain-model, important for shift-translate
        :return:
        """
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

        # todo: fix the terrain shift bug
        # terrain shift bug: origin of .egg-file from blender is centerpoint, not 0/0/0-corner
        # because of that, terrain is shifted relative to player
        # problem of translating terrain: myApp line 433 in playerLoader
        # self.player.setPos(self.ex.world, position) will set position relative to terrain origin, which is centerpoint
        # so: 129/129/3 will be 258/258/3 etc.
        # solutions:
        # 1. below, load terrain, translate it, write bam file which will have the correct origin, load that
        # 2. translate terrain and don't set positions relative to terrain
        shift = ((worldSize-1)/2)+1
        print "shift:", shift
        tempTerrain = self.sb.loader.loadModel(modelTextureMap)
        tempTerrain.setPos(shift, shift, 0)
        tempTerrain.reparentTo(self.sb.render)
        self.sb.render.writeBamFile("models/testgrass.bam")  # todo: use better filename
        tempTerrain.removeNode()
        self.terrain = self.sb.loader.loadModel("models/testgrass.bam")
        self.terrain.setPos(0, 0, 0)
        self.terrain.reparentTo(self.sb.render)
        self.terrain.setShaderAuto()  # enable per pixel rendering for normal mapping

        return self.terrain

    def generate(self, modelSizeSuffix = parameters["modelSizeSuffix"], loadingString = parameters["loadingString"]):
        # todo: this seems to be the same like initTerrain with one extra step, overthink that. Maybe cool to autogenerate complex worlds
        self.worldFilename = "models/world_" + "size:" + modelSizeSuffix \
                             + "_obj:" + loadingString + ".bam"
        print "world file name is ", self.worldFilename
        self.sb.render.writeBamFile(self.worldFilename)
        self.worldModel = self.sb.loader.loadModel(self.worldFilename)  # loads the world_size
        self.worldModel.reparentTo(self.sb.render)
        return self.worldModel
            # create 3D model

class Object():
    """
    handles Objects in VR
    """

    def __init__(self, showbase):
        self.sb = showbase

    def getObjects(self, objPath, objScale):
        """
        loads Model from object path
        scales object
        :param objPath: filepath of object model
        :param objScale: scale factor for object
        :return: scaled object
        """

        obj = self.sb.loader.loadModel(objPath)
        obj.setScale(objScale)
        return obj

    def moveObjects(self, position, obj):
        """
        sets new position for passed object and rerenders it
        rerender maybe not necessary at this place, but prevents bugs
        AttributeError if object is None
        :param position: tuple of position (x/y/z)
        :param obj: object to move
        """

        try:
            obj.setPos(position[0], position[1], position[2])
            obj.reparentTo(self.sb.render)
            obj.setShaderAuto()# here?

        except AttributeError:
            print "obj is None"
        except TypeError:
            print "position is messed up!" \
                  "check is objectPosition is defined correctly (tuple inside list)"


