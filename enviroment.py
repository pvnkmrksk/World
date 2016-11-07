from panda3d.core import GeoMipTerrain

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
        if loadNullModels:  # if null, then create uniform back and sky
            skysphere = self.sb.loader.loadModel(skyMapNull)
        else:
            skysphere = self.sb.loader.loadModel(skyMap)

        skysphere.setEffect(self.sb.CompassEffect.make(self.sb.render))
        skysphere.setScale(maxDistance)  # bit less than "far"
        skysphere.setZ(-3)
        # NOT render - you'll fly through the sky!:
        if humanDisplay:
            skysphere.reparentTo(self.sb.camera)
        else:
            skysphere.reparentTo(self.sb.cameraCenter)

        # Our lighting
        # ambientLight = AmbientLight("ambientLight")
        # ambientLight.setColor(Vec4(.6, .6, .6, 1))
        directionalLight = self.sb.DirectionalLight("directionalLight")
        directionalLight.setDirection(self.sb.Vec3(-1,-1,-1))
        directionalLight.setColor(self.sb.Vec4(1, 1, 1, 1))
        directionalLight.setSpecularColor(self.sb.Vec4(1, 1, 1, 1))

        directionalLight2 = self.sb.DirectionalLight("directionalLight")
        directionalLight2.setDirection(self.sb.Vec3(-1,1,-1))
        directionalLight2.setColor(self.sb.Vec4(1, 1, 1, 1))
        directionalLight2.setSpecularColor(self.sb.Vec4(1, 1, 1, 1))
        directionalLight3 = self.sb.DirectionalLight("directionalLight")
        directionalLight3.setDirection(self.sb.Vec3(1,-1,-1))
        directionalLight3.setColor(self.sb.Vec4(1, 1, 1, 1))
        directionalLight3.setSpecularColor(self.sb.Vec4(1, 1, 1, 1))

        directionalLight4 = self.sb.DirectionalLight("directionalLight")
        directionalLight4.setDirection(self.sb.Vec3(1,1,-1))
        directionalLight4.setColor(self.sb.Vec4(1, 1, 1, 1))
        directionalLight4.setSpecularColor(self.sb.Vec4(1, 1, 1, 1))

        # render.setLight(render.attachNewNode(ambientLight))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight2))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight3))
        self.sb.render.setLight(self.sb.render.attachNewNode(directionalLight4))
        # directionalLight.setShadowCaster(True, 512, 512)
        # render.setShaderAuto()


class Terrain():

    def __init__(self, showbase):
        self.sb = showbase

    def initTerrain(self, modelHeightMap, modelTextureMapNull, modelTextureMap, loadNullModels=False,):  # todo fix grass problem
        self.terrain = GeoMipTerrain("worldTerrain")  # create a self.terrain
        self.terrain.setHeightfield(modelHeightMap)  # set the height map
        if loadNullModels:  # if null, then create uniform back and sky
            self.terrain.setColorMap(modelTextureMapNull)  # set the colour map
        else:
            self.terrain.setColorMap(modelTextureMap)
            # try:
            #     self.tex = self.loader.loadTexture(parameters["modelTextureMap"])
            #     # self.tex.setMinfilter(SamplerState.FT_linear)
            #     # self.tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
            #     print "texture successfully loaded"
            #
            # except:
            #     print "Texture loading failed"
            # try:
            #     self.terrain.setColorMap(self.tex)  # set the colour map
            #     print "Texture successfully applied"
            # except:
            #     print "Applying texture failed"
        self.terrain.setBruteforce(True)  # level of detail
        self.root = self.terrain.getRoot()  # capture root
        self.root.reparentTo(self.sb.render)  # render from root
        self.root.setSz(0.2)  # maximum height
        self.terrain.generate()  # generate

    def generate(self, modelSizeSuffix, loadingString):

        # self.worldNameGen()
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

    def getObjects(self, objPath, objScale, objTex):

        self.obj = self.sb.loader.loadModel(objPath)
        self.tex = self.sb.loader.loadTexture(objTex)
        self.obj.setTexture(self.tex)
        self.obj.setScale(objScale)
        return self.obj

    def setObjects(self, origin, obj, position, instance):

        obj.setPos(tuple(origin))
        instance.setPos(position[0], position[1], position[2])
        obj.instanceTo(instance)

    def moveObjects(self):

        pass

