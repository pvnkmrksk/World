from direct.showbase.ShowBase import ShowBase   # import the bits of panda
from panda3d.core import GeoMipTerrain          # that we need
worldNameSuffix="64"
height="models/height"+worldNameSuffix+".png"
print height
class MyApp(ShowBase):                          # our 'class'
    def __init__(self):
        ShowBase.__init__(self)                        # initialise
        terrain = GeoMipTerrain("worldTerrain2")        # create a terrain
        terrain.setHeightfield("models/height0.png")        # set the height map
        terrain.setColorMap("models/texture0.png")           # set the colour map
        terrain.setBruteforce(True)                    # level of detail
        root = terrain.getRoot()                       # capture root
        root.reparentTo(render)                        # render from root
        root.setSz(65)                                 # maximum height
        terrain.generate()                             # generate
        root.writeBamFile('world'+worldNameSuffix+".bam")                 # create 3D model

app = MyApp()                                   # our 'object'
app.run()

