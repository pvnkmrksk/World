from direct.showbase.ShowBase import ShowBase   # import the bits of panda
from panda3d.core import GeoMipTerrain          # that we need
worldNameSuffix="257"
height="models/height"+worldNameSuffix+"0.png"
texture="models/texture"+worldNameSuffix+"s.png"
# print height
class MyApp(ShowBase):                          # our 'class'
    def __init__(self):
        ShowBase.__init__(self)                        # initialise
        terrain = GeoMipTerrain("worldTerrain")        # create a terrain
        terrain.setHeightfield(height)        # set the height map
        terrain.setColorMap(texture)           # set the colour map
        terrain.setBruteforce(True)                    # level of detail
        root = terrain.getRoot()                       # capture root
        root.reparentTo(render)                        # render from root
        root.setSz(2)                                 # maximum height
        terrain.generate()                             # generate
        root.writeBamFile('models/world'+worldNameSuffix+".bam")                 # create 3D model

app = MyApp()                                   # our 'object'
app.run()

