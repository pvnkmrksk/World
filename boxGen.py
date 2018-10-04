import direct.directbase.DirectStart
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import Texture, TextureStage, DirectionalLight, AmbientLight, TexGenAttrib, VBase4
from panda3d.core import TransformState, VBase3

import PIL.Image
import easygui
import glob

trans = dict(ft=[0, 90], bk=[1, -90], rt=[2, 0], lf=[3, 180], up=[4, 0], dn=[5, 180])


def renamer(dirPath=None, trans=dict(ft=[0, 90], bk=[1, -90], rt=[2, 0], lf=[3, 180], up=[4, 0], dn=[5, 180])):
    if dirPath is None:
        # dirPath = easygui.diropenbox()
        dirPath='/home/rhagoletis/catkin/src/World/ame_siege/'
        # dirPath='/home/rhagoletis/catkin/src/World/ame_desert/'


    fileList = glob.glob(dirPath + '/*.tga')
    for fn in fileList:
        ori = fn.split('/')[-1].split('.tga')[0].split('_')[-1]
        if len(ori)==1:
            continue
        try:
            PIL.Image.open(fn).rotate(trans[ori][1]).save(fn.split('.tga')[0][:-2] + str(trans[ori][0]) + '.tga')
        except KeyError:
            print ('filename contains some non standrd suffix'),fn,ori

    print '_'.join(fileList[0].split('_')[:-1]) + '_#.tga'
    return '_'.join(fileList[0].split('_')[:-1]) + '_#.tga'


class SkySphere(DirectObject):
    def __init__(self,cubeMapPath=None):
        yawOff=198
        self.sphere = loader.loadModel("InvertSphereBlend.egg")
        # self.sphere = loader.loadModel("InvertedSphere.egg")
        # Load a sphere with a radius of 1 unit and the faces directed inward.

        self.sphere.setTexGen(TextureStage.getDefault(),
                              TexGenAttrib.MWorldPosition)
        self.sphere.setTexProjector(TextureStage.getDefault(), render,
                                    self.sphere)
        self.sphere.setTexTransform(TextureStage.getDefault(),
                                    TransformState.makeHpr(VBase3(yawOff, 0, 0)))

        # Create some 3D texture coordinates on the sphere. For more info on this, check the Panda3D manual.
        self.sphere.setPos((0,0,0))
        self.sphere.setTexPos(TextureStage.getDefault(), 0, 0, 0)
        self.sphere.setTexScale(TextureStage.getDefault(), 1)

        # tex = loader.loadCubeMap(cubeMapPath)
        if cubeMapPath is None:
            cubeMapPath=renamer()
        tex = loader.loadCubeMap(cubeMapPath)
        self.sphere.setTexture(tex)
        # Load the cube map and apply it to the sphere.

        self.sphere.setLightOff()
        # Tell the sphere to ignore the lighting.

        self.sphere.setScale(10)

        # Increase the scale of the sphere so it will be larger than the scene.
        print self.sphere.getHpr()
        print self.sphere.getPos()
        self.sphere.reparentTo(render)


        # Reparent the sphere to render so you can see it.
        # result = self.sphere.writeBamFile(cubeMapPath.split('_#.tga')[0]+'.bam')
        print '/'.join(cubeMapPath.split('/')[:-1])+'.bam'
        base.saveCubeMap('streetscene_cube_#.jpg', size=512)
        result = self.sphere.writeBamFile('/'.join(cubeMapPath.split('/')[:-1])+'.bam')
        # Save out the bam file.
        print(result)
        # Print out whether the saving succeeded or not.


SS = SkySphere()
