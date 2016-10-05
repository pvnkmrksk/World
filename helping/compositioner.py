from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import NodePath


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.world = self.loader.loadModel("models/world257.bam")
        self.world.reparentTo(self.render)

        self.player = NodePath("player")
        self.player.setPos(self.world, 80,130,2)
        # self.player.setH(225)
        self.camera.setPos(self.player, 0, 0, 0)
        self.camera.setHpr(self.world,0,0,0)
        self.taskMgr.add(self.updateTask, "update")

    def updateTask(self, task):
        print "POS: " + str(self.camera.getPos(self.world))
        print "HPR: " + str(self.camera.getHpr(self.world))
        return Task.cont


app = MyApp()
app.run()
