import __main__ as myApp
from importHelper import datetime, helper
parameters = helper.paramsFromGUI()

class Player():
    
    def resetPos(self, newPos, initH, speed):
        myApp.app.player.setPos(newPos)
        myApp.app.player.setH(initH)
    
        myApp.app.decayTime = 240
        myApp.app.speedMemory = speed
        myApp.app.closedMemory = myApp.app.keyMap["closed"]
        print "self.newPos is", newPos, "\n"
    
        print "quadrant duration was ", str((datetime.now() - myApp.app.lastResetTime).total_seconds())
        print "\n \n \n"
    
        myApp.app.lastResetTime = datetime.now()
        myApp.app.boutFrame = 0
        myApp.app.reset = True  # set reset to true. Will be set to false after frame updtae

    def reachedDestination(self, target,distance=parameters["bboxDist"]):

        try:

            tl = (target[0] - distance, target[1] + distance)
            br = (target[0] + distance, target[1] - distance)
            #tl, br = self.boundingBoxCoordinates(target)
            x, y, z = myApp.app.player.getPos()
            if x > tl[0] and x < br[0] and y < tl[1] and y > br[1]:
                return True
            else:
                return False
        except TypeError:
            return False
