import __main__ as myApp
from importHelper import datetime, helper
parameters = helper.paramsFromGUI()

class Player():
    """
    handles the player
    still some things in myApp to add to this class
    """
    
    def __init__(self,showbase):
        self.sb=showbase
    
    def resetPos(self, newPos,  initH=parameters["playerInitH"], speed=parameters["speed"]):
        """

        :param newPos:
        :param initH:
        :param speed:
        :return:
        """
        
        
        # myApp.app.player.setPos(newPos)
        # myApp.app.player.setH(initH)
        # 
        # myApp.app.decayTime = 240
        # myApp.app.speedMemory = speed
        # myApp.app.closedMemory = myApp.app.keyMap["closed"]
        # print "self.newPos is", newPos, "\n"
        # 
        # print "quadrant duration was ", str((datetime.now() - myApp.app.lastResetTime).total_seconds())
        # print "\n \n \n"
        # 
        # myApp.app.lastResetTime = datetime.now()
        # myApp.app.boutFrame = 0
        # myApp.app.reset = True  # set reset to true. Will be set to false after frame updtae
        # 
        # 
        # #rest tunnel to zwero phase so that on quad change, the onset of packet is at predicatbale
        # # and at the beginning after an offset of  50frames 300ms so that a keypress doesn't end up with a sustained odour
        # # and insteadof history dependence and so may switch any time
        # myApp.app.haw.phase = 0
        # myApp.app.apple.phase = 0
        # 
        self.sb.player.setPos(newPos)
        self.sb.player.setH(initH)
    
        self.sb.decayTime = 165
        self.sb.speedMemory = speed
        self.sb.closedMemory = self.sb.keyMap["closed"]
        print "self.newPos is", newPos, "\n"
    
        print "quadrant duration was ", str((datetime.now() - self.sb.lastResetTime).total_seconds())
        print "\n \n \n"
    
        self.sb.lastResetTime = datetime.now()
        self.sb.boutFrame = 0
        self.sb.reset = True  # set reset to true. Will be set to false after frame updtae


        #rest tunnel to zwero phase so that on quad change, the onset of packet is at predicatbale
        # and at the beginning after an offset of  50frames 300ms so that a keypress doesn't end up with a sustained odour
        # and insteadof history dependence and so may switch any time
        # self.sb.haw.phase = 0
        # self.sb.apple.phase = 0

    def reachedDestination(self, target,distance=parameters["bboxDist"]):
        """
        checks if player is in area to reset the run
        if one object and therefore one position is None, event is handled with TypeError
        :param target: tuple with position to reset
        :param distance: size of reset-area
        :return: True or False
        """

        try:
            # creates hitbox around target-position with size of distance
            # tl = top left coordinate
            # br = bottom right coordinate
            tl = (target[0] - distance, target[1] + distance)
            br = (target[0] + distance, target[1] - distance)
            x, y, z = myApp.app.player.getPos()

            if x > tl[0] and x < br[0] and y < tl[1] and y > br[1]:
                # if player is in reset-area return true, else return false
                # gets checked every frame
                return True
            else:
                return False
        except TypeError:
            # if one position is None because of None-object, return False
            return False
