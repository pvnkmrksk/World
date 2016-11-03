from __future__ import division
from helping import helper
import numpy as np

class WindTunnel():
    def __init__(self, windDirection,fps=165):
        if windDirection != -1:  # -1 is open loop in wind direction
            self.servoAngle = int((90 - (self.player.getH()) + windDirection - 180) % 360)
        else:
            self.servoAngle = 90
            # print "wind in open loop"
        servo.move(1, self.servoAngle)


class OdourTunnel():
    # current packet frequency from odour field
    def __init__(self,odourField,player,parameters,odourMask=None,phase=0):
        self.of=odourField
        self.om=odourMask
        self.mask=True
        self.player=player
        self.parameters=parameters
        self.phase=phase

    def update(self,packetDur):
        '''

        Args:
            packetDur: packet duration in seconds

        Returns:
            state of the valve object
        '''
        x=int(self.player.getX())
        y=int(self.player.getY())
        self.pf = self.of[x,y]
        if self.om:
            self.mask = self.om[x,y]
            self.pf=np.logical_and(self.mask,self.pf)


        '''calculate Tau=Time period ,
        if pf>0, if in the packet on time, turn on valve else off
        else turn off valve
        set the volume to high or low and send command to arduino to set valve state
        finally increment the phase
        '''


        self.packetDur = helper.round_down(packetDur, 1 / self.parameters['fps'])

        if self.pf > 0:
            # rounding down to the nearest multiple of frame time
            self.tau = helper.round_down(self.parameters['fps'] / self.pf,
                                                1 / self.parameters['fps'])

            if (self.phase % self.tau) < (self.parameters['fps'] * self.packetDur):
                self.state = 1
            else:
                self.state = 0

        else:
            self.state = 0

        self.phase += 1
        return self.state

