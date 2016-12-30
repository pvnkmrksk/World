import os
import serial
import threading
from classes.reader import ArduinoReader
from getArduino import GetArduino
class ValveHandler(GetArduino):
    def __init__(self, casePort, baud=115200, compression=True):
        '''

        Args:
            casePort: Arduino switch case for the valve, led or servo
            baud: Baud rate, match it with arduino
            compression: When true, only value changes will be sent to serial saving bandwidth
        '''
        self.serPort=GetArduino.__init__(self,baud=baud)
        # self.ard=ArduinoReader()
        self.casePort=casePort
        self.compression=compression
        self.statePrev=0
        self.state=0
        # self.moveT=threading.Thread(target=self.moveThread,kwargs={'valvePort':self.valvePort,'state':self.state})
    def moveThread(self,state):
        try:
            self.ser.write(chr(255))
            self.ser.write(chr(self.casePort))
            self.ser.write((chr(int(state))))
            # print "%s is now in state %i"%(self.valvePort,state)
        except:
            print "something reallly bad"

    def move(self, state):
        '''
        Sets the specified object to the given state, a valve, led or servo
        Args:
            state: The state to which the valve has to be set

        Returns:
            None

        '''
        if self.serPort !=1:#error check of return to make sure arduino works
            if (not(self.compression) or (state!=self.statePrev)): #update only if states different or if compression is diabled
                # self.moveT = threading.Thread(target=self.moveThread,
                #                               kwargs={'state': state})
                #
                # self.moveT.start()


                # if state==0:
                #     val="LOW"
                # else:
                #     val="HIGH"
                # self.ard.board.pinMode(2,"OUTPUT")
                # self.ard.board.digitalWrite(2,val)
                #
                # print "val is",val
                # print self.ard.board.analogRead(3)
                try:
                    # pass
                    self.serPort.write(chr(255))
                    self.serPort.write(chr(self.casePort))
                    self.serPort.write((chr(int(state))))
                    # print "serial done",chr(int(state))

                    # print "%s is now in state %i"%(self.valvePort,state)
                except Exception as e:
                    print "something reallly bad in valvehandler",e
                    pass
            self.statePrev = state  # reset to new state
