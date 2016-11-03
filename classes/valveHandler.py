import os
import serial

class ValveHandler:
    def __init__(self,valvePort,baud=115200,compression=True):
        '''

        Args:
            valvePort: Arduino switch case for the valve, led or servo
            baud: Baud rate, match it with arduino
            compression: When true, only value changes will be sent to serial saving bandwidth
        '''
        self.valvePort=valvePort
        self.compression=compression
        self.statePrev=0
        serPort=self.get_serial_port()
        try:
            self.ser=serial.Serial(serPort,baudrate=baud)
            print "Arduino connected via port", serPort
        except serial.SerialException:
            print "arduino not connected, please replug the arduino\n"
            return


    def get_serial_port(self):
        '''

        Returns: The port arduino is connected to

        '''
        return "/dev/" + \
               os.popen("dmesg | egrep ttyACM | cut -f3 -d: | tail -n1").read().strip()


    def move(self, state):
        '''
        Sets the specified object to the given state, a valve, led or servo
        Args:
            state: The state to which the valve has to be set

        Returns:
            None

        '''
        if (not(self.compression) or (state!=self.statePrev)): #update only if states different or if compression is diabled
            try:
                self.ser.write(chr(255))
                self.ser.write(chr(self.valvePort))
                self.ser.write((chr(int(state))))
                print "%s is now in state %i"%(self.valvePort,state)
            except:
                print "something reallly bad"
        self.statePrev = state  # reset to new state
