import os
import serial

class GetArduino():
    def __init__(self, baud=115200):
        '''

        Args:
            baud: Baud rate, match it with arduino
        '''
        serPort = self.get_serial_port()
        try:
            self.ser = serial.Serial(serPort, baudrate=baud)
            print "Arduino connected via port", serPort
        except serial.SerialException:
            print "arduino not connected, please replug the arduino\n"
            self.ser=1

    def get_serial_port(self):
        '''

        Returns: The port arduino is connected to

        '''
        return "/dev/" + \
               os.popen("dmesg | egrep ttyACM | cut -f3 -d: | tail -n1").read().strip()

