import os
import serial

class GetArduino():
    def __init__(self, baud=115200,serPort=None):
        '''

        Args:
            baud: Baud rate, match it with arduino
        '''
        print serPort
        serPort = self.get_serial_port(serPort=serPort)
        try:
            self.ser = serial.Serial(serPort, baudrate=baud)
            print "Arduino connected via port", serPort
        except serial.SerialException:
            print "arduino not connected, please replug the arduino\n"
            self.ser=1
        return self.ser

    def get_serial_port(self, serPort=None):
        '''

        Returns: The port arduino is connected to

        '''
        if (serPort is None) or (serPort is 'ACM'):#if none or acm
            return "/dev/" + os.popen("dmesg | egrep ttyACM | cut -f3 -d: | tail -n1").read().strip()
        elif serPort is 'USB':#if a usb serPort is given , give it ttyUSB*
            return "/dev/" + os.popen("ls --color=auto -Flh /sys/class/tty/ttyUSB* | cut -f10 -d ' ' | cut -f5 -d'/'").read().strip()
        else :#if a serPort is given , give it back
            return serPort

