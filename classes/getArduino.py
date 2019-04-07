import os
import serial

class GetArduino():
    def __init__(self, baud=115200,serPort=None):
        '''

        Args:
            baud: Baud rate, match it with arduino
        '''
        print "trying to use serPort",serPort
        serPort = self.get_serial_port(serPort=serPort)
        try:
            self.ser = serial.Serial(serPort, baudrate=baud)
            print "Arduino connected via port", serPort
        except serial.SerialException as e:
            print "arduino not connected, please replug the arduino\n",e
            self.ser=1
        return self.ser

    def get_serial_port(self, serPort=None):
        '''

        Returns: The port arduino is connected to

        '''
        if (serPort is None) or (serPort is 'ACM'):#if none or acm
            return "/dev/" + os.popen("dmesg | egrep ttyACM | cut -f3 -d: | tail -n1").read().strip()
        elif serPort is 'USB':#if a usb serPort is given , give it ttyUSB*
            usbport10=os.popen("ls --color=auto -Flh /sys/class/tty/ttyUSB* | cut -f10 -d ' ' | cut -f5 -d'/'").\
                read().strip()
            usbport09=os.popen("ls --color=auto -Flh /sys/class/tty/ttyUSB* | cut -f9 -d ' ' | cut -f5 -d'/'").\
                read().strip()
            if 'tty' in usbport10:
                print "usbport10 is",usbport10
                return "/dev/" + usbport10
            else:
                print "using usbport9",usbport09
                return "/dev/"+usbport09
        else :#if a serPort is given , give it back
            return serPort

