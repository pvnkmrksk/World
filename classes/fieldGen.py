from __future__ import division
from importHelper import *
import numpy as np
import matplotlib.pyplot as plt
class FieldGen():
    # generate odour, plumes and wind

    def toPlot(self, obj, plot):
        if plot:
            plt.imshow(obj, interpolation='none', cmap='Greys_r')
            plt.show(block=False)

    def windField(self,width=1025,height=1025,wq=[-1,0,180,270],plot=False,fillMidLine=180):
        '''

        :param width: width of the wind field
        :param height: height of the wind field
        :param wq: wind quad. A list of 4 items setting the angle of that quadrant
        :param plot: whether to plot the image
        :return:
            windField: a matrix filled with 4 quads of particular windDirection
        '''
        wind_field = np.zeros([width, height])

        offset = int(((width - 1) / 2))

        wind_field[0:offset, 0:offset] = wq[2]
        wind_field[offset :width, 0:offset] = wq[3]
        wind_field[0:offset, offset :width] = wq[1]
        wind_field[offset :width, offset :width] = wq[0]
        #
        # wind_field[0:offset, 0:offset] = wq[2]
        # wind_field[offset + 1:width, 0:offset] = wq[3]
        # wind_field[0:offset, offset + 1:width] = wq[1]
        # wind_field[offset + 1:width, offset + 1:width] = wq[0]


        #to fill the midline regions of the matrix which is not accessed by offsets
        wind_field[offset:offset+1,offset:offset+1]=fillMidLine
        self.toPlot(wind_field,plot=plot)
        return wind_field


    def plumeStrip(self, fieldWidth=128, fieldHeight=128,
                   stripWidth=20, stripHeight=100, initX=54, initY=9,
                   plot=False):

        """
        Args:
            fieldWidth: width of array
            fieldHeight: height of array
            stripWidth:  width of the plume strip
            stripHeight: height of the plume strip
            initX:      starting X point of strip in numpy array
            initY:      starting Y point of strip in numpy array

        Returns:
            stripField :    An array with 1 where the plume exists, which is a
                            rectangular strip and 0 else where
        """
        fieldWidth = int(fieldWidth)
        fieldHeight = int(fieldHeight)
        stripWidth = int(stripWidth)
        stripHeight = int(stripHeight)
        initX = int(initX)
        initY = int(initY)

        strip_field = np.zeros([fieldWidth, fieldHeight])
        strip_field[initX:initX + stripWidth, initY:initY + stripHeight] = 1

        # print "stripfield is",stripField
        self.toPlot(strip_field, plot)
        return  strip_field


    def odourPacket(self, width=10, height=10, velocity=3,
                    packetFrequency=10, packetDuration=0.02, scale=10,
                    plot=False):
        '''

        :param width: image width in pixels
        :param height: image height in pixels
        :param velocity: traversing velocity in pixels/second
        :param packetFrequency: odour packets frequency in Hertz
        :param packetDuration: duration of a single packet in seconds
        :param scale: scale factor to do sub pixel localization

        :return packetField: An array of size (w*scale, h*scale) with packets such that,
        when traversing through the matrix at velocity(px/s), one would encounter the packets at a frequency
        packetFrequency(Hz) and each packet would last for packetDuration(s)

        '''

        # on length in pixels
        packetOnPixels = scale * velocity * packetDuration

        # total length in pixels
        packetTotalPixels = scale * velocity * (1.0 / packetFrequency)

        # off length in pixels
        # packetOffPixels = (packetTotalPixels - packetOnPixels)

        # offset X and Y to center, not critical but prettier and the least edge effect
        offsetX = (packetTotalPixels - packetOnPixels) / 2  # offset X to center
        offsetY = (packetTotalPixels - packetOnPixels) / 2  # offset Y to center


        # basic tileable subunit of odourfield
        packetFieldSubunit = np.zeros([int(packetTotalPixels), int(packetTotalPixels)],dtype=bool)  # != 0
        packetFieldSubunit[offsetX:offsetX + packetOnPixels,
        offsetY:offsetY + packetOnPixels] = 1  # replace odour on region to True

        # tile the subunit such that it is of size (w,h)*scale
        packet_field = np.tile(packetFieldSubunit, (int(scale * width / packetTotalPixels), int(
            scale * height / packetTotalPixels)))

        wd=scale*width-packet_field.shape[0]
        hd=scale*height-packet_field.shape[1]
        packet_field=np.lib.pad(packet_field,((0,hd),(0,wd)),'constant',constant_values=(0,0))
        # print packetField#don't do for large images
        # plt.imshow(packetField,interpolation='none',cmap='Greys_r') #don't interpolate and show the pixels as is with reverse grey cmap
        # plt.show()
        self.toPlot(packet_field, plot)

        return packet_field

    def odourField(self, w=257, h=257, oq=['s', 1, 'p', 0],plot=False):
        '''
        GIves an array filley with 4 arrays as quadrants with packets or strips or custom images to be used as odourfield

        :param w: array width
        :param h: array height
        :param oq: a list with 4 items,
        's' is strip,
        'p' is packet,
        'c' is integers of odour images path in models/odour/(1,2,3,4).png,
         a float, will be used as packet frequency at that region
         an array of size (w-1)/2
        :param plot: bool, will plot the final image
        :return: final odorrquad image as an np array

        '''

        odour_field = np.zeros([w, h])
        offsetW = int((w - 1) / 2)
        offsetH = int((h - 1) / 2)

        quad = 0

        for i in oq:

            if i == 'c':#custom image in models/odour/1,2,3,4.png
                oq[quad] = (np.rot90(imread("models/odour/" + str(quad + 1) + ".png"))) != 0
                # py 0 index but non zero quadrants and the image is rotated to fix plt and array axes
            elif i == 's': #strip of solid one
                width = 15

                strip = self.plumeStrip(offsetW, offsetH, width, offsetH, (offsetW / 2 - (width / 2)), 0)
                oq[quad] = strip

            elif i=='p': #packet field of
                packet=self.odourPacket(width=127,height=127,velocity=1,packetFrequency=0.5,packetDuration=1,scale=1)
                oq[quad] = packet

            quad += 1
        #
        # odour_field[0:offsetW, 0:offsetH] = oq[2]
        # odour_field[offsetW + 1:w, 0:offsetH] = oq[3]
        # odour_field[0:offsetW, offsetH + 1:h] = oq[1]
        # odour_field[offsetW + 1:w, offsetH + 1:h] = oq[0]

        odour_field=self.gen(odour_field,oq,w,h)
        self.toPlot(np.rot90(odour_field), plot)

        return odour_field

    def gen(self,field,quad,w,h):
        offsetW = int((w - 1) / 2)
        offsetH = int((h - 1) / 2)


        field[0:offsetW, 0:offsetH] = quad[2]
        field[offsetW :w, 0:offsetH] = quad[3]
        field[0:offsetW, offsetH :h] = quad[1]
        field[offsetW :w, offsetH :h] = quad[0]
        return field


    def maskField(self, mq, w=257, h=257):
        mask_field = np.zeros([w, h])
        mask_field =self.gen(mask_field,mq,w,h)
        return mask_field

if __name__=='main':
    f=FieldGen()
    from params import parameters
    odourField = f.odourPacket(width=parameters['worldSize'],
                                             height=parameters['worldSize'],
                                             scale=parameters['fieldRescale'],
                                             packetFrequency=parameters['packetFrequency'],
                                             plot=True,
                                           velocity=parameters['maxSpeed'],

                               packetDuration=parameters['packetDuration'])
