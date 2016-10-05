from __future__ import division
import numpy as np
import matplotlib.pyplot as plt

class FieldGen():
    # generate odour, plumes and wind
    def __init__(self):
        import numpy as np
        import matplotlib.pyplot as plt

    def toPlot(self, obj, plot):
        if plot:
            plt.imshow(obj, interpolation='none', cmap='Greys_r')
            plt.show(block=False)

    def windField(self,width,height,wq,plot=False):
        windField = np.zeros([width, height])

        offset = (width - 1) / 2

        windField[0:offset, 0:offset] = wq[2]
        windField[offset + 1:width, 0:offset] = wq[3]
        windField[0:offset, offset + 1:width] = wq[1]
        windField[offset + 1:width, offset + 1:width] = wq[0]

        self.toPlot(windField,plot=plot)
        return windField



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

        stripField = np.zeros([fieldWidth, fieldHeight])
        stripField[initX:initX + stripWidth, initY:initY + stripHeight] = 1

        # print "stripfield is",stripField
        self.toPlot(stripField, plot)
        return  stripField

    def odourField(self, w=255, h=255, oq=['s', 1, 'p', 0], plot=False):

        odourField = np.zeros([w, h])
        offset = int((w - 1) / 2)

        oqi = oq
        quad = 0

        for i in oq:
            from skimage.io import imread

            if i == 'c':
                oqi[quad] = (np.rot90(imread("models/odour/" + str(quad + 1) + ".png"))) != 0
                # py 0 index but non zero quadrants and the image is rotated to fix plt and array axes
            #                 print "odour image is", quad, oqi[quad]
            elif i == 's':
                width = 15
                # strip = self.plumeStrip()
                strip = self.plumeStrip(offset, offset, width, offset, (offset / 2 - (width / 2)), 0)
                # print strip
                oqi[quad] = strip

            elif i=='p':
                packet=self.odourPacket(width=127,height=127,velocity=1,packetFrequency=0.5,packetDuration=1,scale=1)
                oqi[quad] = packet

            quad += 1

        odourField[0:offset, 0:offset] = oqi[2]
        odourField[offset + 1:w, 0:offset] = oqi[3]
        odourField[0:offset, offset + 1:w] = oqi[1]
        odourField[offset + 1:w, offset + 1:w] = oqi[0]


        self.toPlot(np.rot90(odourField), plot)

        return odourField

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
        packetField = np.tile(packetFieldSubunit, (int(scale * width / packetTotalPixels), int(
            scale * height / packetTotalPixels)))

        wd=scale*width-packetField.shape[0]
        hd=scale*height-packetField.shape[1]
        packetField=np.lib.pad(packetField,((0,hd),(0,wd)),'constant',constant_values=(0,0))
        # print packetField#don't do for large images
        # plt.imshow(packetField,interpolation='none',cmap='Greys_r') #don't interpolate and show the pixels as is with reverse grey cmap
        # plt.show()
        self.toPlot(packetField, plot)

        return packetField

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
