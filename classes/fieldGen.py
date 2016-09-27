class FieldGen():
    # generate odour, plumes and wind
    # def __init__(self):
    #     pass

    def toPlot(self, obj, plot):
        if plot:
            plt.imshow(obj, interpolation='none', cmap='Greys_r')
            plt.show

    def plumeStrip(self, fieldWidth=128, fieldHeight=128, stripWidth=20, stripHeight=100, initX=54, initY=9,
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

        stripField = np.zeros([fieldWidth, fieldHeight]) != 0
        stripField[initX:initX + stripWidth, initY:initY + stripHeight] = True

        # print "stripfield is",stripField
        self.toPlot(stripField, plot)

    def odourField(self, w=255, h=255, oq=['s', 0, 's', 0], plot=False):

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
                strip = self.plumeStrip(offset, offset, width, offset, offset / 2 - (width / 2), 0)
                oqi[quad] = strip

            quad += 1

        odourField[0:offset, 0:offset] = oqi[2]
        odourField[offset + 1:w, 0:offset] = oqi[3]
        odourField[0:offset, offset + 1:w] = oqi[1]
        odourField[offset + 1:w, offset + 1:w] = oqi[0]

        #         thresh = parameters['odourDensity']
        #         rand = (np.random.rand(odourField.shape[0], odourField.shape[1]) < thresh) * 1
        #         odourField = np.logical_and(odourField, rand)
        #         parameters["odourField"] = odourField

        #         if plot:
        #             plt.imshow(np.rot90(odourField))
        #             plt.gray()
        #     #         plt.show(block=False)
        #             plt.show()

        #         plt.show(block=False)
        # print "odour field",parameters["odourField"]
        self.toPlot(np.rot90(odourField), plot)

        return odourField

    def odourPacket(self, width=10, height=10, velocity=3, packetFrequency=10, packetDuration=0.02, scale=100,
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

        packetOnPixels = scale * velocity * packetDuration  # on length in pixels
        packetTotalPixels = scale * velocity * (1.0 / packetFrequency)  # total length in pixels
        packetOffPixels = (packetTotalPixels - packetOnPixels)  # off length in pixels
        # offset X and Y to center, not critical but prettier and the least edge effect
        offsetX = (packetTotalPixels - packetOnPixels) / 2  # offset X to center
        offsetY = (packetTotalPixels - packetOnPixels) / 2  # offset Y to center

        packetFieldSubunit = np.zeros(
            [int(packetTotalPixels), int(packetTotalPixels)])  # != 0  # basic tileable subunit of odourfield
        packetFieldSubunit[offsetX:offsetX + packetOnPixels,
        offsetY:offsetY + packetOnPixels] = 1  # replace odour on region to True

        packetField = np.tile(packetFieldSubunit, (int(scale * width / packetTotalPixels), int(
            scale * height / packetTotalPixels)))  # tile the subunit such that it is of size (w,h)*scale
        # print packetField#don't do for large images
        # plt.imshow(packetField,interpolation='none',cmap='Greys_r') #don't interpolate and show the pixels as is with reverse grey cmap
        # plt.show()
        self.toPlot(packetField, plot)
        return packetField

