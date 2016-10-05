class BagControl():
    def __init__(self, bagType, topics):
        self.bagType = bagType
        self.topics = topics
        self.startBag()

    def startBag(self):
        self.bagger()
        obj = self.metadataGen()
        app.pickler(obj, self.bagFilename)

        with open(self.bagFilename + ".json", 'w') as outfile:
            json.dump(obj, outfile, indent=4, sort_keys=True, separators=(',', ':'))
        time.sleep(0.15)  # sleep to prevent multiple instatntiations for a single keypress

    def stopbag(self):
        # self.runBagCommand.send_signal(subprocess.signal.SIGINT) #send signal on stop command
        self.terminate_ros_node("/record")
        rospy.loginfo("\n \n \n Bag recording stopped \n \n \n ")
        self.addMetadata()
        print "metadata added \n \n "
        print "\n \n bagfilename is", self.bagFilename

    def bagger(self):
        self.bagFilename = self.bagFilenameGen()
        self.bagCommand = "rosbag record --lz4 --output-name=" + self.bagFilename + " " \
                          + self.topics
        # print self.bagCommand
        self.runBagCommand = subprocess.Popen(self.bagCommand, shell=True, stdout=subprocess.PIPE)
        rospy.loginfo("Bag recording started")

    def bagFilenameGen(self):
        self.timeNow = str(datetime.now().strftime('%Y-%m-%d__%H:%M:%S'))
        mode = ""
        if parameters["hcp"]:
            mode += "hcp_"
        if parameters["loadWind"]:
            mode += "wind_"
        if parameters["imposeStimulus"]:
            mode += "impose_"
        if parameters["quad"]:
            mode += "quad_"

        bagDir = "bags/" + str(datetime.now().strftime('%Y_%m_%d'))  # make a directory of current date
        if not os.path.exists(bagDir):
            os.makedirs(bagDir)  # create dir if non existent

        fileName = bagDir + "/" + self.timeNow + "_" + parameters["fly"] + "_" \
                   + mode + parameters["loadingString"] \
                   + "_gain" + str(parameters["gain"]) \
                   + "_trial_" + str(parameters["trialNo"]) + "_" + self.bagType
        print fileName
        return fileName

    def addMetadata(self):
        print "allo " + self.bagFilename
        bagName = self.bagFilename + ".bag"
        if self.bagType == 'full':
            time.sleep(5)  # so that bag file can be transfereed from memory

        obj = self.metadataGen()
        metadata = (json.dumps(obj))
        metadata = String(metadata)
        # print "metadata is:", metadata

        with rosbag.Bag(bagName, 'a') as bag:
            i = 0
            for _, _, t in bag.read_messages():
                if i == 0:
                    tstamp = t
                i += 1
                break
            bag.write('/metadata', metadata, tstamp)
            # datasave

    def metadataGen(self):
        file = open(__file__, 'r')
        servo = open("servo.py", 'r')
        arduino = open("servoControl/servoControl.ino", 'r')
        bam = open(app.worldFilename)
        obj = dict(parameters=parameters, world=file.read(),
                   servo=servo.read(), arduino=arduino.read(), )
        # bam=bam.read().encode('utf-8').strip())
        # obj = [parameters, file.read(), servo.read(), arduino.read(), world.read()]
        return obj

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)