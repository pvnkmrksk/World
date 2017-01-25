from datetime import datetime
import json_tricks as json
import os
import subprocess
import time

import rosbag
import rospy
from std_msgs.msg import String

from helping.helper import pickler,depickler, paramsFromGUI
# from params import parameters
parameters= paramsFromGUI()



class BagControl():
    def __init__(self, bagType, topics, parameters=parameters):
        self.bagType = bagType
        self.topics = topics
        self.startBag()
        print "test:", parameters["fly"], parameters["loadingString"]


    def startBag(self):
        self.bagger()
        obj = self.metadataGen()
        pickler(obj, self.bagFilename)

        try:
            with open(self.bagFilename + ".json", 'w') as outfile:
                pickler(obj,self.bagFilename+'.pickle')

                json.dump(obj, outfile, indent=4, sort_keys=True, separators=(',', ':'),ensure_ascii=False)
        except UnicodeDecodeError as e:

            print "error with unicode FIX IT", e
            pass
        # time.sleep(0.15)  # sleep to prevent multiple instatntiations for a single keypress

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
        # todo: fix filename generate

        self.timeNow = str(datetime.now().strftime('%Y-%m-%d__%H:%M:%S'))
        mode = ""
        try:
            # if parameters["hcp"]:
            #     mode += "hcp_"
            if parameters["loadWind"]:
                mode += "wind_"
            if parameters["imposeStimulus"]:
                mode += "impose_"
            if parameters["quad"]:
                mode += "quad_"
        except KeyError:
            print "key missing, change the code now!"

        bagDir = "bags/" + str(datetime.now().strftime('%Y_%m_%d'))  # make a directory of current date
        if not os.path.exists(bagDir):
            os.makedirs(bagDir)  # create dir if non existent

        # fileName = bagDir + "/" + self.timeNow + "_" + parameters["fly"] + "_" \
        #            + mode + parameters["loadingString"] \
        #            + "_gain" + str(parameters["gain"]) \
        #            + "_trial_" + str(parameters["trialNo"]) + "_" + self.bagType
        #


        fileName = bagDir + "/" + self.timeNow + "_" + parameters["fly"] + "_" + parameters["loadingString"] \
                   + "_" + self.bagType

        print "filename:", fileName
        return fileName

    def addMetadata(self):
        print "allo " + self.bagFilename
        bagName = self.bagFilename + ".bag"
        if self.bagType == 'full':
            time.sleep(5)  # so that bag file can be transfereed from memory

        obj = self.metadataGen()
        try:
            metadata = (json.dumps(obj,ensure_ascii=False))
        except UnicodeDecodeError as e:
            print "FIX this unicode error in stop", e

            metadata = "empty"
        metadata = String(metadata)
        # print "metadata is:", metadata

        def saver():
            with rosbag.Bag(bagName, 'a') as bag:
                i = 0
                for _, _, t in bag.read_messages():
                    if i == 0:
                        tstamp = t
                    i += 1
                    break
                bag.write('/metadata', metadata, tstamp)
                # datasave

        #large bags take more than 5 sefconds to save, if so wait and then try again
        try:
            saver()
        except rosbag.bag.ROSBagException:
            time.sleep(10)
            saver()


    def metadataGen(self):
        file = open(__file__, 'r')
        # servo = open("servo.py", 'r')
        # arduino = open("servoControl/servoControl.ino", 'r')
        # bam = open(app.worldFilename)


        # obj = dict(parameters=parameters, world=file.read(),
        #            servo=servo.read(), arduino=arduino.read(), )
        #

        obj=dict(parameters=parameters)

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
