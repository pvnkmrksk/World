from datetime import datetime
import json_tricks as json
import os
import subprocess
import time
import zipfile

from os import listdir
from os.path import isfile, join
from helping.libsOnly import *
from helping.importHelper import *
import rosbag
import rospy
from std_msgs.msg import String

from helping.helper import pickler,depickler, paramsFromGUI
# from params import parameters
parameters= paramsFromGUI()



class BagControl():
    def __init__(self,showBase, bagType, topics, parameters):
        self.sb=showBase
        self.bagType = bagType
        self.topics = topics
        self.startBag()
        print "test:", parameters["fly"], parameters["loadingString"]


    def startBag(self):
        self.bagger()
        self.metadata = self.metadataGen()
        # pickler(self.metadata, self.bagFilename)

        try:
            pickler(self.metadata, self.bagFilename + '.pickle')

            with open(self.bagFilename + ".json", 'w') as outfile:
                json.dump(self.metadata, outfile, indent=4, sort_keys=True, separators=(',', ':'),ensure_ascii=False)

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
        state=""
        try:
            # if parameters["hcp"]:
            #     mode += "hcp_"
            mode+= parameters["loadingString"]+'_'

            if parameters["loadOdour"]:
                mode += "odour_"

            if parameters["loadWind"]:
                mode += "wind_"

            if parameters["imposeStimulus"]:
                mode += "impose_"

            if parameters["loadNullModels"]:
                mode += "null_"



            state+='gain:'+str(parameters["gain"])+"_"
            state+='speed:'+str(parameters["maxSpeed"])+"_"
            state+='bout:'+str(parameters["maxBoutDur"])+"_"
            state+='DC:'+str(parameters["DCoffset"])+"_"
                    # if parameters["quad"]:
            #     mode += "quad_"

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


        fileName = bagDir + "/" + self.timeNow + "_" + parameters["fly"] + "_" + mode +state+ self.bagType

        print "filename:", fileName
        return fileName

    def addMetadata(self):
        self.zipRepo()
        models=[parameters['object1'],parameters['object2'],parameters['modelTextureMap'],parameters['skyMap'],
                parameters['modelTextureMapNull'],parameters['skyMapNull'],
                parameters['redTexPath'],parameters['treeTexPath'],parameters['greenTexPath'],
                parameters['modelHeightMap']]
        self.appendModels(models)

        print "allo " + self.bagFilename
        bagName = self.bagFilename + ".bag"
        if self.bagType == 'full':
            time.sleep(5)  # so that bag file can be transfereed from memory

        # self.metadata= self.metadataGen()
        try:
            metadata = (json.dumps(self.metadata,ensure_ascii=False))
        except UnicodeDecodeError as e:
            print "FIX this" \
                  "x unicode error in stop", e

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




    def zipRepo(self):
        #get a git hash of the current modified changes by using git stash
        #call the popen pipe out the stdout
        a = subprocess.Popen(['git', 'stash', 'create'], stdout=subprocess.PIPE)
        #talk to pipe and get only the key
        gitHash = a.communicate()[0].strip()

        #if no modified changes, get the current commit hash
        if not gitHash:
            b = subprocess.Popen(['git', 'rev-parse', 'HEAD'], stdout=subprocess.PIPE)
            gitHash = b.communicate()[0].strip()

        #using the hash, archive the current git repo as a bagfilename zip file
        arch=subprocess.Popen(['git', 'archive', '-o', self.zipFilename, gitHash])
        arch.communicate()#makes sure the zip process is complete

    def appendModels(self,models):
        #open zip in append mode with compression
        z = zipfile.ZipFile(self.zipFilename, "a", zipfile.ZIP_DEFLATED)

        texList=[]
        #iterate through model paths
        for fname in models:
            try:
                # if file exists append it, else continue
                fname = os.path.abspath(fname)

                fnz=str(Path(fname).relative_to(os.getcwd()))
                z.write(fnz)

                #make tentative tex path
                tex = '/'.join(fname.split('/')[0:-1]) + '/tex/'
                if tex not in texList:
                    texList.append(tex)
                else:
                    continue

                #if tex folder exists, list all files inside NON-Recursively
                if os.path.exists(tex):
                    onlyfiles = [f for f in listdir(tex) if isfile(join(tex, f))]

                    #for every item in tex folder, find path w.r.t current working dir so that
                    # zip tree structure matches what is now
                    for f in onlyfiles:
                        # local path, get path relayive to repo root directory
                        f = str(Path(tex + f).relative_to(os.getcwd()))  # local path, get path relayive to repo root directory so that bugs
                        # f = str(Path(os.path.abspath(tex) + f).relative_to(os.getcwd()))
                        z.write(f)

            except OSError :
                print "file doesn't exist",fname
                continue
            except ValueError:
                print "file",fname
                continue


    def metadataGen(self):

        self.zipFilename=self.bagFilename+'.zip'
        parameters['zipFileName']=self.zipFilename

        parameters['bagFileName']=self.bagFilename
        # parameters['omCase'] = self.sb.omCase
        # print "\n\n\n\n\nnomcase is ",parameters['omCase']

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
