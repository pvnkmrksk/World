from __future__ import division

# from params import parameters
from World.msg import MsgFlystate, MsgTrajectory
from classes.bagControl import BagControl
from classes.fieldGen import FieldGen
# from classes.helper import Helper
from classes.exceptionHandlers import ExceptionHandlers
from classes.valveHandler import ValveHandler
from classes.tunnel import OdourTunnel
from classes.tunnel import WindTunnel
from classes.stimulus import Stimulus, NextStimStartException, NextPreStimStartException
# import servo
from helping import helper
import fractions

from datetime import datetime
import sys, time, subprocess, os, serial  # ROS imports
import json_tricks as json
import rospy, rostopic, roslib, std_msgs.msg, rosbag
from std_msgs.msg import String

# from rospy_message_converter import message_converter

from direct.showbase.ShowBase import ShowBase  # Panda imports
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog, Camera, PerspectiveLens
from panda3d.core import loadPrcFileData, NodePath, TextNode, CullFaceAttrib
from panda3d.core import CompassEffect, ClockObject
from panda3d.core import Shader
from panda3d.core import loadPrcFileData
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import WindowProperties

import matplotlib.pyplot as plt  # plotting imports
from matplotlib.path import Path
import matplotlib.patches as patches
import cPickle as pickle
import random
import numpy as np
import easygui
import pandas as pd
from skimage.io import imread
import zipfile
from pathlib import Path

from os import listdir
from os.path import isfile, join

i=0
print "I have been imported",i
i+=1
#todo.FIXME paramsFromGUI uses current values instead of replay and makes inaccurate replay
parameters=helper.paramsFromGUI()
# parameters=helper.paramsFromGUI()
useGui = True
replayFileSelected=False

def exceptionReplay(parameters):
    """
    Replay world playsback position and orientation, can be additionally used for screen capturing
    Servo instantiates the servo object
    next check for if ROS is running in the background, else start ros and the kinefly for WBAD
    """
    # If replay world for playback old data, copy existing params to prevent overwrite from the saved state of previous run
    if parameters["replayWorld"]:

        # save params from current instance as backup toload back into the playback file
        replay = parameters["replayWorld"]
        scale = parameters["captureScale"]
        start = parameters["captureStart"]
        increment = parameters["playbackIncrement"]

        # replayPath = easygui.fileopenbox(multiple=False, filetypes=["*.pickle"])
        replayPath = "/home/behaviour/catkin/src/beginner/scripts/panda/world/bags/fly4/fly4_quad_rg_gain7.0_speed_3.5_" \
                     "trial_1_2016-04-13__23:31:35.bag_df.pickle"

        print replayPath
        df = pd.read_pickle(replayPath)

        # slice pos and orientation and remove nans heading,pitch,roll, x,y,z and drop na which is from camera message time
        traj = df.loc[:, "trajectory__orientation_x":"trajectory__position_z"].dropna()
        cols = traj.columns.tolist()  # list of colums
        cols = cols[-3:] + cols[:-3]  # reorder colums. hprpos to poshpr by python splicing
        traj = traj[cols]  # reassign with this order

        # //todo change current parameters to parameters saved in dataframe
        parameters = None

        try:
            parameters = json.loads(df.metadata__data.values[1])

        except:
            parameters = json.loads(df.metadata__data.values[0])
            print "using exceprion to load params"

        # dump back params from vars
        parameters["replayWorld"] = replay
        parameters["captureScale"] = scale
        parameters["captureStart"] = start
        parameters["playbackIncrement"] = increment

        return parameters

class ParamCache():
    def __init__(self,parameters):
        '''
        takes the current parameters, saves a few values into class vars
        paramLoad will load back those saved values back to the inpuit dictionary
        Args:
            parameters: dict which is currently having the replayworld params
        '''
        self.parameters=parameters
        self.paramDump()

    def paramDump(self):
        '''
        dumps a few a vars into class var
        Returns:

        '''
        self.replay = self.parameters["replayWorld"]
        self.scale = self.parameters["captureScale"]
        self.start = 0
        self.increment = self.parameters["playbackIncrement"]
        self.record = parameters["frameRecord"]
        self.recordPath= parameters['frameRecordPath']
        self.dur = parameters["recordDur"]
        self.recordfps = parameters["recordFps"]


    def paramLoad(self,parameters):
        '''
        loads the input params with the saved params
        Args:
            parameters: the dict which is now updated with bag params but has lost the replay values

        Returns:
            parameters: the new updated dict from bag file and the old replay params
        '''
        # dump back params from vars
        parameters["frameRecord"]=self.record
        parameters["replayWorld"] = self.replay
        parameters["captureScale"] = self.scale
        parameters["captureStart"] = self.start
        parameters["playbackIncrement"] = self.increment
        parameters['frameRecordPath']= self.recordPath
        parameters["recordDur"] = self.dur
        parameters["recordFps"] = self.recordfps
        parameters["modelTextureMapNull"] = parameters["modelTextureMap"]
        parameters["skyMapNull"] = parameters["skyMap"]
        parameters["loadingString"] = "circ" #todo: remove that after recording


        return parameters


def replayLoader():
    '''Load a dataframe from pickle and analyse lr cases'''

    defaultPath="/home/rhagoletis/catkin/src/World/bags/"
    # if parameters['replaybag']:
    #    ftype= ["*.bag"]
    #    text='Choose the bag to replay'
    # else:
    ftype= ["*.pickle"]
    text='Choose the pickle file to replay '
    path=easygui.fileopenbox(title=text
                              ,default=defaultPath,
                              multiple=False,filetypes=ftype)
    # path='/home/rhagoletis/catkin/src/World/bags/2017_01_04/2017-01-04__23:08:26_apple15_11_traj.bag_df.pickle'

    print 'path to replay is',path

    dff=pd.read_pickle(path)
    df=dff['df']
    metadata=dff['metadata']
    return df,metadata

if parameters['replayWorld']:
    print "\n \n replayFIlke is \n\n",replayFileSelected
    if not replayFileSelected:
        # if parameters['replayBag']:

        #copy the needed params from parameters
        pc=ParamCache(parameters)

        df,metadata=replayLoader()
        dfPosHpr = df[['trajectory__pPos_x', 'trajectory__pPos_y', 'trajectory__pPos_z',
                     'trajectory__pOri_x', 'trajectory__pOri_y','trajectory__pOri_z']]
        dfPosHpr.columns = [['x', 'y', 'z', 'h', 'p', 'r']]
        #reassigning the bag parameters
        parameters=metadata['parameters']
        print "loadingString metadata importHelper:", metadata["parameters"]["loadingString"]
        for item in parameters['toTuplify']:
            parameters[item] = tuple(parameters[item])
        parameters['replayWorld']=True #this is because, while the actual bag never had replayworld checked.
        # But the code needs it to be true to playback bag data

        #dump the saved parameters into the current one
        parameters=pc.paramLoad(parameters)
        replayFileSelected=True
        print "loadingString importHelper:", parameters["loadingString"]
        print "bagDur:", parameters["maxBoutDur"]


    else:
        pass