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
# import servo
from helping import helper


from datetime import datetime
import sys, time, subprocess, os, serial  # ROS imports
import json_tricks as json
import rospy, rostopic, roslib, std_msgs.msg, rosbag
from std_msgs.msg import String
from rospy_message_converter import message_converter

from direct.showbase.ShowBase import ShowBase  # Panda imports
from direct.task import Task
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Vec3, Fog, Camera, PerspectiveLens
from panda3d.core import loadPrcFileData, NodePath, TextNode
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

parameters=helper.paramsFromGUI()
