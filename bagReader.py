import easygui
import rosbag
from std_msgs.msg import String
import json
bagPath=easygui.fileopenbox()#file open dialog

bag=rosbag.Bag(bagPath)#init bag

for topic , msg, t in bag.read_messages(topics='/metadata'):
    parameters=msg #metadata topic , a String which is a json dump

parameters=json.loads(parameters.data) #undump the json and load it to recreate the dictionary

