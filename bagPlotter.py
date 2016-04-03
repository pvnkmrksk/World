#!/usr/bun.env python
import rosbag_pandas
import easygui

bagPath=easygui.fileopenbox()
dataframe=rosbag_pandas.bag_to_dataframe(bagPath)
