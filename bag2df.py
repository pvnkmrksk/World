# %matplotlib inline
# import mpld3
# mpld3.enable_notebook()
import rosbag_pandas
import pandas as pd
import matplotlib.pyplot as plt
import easygui
import rosbag
import json
import numpy as np
import cPickle as pkl
import time


def TicTocGenerator():
    # Generator that returns time differences
    ti = 0  # initial time
    tf = time.time()  # final time
    while True:
        ti = tf
        tf = time.time()
        yield tf - ti  # returns the time difference


TicToc = TicTocGenerator()  # create an instance of the TicTocGen generator


# This will be the main function through which we define both tic() and toc()
def toc(tempBool=True):
    # Prints the time difference yielded by generator instance TicToc
    tempTimeInterval = next(TicToc)
    if tempBool:
        print("Elapsed time: %f seconds.\n" % tempTimeInterval)


def tic():
    # Records a time in TicToc, marks the beginning of a time interval
    toc(False)


def pickler(obj, path):
    """
    Pickle a Python object
    """
    with open(path, "wb") as pfile:
        pkl.dump(obj, pfile)


def depickler(path):
    """
    Extracts a pickled Python object and returns it
    """
    with open(path, "rb") as pfile:
        data = pkl.load(pfile)
    return data


'''
Load  bag filesto make into respective dataframes
'''
defaultPath = "/home/rhagoletis/catkin/src/beginner/scripts/panda/world/bags/"
paths = easygui.fileopenbox(title="Bags to Dataframes"
                            , default=defaultPath,
                            multiple=True, filetypes=["*traj.bag"])
print paths, "\n"
metadata = None

i = 1
for path in paths:
    tic()
    print "starting analysis of file %s , %s / %s files" % (path.split('/')[-1], i, len(paths))
    df = rosbag_pandas.bag_to_dataframe(path, include=['/trajectory'])
    bag = rosbag.Bag(path)

    try:
        for topic, msg, t in bag.read_messages(topics='/metadata'):
            a = msg
        # parameters=json.loads(a.data)
        #         metadata={"meta":parameters}
        metadata = json.loads(a.data)

    except:
        print "no such file!"

    obj = dict(df=df, metadata=metadata)

    picklepath = path + "_df.pickle"
    pickler(obj, picklepath)
    #     df.to_pickle(picklepath)

    i += 1
    toc()
print "\nanalysis of %s files complete" % len(paths)
