import cPickle as pickle
import json

import numpy as np


# class Helper():

def pickler(obj, path):
    """
    Pickle a Python object
    """
    with open(path, "wb") as pfile:
        pickle.dump(obj, pfile)


def depickler(path):
    """
    Extracts a pickled Python object and returns it
    """
    with open(path, "rb") as pfile:
        data = pickle.load(pfile)
    return data

def paramsFromGUI():
    useGUI=True
    if useGUI:
        from GUI_caller import jsonVR
        print "using GUI"

        #def paramsGen(jsonPath):
        with open(jsonVR, 'r') as jfile:
            parameters = json.load(jfile)
            for item in parameters['toTuplify']:
                parameters[item] = tuple(parameters[item])

    else:
        from helping.params import parameters
        print "using old parms file"

    return parameters

def isInt(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def clamp(n, minn, maxn):
    """
    clamps values to lie between min and max
    Args:
        n: value to be clamped
        minn: min value of clamp
        maxn: max value of clamp

    Returns:
        Clamped value of n
    """
    if n <= minn:
        return minn
    elif n >= maxn:
        return maxn
    else:
        return n

def round_down(num, divisor):
    return num - (num%divisor)


def randIndexArray(range, shuffle):
    """
    creates index with range numObj
    if randPos = True(parameters), shuffles array
    necessary for random object positioning
    :return: array of indexes
    """

    arr = np.arange(range)
    if shuffle == True:
        np.random.shuffle(arr)
    return arr