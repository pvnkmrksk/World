import cPickle as pickle
import json
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

        with open(jsonVR, 'r') as jfile:
            parameters = json.load(jfile)
            for item in parameters['toTuplify']:
                parameters[item] = tuple(parameters[item])
    else:
        from params import parameters
    return parameters
