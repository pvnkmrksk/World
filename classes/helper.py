import cPickle as pickle
class Helper():

    def pickler(self, obj, path):
        """
        Pickle a Python object
        """
        with open(path, "wb") as pfile:
            pickle.dump(obj, pfile)


    def depickler(self, path):
        """
        Extracts a pickled Python object and returns it
        """
        with open(path, "rb") as pfile:
            data = pickle.load(pfile)
        return data

