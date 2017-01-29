# example3.py
def main(fp):
    '''
    takes a filepath to print out the parameters in it
    Args:
        fp: FilePath of the pickle or json file with params

    Returns:
        None, prints parameters in the shell

    '''
    try:
        from helper import depickler
        import json_tricks as json

        #get the extension
        ext=fp.split('.')[-1]

        #depickle it if in pickle
        if ext=='pickle':
            a = depickler(fp)
            parameters = a['parameters']

        #json load if json
        elif ext=='json':
            with open(fp) as j:
                parameters=json.load(j)['parameters']

        #print it neatly using json dumps in a sorted fashion with nice indents
        print json.dumps(parameters, indent=4, sort_keys=True)

    except IOError as e:
        print 'no such file', e


if __name__ == '__main__':
    #plac handles the CLI with a simple argparser frontend
    import plac;
    plac.call(main)
