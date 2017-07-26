from opterator import opterate
import numpy as np

#make auto cli
@opterate


def flowVel(voltage=None, flowRate=0.3, diameter=1.4, wantedVelocity=None,
            csvPath = '/home/rhagoletis/catkin/src/helpingWorld/flowCalib/volt_flow.csv', ):
    '''

    Units:
    voltage in V
    flowRate in L/min
    diameter in mm
    wantedVelocity in m/s
    csvPath can be given incase a different LUT is needed
    Either voltage or flowRate or wantedVelocity is necessary to give the others
     Else ideal config is given

    Returns : velocity in m/s
    '''

    #cast all inputs to float
    flowRate=float(flowRate)
    diameter=float(diameter)
    try:
        voltage=float(voltage)
    except TypeError:
        pass
    try:
        wantedVelocity=float(wantedVelocity)
    except TypeError:
        pass


    def getnearpos(array, value):
        '''
        returns the index of the nearest value in the array
        Args:
            array:the Look Up Table
            value: the item to look for

        Returns:

        '''
        idx = (np.abs(array - value)).argmin()
        return idx

    #read the LUT
    c = np.genfromtxt(csvPath, delimiter=',')
    flow = c[:, 0]
    volt = c[:, 1]

    #if there is a wanted velocity, all other are params are calculated
    if wantedVelocity is not None:
        flowRate = (wantedVelocity * 6 * np.pi * (diameter ** 2)) / (4 * 100)
        voltage = None #to override any spurious input

    #find the corresponding flowrate if voltage given
    if voltage is not None:
        flowRate=flow[getnearpos(volt,voltage)]

    #find the corresponding voltage for the desired flowrate
    else:
        voltage=volt[getnearpos(flow,flowRate)]

    #calculate the velocity from continuity
    vel=((flowRate)*4*100)/(6 * np.pi * (diameter) ** 2)

    print ("diameter\t:%0.3f mm") % diameter
    print ("voltage\t\t:%0.3f V")%voltage
    print ("flow rate\t:%0.3f L/min")%flowRate
    print ("\nflow velocity\t:%0.3f m/s\n")%vel


if __name__ == '__main__':
    #plac handles the CLI with a simple argparser frontend
    # import plac;
    # plac.call(flowVel)
    flowVel()