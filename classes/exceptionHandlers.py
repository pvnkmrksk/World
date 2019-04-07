import json
import subprocess
import time
import easygui
import pandas as pd
import rostopic
import serial
from classes.valveHandler import ValveHandler


class ExceptionHandlers():

    def __init__(self,parameters):
        self.parameters=parameters
        # self.exceptionGUI()
        # self.exceptionReplay()
        # self.exceptionArduino()
        self.exceptionROS()

    def exceptionGUI(self):
        useGui = True
        if useGui:
            from GUI_caller import jsonVR
            print "using GUI"
            with open(jsonVR, 'r') as jfile:
                self.parameters = json.load(jfile)
                for item in self.parameters['toTuplify']:
                    self.parameters[item]=tuple(self.parameters[item])
    def exceptionReplay(self):
        """
        Replay world playsback position and orientation, can be additionally used for screen capturing
        Servo instantiates the servo object
        next check for if ROS is running in the background, else start ros and the kinefly for WBAD
        """
        # If replay world for playback old data, copy existing params to prevent overwrite from the saved state of previous run
        if self.parameters["replayWorld"]:

            # save params from current instance as backup toload back into the playback file
            replay = self.parameters["replayWorld"]
            scale = self.parameters["captureScale"]
            start = self.parameters["captureStart"]
            increment = self.parameters["playbackIncrement"]

            # replayPath = easygui.fileopenbox(multiple=False, filetypes=["*.pickle"])
            # replayPath = "/home/rhagoletis/catkin/src/World/bags/2017_06_06/2017-06-06__19~51~30_haw09_01_gain~8_speed~1.0_bout~5_DC~-0.002_traj.bag_df.pickle"
            replayPath = self.parameters["replayPath"]
            fr=self.parameters['frameRecord']
            frp=self.parameters['frameRecordPath']
            rd=self.parameters['recordDur']
            rfps=self.parameters['recordFps']

            print replayPath
            repExt = replayPath.split('.')[-1]
            from analyseWorld.ipy_notebooks import ipyimports
            if repExt == 'bag':
                df, meta, p = ipyimports.bag2pickle([replayPath])
                print "hello"

            elif repExt == 'h5':
                df,meta=ipyimports.h5load(replayPath)
            # pick = pd.read_pickle(replayPath)
            # df= pick["df"]
            # meta=pick["metadata"]
            # print meta

            # # print (df.keys())
            # # slice pos and orientation and remove nans heading,pitch,roll, x,y,z and drop na which is from camera message time
            # traj = df.loc[:, "trajectory__orientation_x":"trajectory__position_z"].dropna()
            # cols = traj.columns.tolist()  # list of colums
            # cols = cols[-3:] + cols[:-3]  # reorder colums. hprpos to poshpr by python splicing
            # traj = traj[cols]  # reassign with this order
            dfPosHpr=df.loc[:,['trajectory__pPos_x', 'trajectory__pPos_y',
            'trajectory__pPos_z','trajectory__pOri_x', 'trajectory__pOri_y',
            'trajectory__pOri_z' ]]

            # //todo change current self.parameters to self.parameters saved in dataframe
            self.parameters = None

            try:
                self.parameters = json.loads(df.metadata__data.values[1])

            except:
                try:
                    # self.parameters = json.loads(df.metadata__data.values[0])
                    self.parameters = meta["parameters"]
                    print "ewo is",self.parameters
                    print "using exceprion to load params"

                except KeyError:
                    self.parameters=meta['metadata']

            # dump back params from vars
            self.parameters["replayWorld"] = replay
            self.parameters["captureScale"] = scale
            self.parameters["captureStart"] = start
            self.parameters["playbackIncrement"] = increment
            self.parameters["replayPath"] = replayPath
            self.parameters['frameRecord']=fr
            self.parameters['frameRecordPath']=frp
            self.parameters['recordDur']=rd
            self.parameters['recordFps']=rfps



            return self.parameters, df ,dfPosHpr

    def exceptionROS(self):
        # Checkif rosmaster is running else run roscore
        try:
            rostopic.get_topic_class('/rosout')  # is_rosmaster_running = True
        except rostopic.ROSTopicIOException as e:
            roscore = subprocess.Popen('roscore')  # then start roscore yourself
            time.sleep(1)  # wait a bit to be sure the roscore is really launched

            subprocess.Popen(["roslaunch", "Kinefly", "main.launch"])  # start kinefly


    def exceptionFinally(self):
        PROCNAME = ['python', 'realTimePlotter.py']
        import psutil
        import sys

        for proc in psutil.process_iter():
            # check whether the process name matches and close plotter window
            if proc.cmdline == PROCNAME:
                proc.kill()

        try:
            # s=ValveHandler(1)
            self.baud = 115200
            v1= ValveHandler(casePort=97, baud=self.baud)
            v2= ValveHandler(casePort=98, baud=self.baud)
            v3= ValveHandler(casePort=99, baud=self.baud)

            # s.move(90)
            v1.move(1)
            v2.move(1)
            v3.move(1)
            v1.move(0)
            v2.move(0)
            v3.move(0)
            # servo.move(99, 0)  # close valve to prevent odour bleeding through
            # servo.move(1, 90)  # close valve to prevent odour bleeding through

        except  (serial.serialutil.SerialException, NameError):
            print "arduino faulty finally"
            pass  # arduino disconnected or faulty, let go

        print "\n\n\nQuitting Now\n"

        sys.exit()
