import pandas as pd
import serial
import rostopic
import subprocess
import time
class ExceptionHandlers():

    def __init__(self,parameters):
        self.parameters=parameters
        self.exceptionGUI()
        self.exceptionReplay()
        self.exceptionArduino()
        self.exceptionROS()

    def exceptionGUI(self):
        useGui = False
        if useGui:
            from GUI_caller import jsonVR

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
            replayPath = "/home/behaviour/catkin/src/beginner/scripts/panda/world/bags/fly4/fly4_quad_rg_gain7.0_speed_3.5_" \
                         "trial_1_2016-04-13__23:31:35.bag_df.pickle"

            print replayPath
            df = pd.read_pickle(replayPath)

            # slice pos and orientation and remove nans heading,pitch,roll, x,y,z and drop na which is from camera message time
            traj = df.loc[:, "trajectory__orientation_x":"trajectory__position_z"].dropna()
            cols = traj.columns.tolist()  # list of colums
            cols = cols[-3:] + cols[:-3]  # reorder colums. hprpos to poshpr by python splicing
            traj = traj[cols]  # reassign with this order

            # //todo change current self.parameters to self.parameters saved in dataframe
            self.parameters = None

            try:
                self.parameters = json.loads(df.metadata__data.values[1])

            except:
                self.parameters = json.loads(df.metadata__data.values[0])
                print "using exceprion to load params"

            # dump back params from vars
            self.parameters["replayWorld"] = replay
            self.parameters["captureScale"] = scale
            self.parameters["captureStart"] = start
            self.parameters["playbackIncrement"] = increment
    def exceptionArduino(self):
        # check if arduino servo is connected
        try:
            import servo
        except serial.serialutil.SerialException:
            self.parameters["loadWind"] = False
            print ("\n \n \n servo disabled \n \n \n")
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
            servo.move(99, 0)  # close valve to prevent odour bleeding through
            servo.move(1, 90)  # close valve to prevent odour bleeding through

        except  (serial.serialutil.SerialException, NameError):
            print "arduino faulty"
            pass  # arduino disconnected or faulty, let go

        print "\n\n\nQuitting Now\n"

        sys.exit()
