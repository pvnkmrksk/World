import ast,os,sys
import signal
from PyQt4 import QtCore, QtGui
from PyQt4.Qwt5 import Qwt

import pyqtgraph as pg
from GUI.makeGUI import Ui_RhagGUI
from PyQt4.QtCore import QTimer
from PyQt4.QtGui import QApplication, QMainWindow
from classes.rosSubscriber import RosSubscriber
from helping.importHelper import *
from pathlib import Path
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os
import sys



from qrangeslider import QRangeSlider

pathRun=os.path.abspath(os.path.split(sys.argv[0])[0]) #path of the runfile
pathJson= pathRun + '/GUI/jsonFiles/'
pathModel = pathRun + '/models/'
jsonDefault= pathJson + 'default.json' #path of 'default.json' #default .json-file
jsonRecent= pathJson + 'recent.json'
jsonCurrent=jsonRecent#pathJson+'temp.json' #modify a temp json file
jsonVR= pathJson + 'VR.json'
traj = 0
signal.signal(signal.SIGINT, signal.SIG_DFL)
filePath=Path(sys.path[0])




def saveSettings(win, path):
    '''
    collects attributes of all settings-objects
    puts all objects in sereval lists
    iterates through lists, puts object names with attribute in dictonary 'settings'
    dumps dictionary in .json-file

    :param win: Current window
    :param path: File Path to which parameters of object to be dumped to
    :return:

    '''
    #todo. rebase repetitive settings


    settings = {}
    box = win.findChildren(QtGui.QCheckBox)
    line = win.findChildren(QtGui.QLineEdit)
    radio = win.findChildren(QtGui.QRadioButton)
    slider = win.findChildren(QtGui.QSlider)
    spinInt = win.findChildren(QtGui.QSpinBox)
    date = win.findChildren(QtGui.QDateEdit)
    spinFloat = win.findChildren(QtGui.QDoubleSpinBox)
    combo = win.findChildren(QtGui.QComboBox)

    for item in radio:
        box.append(item)
    for item in spinInt:
        slider.append(item)
    for item in spinFloat:
        slider.append(item)

    for item in combo:
        name = item.objectName()
        text = str(item.currentText())
        settings[str(name)] = text

    for item in box: #checkboxes
        name = item.objectName()
        state = item.isChecked()
        settings[str(name)] = state

    for item in line: #lineEdits
        name = item.objectName()

        if str(name) == 'qt_spinbox_lineedit': #Bugfix: dateEdit, spinBox, doubleSpinbox
            continue #get found by findChildren lineEdit, skip them here

        text = item.text()
        text = str(text)
        try:
            if ('[' and ']') in text: #if lineEdit returns [] convert to list

                #if there is a fraction representation, converts it to float
                if '/' in text:

                    #first convert the string such that, the elements are shielded by ast eval as a string
                    #  as ast eval will cough up malformed string
                    text=text.replace('[', "['").replace(',', "','").replace(']', "']")

                    #then convert the string to a list with ast eval, with the fractions treated as strings
                    text=ast.literal_eval(text)

                    #now convert the string fractions into a fraction then to a float
                    text=[float(fractions.Fraction(x)) for x in text]

                    #Yay, now we have parsed the string of list rep into an actual list

                else:#if not a fraction contatining list
                    text = ast.literal_eval(text)

            elif ('(' and ')') in text:#if lineEdit returns () convert to list
                text = ast.literal_eval(text)
            else :
                pass#non list tuple items
                # print 'what have you entered here?',text
        except Exception as e:
            ui.statusbar.showMessage('Error')#todo: better message
            print "error is",e
            showError('list gone wrong')
        settings[str(name)] = text

    for item in slider: #sliders
        name = item.objectName()
        value = item.value()
        settings[str(name)] = value

    for item in date: #dateEdits
        name = item.objectName()
        value = str(item.date().toPyDate())#necessary, dateEdit returns strange Qdatething
        #value = value.replace('-', '')
        settings[str(name)] = value


    tupleList=[]
    for key, value in settings.iteritems():
        if type(value)==tuple:
            tupleList.append(key)

    settings['toTuplify']=tupleList

    with open(path, 'w') as dictFile:#dump everything
        json.dump(settings, dictFile, sort_keys=True)
        # print json.dumps(settings, sort_keys=True)


    ui.statusbar.showMessage('Settings successfully saved to ' + path )

def loadSettings(win,path):
    # collects attributes of all settings-objects
    # puts all objects in sereval lists
    # iterates through lists, sets attributes of objects to stored values
    #saveSettings reverse
    global jsonCurrent
    jsonCurrent=path
    load = {}
    box = win.findChildren(QtGui.QCheckBox)
    line = win.findChildren(QtGui.QLineEdit)
    radio = win.findChildren(QtGui.QRadioButton)
    slider = win.findChildren(QtGui.QSlider)
    spinInt = win.findChildren(QtGui.QSpinBox)
    spinFloat = win.findChildren(QtGui.QDoubleSpinBox)
    date = win.findChildren(QtGui.QDateEdit)
    combo = win.findChildren(QtGui.QComboBox)

    for item in radio:
        box.append(item)
    for item in spinInt:
        slider.append(item)
    for item in spinFloat:
        slider.append(item)

    try:
        with open(path, 'r') as dictFile:
            set = json.load(dictFile)
            try:
                for item in set['toTuplify']:
                    set[item]=tuple(set[item])
                # print set
            except KeyError:
                pass

    except IOError:
            ui.statusbar.showMessage('.json-file not changed')
            return


    for item in box:
        try:
            name = item.objectName()
            load[name] = item
            temp = load[name]
            temp.setChecked(set[str(name)])
        except KeyError:
            pass

    for item in combo:
        try:
            name = item.objectName()
            load[name] = item
            temp = load[name]
            index = temp.findText(set[str(name)], QtCore.Qt.MatchFixedString)
            temp.setCurrentIndex(index)
        except:
            pass

    for item in line:
        try:
            name = item.objectName()

            if str(name) == 'qt_spinbox_lineedit':
                continue

            load[name] = item
            temp = load[name]
            temp.setText(str(set[str(name)]))
        except KeyError:
            pass

    for item in slider:
        try:

            name = item.objectName()
            load[name] = item
            temp = load[name]
            temp.setValue(set[str(name)])
        except KeyError:
            pass

    for item in date:
        try:

            name = item.objectName()
            load[name] = item
            temp = load[name]
            qtDate = QtCore.QDate.fromString(set[str(name)], 'yyyy-MM-dd')
            temp.setDate(qtDate)
        except KeyError:
            pass


    ui.statusbar.showMessage('Settings successfully loaded from ' +path)
    # ui.currentLabel.setText(jsonCurrent)


def openLoad(win):
    '''
    helper function to open a file and load the json
    opens a fileDialog
    :param win: current window
    :return:
    '''
    global jsonCurrent
    path=showFileDialog(win, None, pathJson)
    print "path is", path
    if path == '':
        ui.statusbar.showMessage('Canceled')
        pass
    else:
        loadSettings(win, path)
        jsonCurrent=path

def openSave(win):

    global jsonCurrent
    path = showSaveDialog(win, pathJson)
    print "path is",path
    if path == '':
        ui.statusbar.showMessage('Canceled')
        pass
    else:
        saveSettings(win, path)
        jsonCurrent=path


def showFileDialog(win, line, pathStart,local=True):
    '''
    opens file dialog, returns selected file as string
    if selected file is .json, changes jsonFile
    prints filename in defined lineEdit

    :param win:  current window
    :param line: the line to be used to display the path
    :return: the filepath selected
    '''

    fname = str(QtGui.QFileDialog.getOpenFileName(win, 'Open file', pathStart))
    if local:
        try:
            fname = Path(fname).relative_to(filePath)#local path, get path relayive to repo root directory so that bugs
        except ValueError:
            print "SOmething off with fnames"

    # dues to different usernames are avoided.
    fname = str(fname)
    if line and fname != '': #set only if given a label to setText
        line.setText(fname)

    return fname


def showSaveDialog(win, line):

    fname = str(QtGui.QFileDialog.getSaveFileName(win, "Save file as", pathJson))

    try:
        if line and fname != '': #set only if given a label to setText
            line.setText(fname)
        return fname
    except AttributeError:
        return fname

def caller(btn, fx, line):
    btn.clicked.connect(lambda: fx(window, line, pathModel))

def callLooper(myDict):
    for key, values in myDict.iteritems():
        caller(values[0], values[1], values[2])

def showError(message):
    error = QtGui.QMessageBox()
    error.setWindowTitle('Error message')
    error.setText(message)
    error.exec_()

def saveClose(win):
    saveSettings(win, jsonRecent)
    win.close()

def startVR():
    global procVR
    # procVR=subprocess.Popen([sys.executable,'~/Desktop/Test4/Impose.x86_64 '])
    procVR=subprocess.call('~/Desktop/Test4/Impose.x86_64 ')
    # procVR=subprocess.Popen(['python', 'main.py'])
    ui.tabWidget.setCurrentIndex(5)

def startRqt():
    global procRqt
    procRqt=subprocess.Popen(['rqt_plot'])

    #showError("VR is not available")
def stopVR():
    try:
        procVR.kill()
        try:
            # s=ValveHandler(1)
            baud = 115200
            v1= ValveHandler(casePort=97, baud=baud)
            v2= ValveHandler(casePort=98, baud=baud)
            v3= ValveHandler(casePort=99, baud=baud)

            # s.move(90)
            v1.move(1)
            v2.move(1)
            v3.move(1)
            v1.move(0)
            v2.move(0)
            v3.move(0)
            # servo.move(99, 0)  # close valve to prevent odour bleeding through
            # servo.move(1, 90)  # close valve to prevent odour bleeding through
            print "disabled valves in VR"
        except  (serial.serialutil.SerialException, NameError):
            print "arduino faulty finally GUI"
            pass  # arduino disconnected or faulty, let go
    except NameError:
        print "VR not running"
        pass

def startRoscore():
    subprocess.Popen(['roscore'])

def startCameraParam():
    # text expansion of bash is disabled  by default for security concerns.
    # Since this is not user input string, security isn't a issue in this case
    # subprocess.Popen(['gedit', '~/a.txt'],shell=True)
    # subprocess.Popen(['gedit', '~/src.git/Kinefly/launch/rhag/camera_1394.launch'])
    subprocess.call('gedit ~/src.git/Kinefly/launch/rhag/camera_1394.launch',shell=True)

def startWbad():
    subprocess.Popen(["roslaunch", "Kinefly", "main.launch"])  # start kinefly


def clbk(data):
    global traj
    traj = data
    return traj

def resetView(cx=512,cy=512,off=20):

    # off=20
    s0.setRange(xRange=(cx-off,cx+off),yRange=(cy-off,cy+off))
    s1.setRange(xRange=(cx-off,cx+off),yRange=(cy-off,cy+off))
    s2.setRange(xRange=(cx-off,cx+off),yRange=(cy-off,cy+off))
    s3.setRange(xRange=(cx-off,cx+off),yRange=(cy-off,cy+off))
    r0.setRange(xRange=(cx-off,cx+off),yRange=(cy-off,cy+off))

    # s0.setRange(xRange=(512-off,512+off),yRange=(512-off,512+off))
    # s1.setRange(xRange=(512-off,512+off),yRange=(512-off,512+off))
    # s2.setRange(xRange=(512-off,512+off),yRange=(512-off,512+off))
    # s3.setRange(xRange=(512-off,512+off),yRange=(512-off,512+off))


def tick():

    global traj, vals,curve,traj0s,traj1s,traj2s,traj3s,traj0r
    try:
        ui.compassServo.setValue(-traj.servoAngle)
        ui.compassServo_2.setValue(-traj.servoAngle)
        ui.compassHeading.setValue(-traj.pOri.y)
        ui.compassSlip.setValue(traj.slip)
        # ui.compassSlip.setValue((((traj.slip+180)%360)-180-90)%360)

        ui.lcdServoAngle.display(traj.servoAngle)
        ui.lcdHeadingAngle.display(traj.pOri.x%360)
        ui.lcdSlipAngle.display(traj.slip)

        ui.livePosition.setText(str(traj.pPos))
        # if bool(traj.valve1):
        #     ui.odourLed.on()
        #     ui.replayOdourLed.on()
        # else:
        #     ui.odourLed.off()
        #     ui.replayOdourLed.off()

        stateText="\ntrial\t\t: "+str(traj.trial)+ \
                  "\nrunNum\t\t: " + str(traj.runNum) + \
                  "\ncase\t\t: " + str(traj.case_) + \
                  \
                  "\n\nDCoffset\t: " + str(traj.DCoffset) + \
                  "\nisFlying\t\t: " + str(bool(traj.isFlying)) + \
                  "\npacketFrequency\t: " + str(traj.packetFrequency) + \
                  \
                  "\n\nvalve1\t\t: " + str(bool(traj.valve1)) + \
                  "\nvalve2\t\t: " + str(bool(traj.valve2)) + \
                  "\nvalve3\t\t: " + str(bool(traj.valve3)) + \
                  \
                  "\n\nslip\t\t: " + str((traj.slip)) + \
                  "\ngroundSpeed\t: " + str((np.round(traj.groundSpeed,2))) + \
                  \
                  \
                  "\n\n\ngain\t\t: "+str(traj.gain)+\
                  "\nHeading Control\t: "+str(bool(traj.headingControl))+\
                  "\nSpeed Control\t: "+str(bool(traj.speedControl))+\
                  "\nspeed\t\t: "+str(traj.speed)+ \
                  "\npacketDuration\t: "+str(traj.packetDuration)+ \
                  "\nreset\t\t: " + str(bool(traj.reset))

                  # "\nheadingControl\t\t: " + str(bool(traj.headingControl)) + \
                  # "trial\t\t: "+str(traj.trial)+\





        ui.liveState.setText(stateText)
        if not ui.pausePlot.isChecked():
            # spots = [{'pos': np.array([traj.pPos.x, traj.pPos.y])
            #              , 'data': 1}]
            # spots=np.array([traj.pPos.x, traj.pPos.y])

            # s1.addPoints(pos=[(traj.pPos.x, traj.pPos.y)])
            # my_plot.addItem(s1)
            def quadPlot(trajs,s):
                trajs = np.vstack((trajs,
                               np.array([traj.pPos.x,
                                         traj.pPos.y,
                                         traj.runNum,
                                         traj.trial,
                                         traj.runNum*traj.trial])))
                s.clear()
                if traj.valve2==True:
                    sp=(255,0,0)

                else:
                    sp=(0,0,255)
                # print sp
                # print 'a',trajs[:,3]
                s.plot(trajs[:, 0], trajs[:, 1], pen=None,
                       symbol='o', symbolPen=(255,0,0),
                       symbolBrush=(255,0,0),symbolSize=4)
                # s.addPoints(trajs[-1, 0], trajs[-1, 1], pen=None, symbol='o', symbolPen=(255, 255, 0), symbolSize=2)
                s.plot([trajs[-1, 0]], [trajs[-1, 1]], pen=(6),
                       symbolBrush=(0,0,255), symbolPen='w',
                       symbol='o', symbolSize=4)

                return trajs


            if traj.case_ ==0:
                traj0s=quadPlot(traj0s,s0)
            elif traj.case_ ==1:
                traj1s=quadPlot(traj1s,s1)
                traj0r=quadPlot(traj0r,r0)

            elif traj.case_ ==2:
                traj2s=quadPlot(traj2s,s2)
                traj0r=quadPlot(traj0r,r0)


            elif traj.case_ ==3:
                traj3s=quadPlot(traj3s,s3)

            #
            # traj1s=np.vstack((traj1s,np.array([traj.pPos.x,traj.pPos.y])))
            # vals=np.append(vals,traj.wbad)
            # s1.clear()
            # s1.plot(traj1s[:,0],traj1s[:,1], pen=None, symbol='o', symbolPen=None, symbolSize=5)

            vals = np.append(vals, traj.wbad)
            y, x = np.histogram(vals, bins=np.linspace(-1, 1, 60))

            ## notice that len(x) == len(y)+1
            ## We are required to use stepMode=True so that PlotCurveItem will interpret this data correctly.
            curve = pg.PlotDataItem(x, y, stepMode=True, fillLevel=0, brush=(0, 0, 255, 80))

            line=pg.InfiniteLine(pos=vals.mean())
            my_hist.clear()
            my_hist.addItem(curve)
            my_hist.addItem(line)

            def trackingFly(w=10):
                resetView(cx=traj.pPos.x,cy= traj.pPos.y,off=w)#cx=trajs[-1, 0],cy= trajs[-1, 1]), off=w)

            if ui.trackFly.isChecked():
                a=2.**(0.05*(-ui.verticalSlider.value()))
                # print "a",a
                trackingFly(w=a)
                # trackingFly(w=ui.trackWidth.value())

                # ui.trackWidth.setValue(s0.get)

            # # Example 1
            # rs1 = QRangeSlider()
            # rs1.show()
            # rs1.setWindowTitle('example 1')
            # rs1.setRange(15, 35)
            # rs1.setBackgroundStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222, stop:1 #333);')
            # rs1.setSpanStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #282, stop:1 #393);')
            #






    except AttributeError as e:
        # print "something bad,no gui update",e
        pass


def setHeadingLcd():
    ui.lcdNumber_3.display(ui.compassHeading.value() - 90)  # offset origin to E and not North



def clearPlot():
    global vals,curve,traj0s,traj1s,traj2s,traj3s,traj0r
    vals = np.array([])
    traj0s = np.array([0,0,0,0,0])
    traj1s = np.array([0,0,0,0,0])
    traj2s = np.array([0,0,0,0,0])
    traj3s = np.array([0,0,0,0,0])
    traj0r = np.array([0,0,0,0,0])

    # traj0s = np.array([0,0])
    # traj1s = np.array([0,0])
    # traj2s = np.array([0,0])
    # traj3s = np.array([0,0])
    # curve.clear()
    # my_hist.clear()
    # s1.points()
    s1.clear()
    # my_plot.clear()

    # my_plot.removeItem(s1)
    #todo. clear plot os not working
if __name__ == '__main__':
#necessary for getting the GUI running
    try:
        rostopic.get_topic_class('/rosout')  # is_rosmaster_running = True
    except rostopic.ROSTopicIOException as e:
        roscore = subprocess.Popen('roscore')  # then start roscore yourself
        time.sleep(1)  # wait a bit to be sure the roscore is really launched

    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = Ui_RhagGUI()
    ui.setupUi(window)
# # Example 1
# rs1 = QRangeSlider()
# rs1.show()
# rs1.setWindowTitle('example 1')
# rs1.setRange(15, 35)
# rs1.setBackgroundStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222, stop:1 #333);')
# rs1.setSpanStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #282, stop:1 #393);')

    # rs1 = QRangeSlider()
    ui.trajRange=QRangeSlider(ui.tab_2)
    # ui.trackFly = QtGui.QCheckBox(ui.tab_2)
    # ui.trajRange.setGeometry(QtCore.QRect(540, 47, 111, 22))  #functions for several buttons

    ui.trajRange.show()
    # rs1.setWindowTitle('example 1')
    ui.trajRange.setRange(15, 35)
    ui.trajRange.setBackgroundStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222, stop:1 #333);')
    ui.trajRange.setSpanStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #282, stop:1 #393);')
    #applying and loading settings, closing etc.

    okBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Ok)#todo: Ok needs a function
    okBtn.clicked.connect(lambda : saveSettings(window, jsonVR))
    #okBtn.clicked.connect(ui.statusbar.showMessage('Ok has no function yet'))

    cancelBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Cancel)
    cancelBtn.clicked.connect(lambda: saveClose(window))#save to recent and close
    # app.aboutToQuit.connect(lambda :saveClose(window))
    #todo: is closing the window smart?
    #todo: is saving last known config on cancel smart?

    defaultBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.RestoreDefaults)
    defaultBtn.clicked.connect(lambda: loadSettings(window, jsonDefault))

    saveBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Save)
    saveBtn.clicked.connect(lambda: saveSettings(window, jsonCurrent))

    saveAsBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.SaveAll)
    saveAsBtn.setText("Save as")
    saveAsBtn.clicked.connect(lambda: openSave(window))

    loadBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Open)
    loadBtn.setText("Load")#because it is actually open button
    # loadBtn.clicked.connect(lambda: loadSettings(window, filePath + jsonFile))
    loadBtn.clicked.connect(lambda: openLoad(window))

    resetBtn=ui.buttonBox.button(QtGui.QDialogButtonBox.Reset)
    resetBtn.clicked.connect(lambda: loadSettings(window, jsonCurrent))

    # recordPathBtn = ui.buttonBox.button(ui.frameRecordPathBtn)
    # file = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
    # ui.frameRecordPathBtn.clicked.connect(lambda: showSaveDialog(window, ui.frameRecordPath))
# 'frameRecordPath': [ui.frameRecordPathBtn,showSaveDialog,ui.frameRecordPath],


    #todo. open the last tab on close


    ui.startVRBtn.clicked.connect(lambda: startVR())
    ui.stopVRBtn.clicked.connect(lambda: stopVR())
    ui.camParamBtn.clicked.connect(lambda: startCameraParam())
    ui.wbadBtn.clicked.connect(lambda: startWbad())
    ui.rqtBtn.clicked.connect(lambda: startRqt())
    ui.resetView.clicked.connect(lambda :resetView())
    ui.clearPlot.clicked.connect(lambda :clearPlot())
    ui.clearPlot_2.clicked.connect(lambda :clearPlot())

    ui.replayPathBtn.clicked.connect(lambda :showFileDialog(window, ui.replayPath, pathModel,local=False))

# ui.greenTexPathBtn, showFileDialog, ui.greenTexPath

    origin =270 #unity/panda3d

    ui.compassServo.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Arrow))
    ui.compassServo.setOrigin(origin)
    ui.compassServo_2.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Arrow))
    ui.compassServo_2.setOrigin(origin)
    # to set north as north
    # always start rosnode inside main else imports end in loop

    ui.compassHeading.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Arrow))
    ui.compassHeading.setOrigin(origin)# to set north as north

    ui.compassSlip.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Arrow))
    ui.compassSlip.setOrigin(origin)# to set north as north





    RosSubscriber('GUI', '/trajectory', MsgTrajectory, clbk)


    '''
    replay plot
    '''
    replay_plot= pg.GraphicsLayoutWidget()
    ui.replayTrajectoryLayout.addWidget(replay_plot)
    r0 = replay_plot.addPlot(title="1")


    # my_plot = pg.PlotWidget()
    # ui.trajectoryLayout.addWidget(my_plot)
    # s1 = pg.ScatterPlotItem(size=2, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
    my_plot = pg.GraphicsLayoutWidget()
    ui.trajectoryLayout.addWidget(my_plot)

    s1 = my_plot.addPlot(title="1")
    s0 = my_plot.addPlot(title="0")
    my_plot.nextRow()
    s2 = my_plot.addPlot(title="2")
    s3 = my_plot.addPlot(title="3")


    s3.setXLink(s0)
    s3.setYLink(s0)
    s2.setXLink(s0)
    s2.setYLink(s0)
    s1.setXLink(s0)
    s1.setYLink(s0)
    my_hist=pg.PlotWidget()
    ui.histog.addWidget(my_hist)
    global vals,traj0s,traj1s,traj2s,traj3s,replay
    traj0s = np.array([0, 0, 0, 0, 0])
    traj1s = np.array([0, 0, 0, 0, 0])
    traj2s = np.array([0, 0, 0, 0, 0])
    traj3s = np.array([0, 0, 0, 0, 0])
    traj0r = np.array([0, 0, 0, 0, 0])
    # traj0s = np.array([0,0])
    # traj1s = np.array([0,0])
    # traj2s = np.array([0,0])
    # traj3s = np.array([0,0])
    vals = np.array([0])
    y, x = np.histogram(vals, bins=np.linspace(-0.5, 0.5, 80))
    resetView()
    ## notice that len(x) == len(y)+1
    ## We are required to use stepMode=True so that PlotCurveItem will interpret this data correctly.
    # curve = pg.PlotItem(x, y, stepMode=True, fillLevel=0, brush=(0, 0, 255, 80))
    #
    # my_hist.addItem(curve)

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(1000/60.)

    myDict = {
        # 'greenTexPath': [ui.greenTexPathBtn, showFileDialog, ui.greenTexPath],
        # 'redTexPath': [ui.redTexPathBtn, showFileDialog, ui.redTexPath],
        # 'object1': [ui.obj1PathBtn, showFileDialog, ui.object1],
        # 'object2': [ui.obj2PathBtn, showFileDialog, ui.object2],
        # 'treeTexPath': [ui.treeTexPathBtn, showFileDialog, ui.treeTexPath],
        # 'skyMapBtn': [ui.skyMapBtn, showFileDialog, ui.skyMap],
        # 'skyMapNullBtn': [ui.skyMapNullBtn, showFileDialog, ui.skyMapNull],
        # 'modelHeightMap': [ui.modelHeightMapBtn, showFileDialog, ui.modelHeightMap],
        # 'modelTextureMap': [ui.modelTextureMapBtn, showFileDialog, ui.modelTextureMap],
        # 'modelTextureMapNull': [ui.modelTextureMapNullBtn, showFileDialog, ui.modelTextureMapNull],

        # 'odour1': [ui.odourBtn1, showFileDialog, ui.odour1],
        # 'odour2': [ui.odourBtn2, showFileDialog, ui.odour2],
        # 'odour3': [ui.odourBtn3, showFileDialog, ui.odour3],
        # 'odour4': [ui.odourBtn4, showFileDialog, ui.odour4],
        # 'odour1Mask': [ui.odour1MaskBtn, showFileDialog, ui.odour1Mask],
        # 'odour2Mask': [ui.odour2MaskBtn, showFileDialog, ui.odour2Mask],
        # 'odour3Mask': [ui.odour3MaskBtn, showFileDialog, ui.odour3Mask],
        # 'odour4Mask': [ui.odour4MaskBtn, showFileDialog, ui.odour4Mask],
        # 'beepPath': [ui.beepPathBtn,showFileDialog,ui.beepPath],


    }
    callLooper(myDict)

    try:
        loadSettings(window, jsonCurrent)#load the last run config
    except ValueError:
        pass

    window.show()

    try:
        (app.exec_())
    except KeyboardInterrupt:
        sys.exit()

#nothing shall be behind this line!
#don"t even dare writing somethin here!

__Version__ = '3.2'

