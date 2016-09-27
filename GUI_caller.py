import sys, os
import json
from PyQt4.QtGui import QApplication, QMainWindow
from GUI_VR import Ui_MainWindow
from PyQt4 import QtCore, QtGui
import ast
import subprocess

#todo: the file/path-management works, but is ugly and may cause bugs
pathRun=os.path.abspath(os.path.split(sys.argv[0])[0]) #path of the runfile
pathJson= pathRun + '/jsonFiles/'
pathModel = pathRun + '/models/'
jsonDefault= pathJson + 'default.json' #path of 'default.json' #default .json-file
jsonRecent= pathJson + 'recent.json'
jsonCurrent=jsonRecent#pathJson+'temp.json' #modify a temp json file
jsonVR= pathJson + 'VR.json'

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

    settings = {}
    box = win.findChildren(QtGui.QCheckBox)
    line = win.findChildren(QtGui.QLineEdit)
    radio = win.findChildren(QtGui.QRadioButton)
    slider = win.findChildren(QtGui.QSlider)
    spinInt = win.findChildren(QtGui.QSpinBox)
    date = win.findChildren(QtGui.QDateEdit)
    spinFloat = win.findChildren(QtGui.QDoubleSpinBox)

    for item in radio:
        box.append(item)
    for item in spinInt:
        slider.append(item)
    for item in spinFloat:
        slider.append(item)

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
                text = ast.literal_eval(text)
            elif ('(' and ')') in text:#if lineEdit returns () convert to list
                text = ast.literal_eval(text)
        except:
            ui.statusbar.showMessage('Error')#todo: better message
            showError()

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

    print "pre dump",settings #todo: remove (when not needed anymore)

    with open(path, 'w') as dictFile:#dump everything
        json.dump(settings, dictFile)

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

    for item in radio:
        box.append(item)
    for item in spinInt:
        slider.append(item)
    for item in spinFloat:
        slider.append(item)

    try:
        with open(path, 'r') as dictFile:
            set = json.load(dictFile)
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
    ui.currentLabel.setText(jsonCurrent)

def openLoad(win):
    '''
    helper function to open a file and load the json
    opens a fileDialog
    :param win: current window
    :return:
    '''
    global jsonCurrent
    path=showFileDialog(win, None, pathJson)
    if path == '':
        ui.statusbar.showMessage('Canceled')
        pass
    else:
        loadSettings(win, path)
        jsonCurrent=path

def openSave(win):

    global jsonCurrent
    path = showSaveDialog(win, ui.currentLabel)
    if path == '':
        ui.statusbar.showMessage('Canceled')
        pass
    else:
        saveSettings(win, path)
        jsonCurrent=path


def showFileDialog(win, line, pathStart):
    '''
    opens file dialog, returns selected file as string
    if selected file is .json, changes jsonFile
    prints filename in defined lineEdit

    :param win:  current window
    :param line: the line to be used to display the path
    :return: the filepath selected
    '''

    fname = str(QtGui.QFileDialog.getOpenFileName(win, 'Open file', pathStart))

    if line and fname != '': #set only if given a label to setText
        line.setText(fname)
    return fname

def showSaveDialog(win, line):

    fname = str(QtGui.QFileDialog.getSaveFileName(win, "Save file as", pathJson))

    if line and fname != '': #set only if given a label to setText
        line.setText(fname)
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
    subprocess.Popen(['python', 'world.py'])

if __name__ == '__main__':
#necessary for getting the GUI running
    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(window)

    myDict={
            # 'jsonBtn':[ui.jsonBtn,showFileDialog,ui.currentLabel],
            # 'bagFullTopicsBtn':[ui.bagFullTopicsBtn,showFileDialog,ui.bagFullTopics],
            # 'bagTrajTopicsBtn':[ui.bagTrajTopicsBtn,showFileDialog,ui.bagTrajTopics],
            'skyMapBtn':[ui.skyMapBtn,showFileDialog,ui.skyMap],
            'skyMapNullBtn':[ui.skyMapNullBtn,showFileDialog,ui.skyMapNull],
            'greenTexPath':[ui.greenTexPathBtn,showFileDialog,ui.greenTexPath],
            'redTexPath':[ui.redTexPathBtn, showFileDialog, ui.redTexPath],
            'spherePath':[ui.spherePathBtn, showFileDialog, ui.spherePath],
            'treePath':[ui.treePathBtn, showFileDialog, ui.treePath],
            'treeTexPath':[ui.treeTexPathBtn, showFileDialog, ui.treeTexPath],
            'odour1':[ui.odourBtn1, showFileDialog, ui.odour1],
            'odour2':[ui.odourBtn2, showFileDialog, ui.odour2],
            'odour3':[ui.odourBtn3, showFileDialog, ui.odour3],
            'odour4': [ui.odourBtn4, showFileDialog, ui.odour4],

            }

#functions for several buttons
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
    #todo. open the last tab on close


    startVRBtn = ui.startVRBtn
    startVRBtn.clicked.connect(lambda: startVR())

    callLooper(myDict)
    try:
        loadSettings(window, jsonCurrent)#load the last run config
    except ValueError:
        pass
    window.show()
    sys.exit(app.exec_())#nothing shall be behind this line!
#don"t even dare writing somethin here!

__Version__ = '3.2'

