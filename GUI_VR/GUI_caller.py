import sys
import json
from PyQt4.QtGui import QApplication, QMainWindow
from GUI_VR import Ui_MainWindow
from PyQt4 import QtCore, QtGui
import ast

#todo: the file/path-management works, but is ugly and may cause bugs
jsonFile = 'default.json' #default .json-file
filePath = 'jsonFiles/' #path of .json-files

def isfloat(s):#todo: prob not neccassary
#helper function to check if a str is a float.
#needed to convert str to float
#same as isdigit()

    try:
        s = float(s)
        return True
    except ValueError:
        return False

def applySettings(win, path):
#collects attributes of all settings-objects
#puts all objects in sereval lists
#iterates through lists, puts object names with attribute in dictonary 'settings'
#dumps dictionary in .json-file

    settings = {}
    box = win.findChildren(QtGui.QCheckBox)
    line = win.findChildren(QtGui.QLineEdit)
    radio = win.findChildren(QtGui.QRadioButton)
    slider = win.findChildren(QtGui.QSlider)
    spinInt = win.findChildren(QtGui.QSpinBox)
    date = win.findChildren(QtGui.QDateEdit)
    spinFloat = win.findChildren(QtGui.QDoubleSpinBox)

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
            if isfloat(text):#todo: prob not necassary
                text = float(text)
            elif ('[' and ']') in text: #if lineEdit returns [] convert to list
                text = ast.literal_eval(text)
            elif ('(' and ')') in text:#if lineEdit returns () convert to list
                text = ast.literal_eval(text)
        except:
            ui.statusbar.showMessage('Error')#todo: better message
            showError()

        settings[str(name)] = text

    for item in radio: #radioButtons
        name = item.objectName()
        state = item.isChecked()
        settings[str(name)] = state

    for item in slider: #sliders
        name = item.objectName()
        value = item.value()
        settings[str(name)] = value

    for item in spinInt: #spinBoxes
        name = item.objectName()
        value = item.value()
        settings[str(name)] = value

    for item in date: #dateEdits
        name = item.objectName()
        value = str(item.date().toPyDate())#necessary, dateEdit returns strange Qdatething
        #value = value.replace('-', '')
        settings[str(name)] = value

    for item in spinFloat: #doubleSpinBocxes
        name = item.objectName()
        value = item.value()
        settings[str(name)] = value

    print "pre dump",settings #todo: remove (when not needed anymore)

    global jsonFile
    temp = ui.settingsFile.text()

    if temp != jsonFile: #when new filename inserted (in lineEdit: settingsFile)
        jsonFile = ui.settingsFile.text() #change string (filename.json)

    with open(path + jsonFile, 'w') as dictFile:#dump everything
        json.dump(settings, dictFile)

    ui.statusbar.showMessage('Settings successfully saved to ' + path + jsonFile)

def loadSettings(win, path):
    # collects attributes of all settings-objects
    # puts all objects in sereval lists
    # iterates through lists, sets attributes of objects to stored values
    #applySettings reverse

    load = {}
    box = win.findChildren(QtGui.QCheckBox)
    line = win.findChildren(QtGui.QLineEdit)
    radio = win.findChildren(QtGui.QRadioButton)
    slider = win.findChildren(QtGui.QSlider)
    spinInt = win.findChildren(QtGui.QSpinBox)
    spinFloat = win.findChildren(QtGui.QDoubleSpinBox)
    date = win.findChildren(QtGui.QDateEdit)

    with open(path, 'r') as dictFile:
        set = json.load(dictFile)

    for item in box:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        temp.setChecked(set[str(name)])

    for item in line:
        name = item.objectName()

        if str(name) == 'qt_spinbox_lineedit':
            continue

        load[name] = item
        temp = load[name]
        temp.setText(str(set[str(name)]))

    for item in radio:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        temp.setChecked(set[str(name)])

    for item in slider:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        temp.setValue(set[str(name)])

    for item in spinInt:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        temp.setValue(set[str(name)])

    for item in spinFloat:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        temp.setValue(set[str(name)])

    for item in date:
        name = item.objectName()
        load[name] = item
        temp = load[name]
        qtDate = QtCore.QDate.fromString(set[str(name)], 'yyyy-MM-dd')
        temp.setDate(qtDate)

    ui.statusbar.showMessage('Settings successfully loaded from ' +path)

#
# def updateParam():
#     with open('dict.json', 'r') as f:
#         data = json.load(f)
#         return data


def showFileDialog(win, line):
#opens file dialog, returns selected file as string
#if selected file is .json, changes jsonFile
#prints filename in defined lineEdit

    global jsonFile
    fname = str(QtGui.QFileDialog.getOpenFileName(win, 'Open file',
                                              'home/transire/catkin/src/beginner/'))

    if '.json' in fname:
        jsonFile = fname.replace('home/transire/catkin/src/beginner/GUI_VR/jsonFiles/', '')
        lineText = jsonFile
    else:
        lineText = fname.replace('home/transire/catkin/src/beginner/', '')#todo: has to be a better way to replace
    line.setText(lineText)

def caller(btn, fx, line):
    btn.clicked.connect(lambda: fx(window, line))

def callLooper(myDict):
    for key, values in myDict.iteritems():
        caller(values[0], values[1], values[2])


def fileError():
    error = QtGui.QErrorMessage()
    error.showMessage('Error')
    error.exec_()

def showError():
    error = QtGui.QMessageBox()
    error.setText('Error')
    error.exec_()

if __name__ == '__main__':
#necessary for getting the GUI running
    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(window)

    myDict={'jsonBtn':[ui.jsonBtn,showFileDialog,ui.settingsFile],
            'bagFullTopicsBtn':[ui.bagFullTopicsBtn,showFileDialog,ui.bagFullTopics],
            'bagTrajTopicsBtn':[ui.bagTrajTopicsBtn,showFileDialog,ui.bagTrajTopics],
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
    #okBtn.clicked.connect(ui.statusbar.showMessage('Ok has no function yet'))

    cancelBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Cancel)
    cancelBtn.clicked.connect(window.close)#todo: is closing the window smart?

    applyBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Apply)
    applyBtn.clicked.connect(lambda: applySettings(window, filePath))

    loadBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.Open)
    loadBtn.setText("Load")
    loadBtn.clicked.connect(lambda: loadSettings(window, filePath + jsonFile))

    defaultBtn = ui.buttonBox.button(QtGui.QDialogButtonBox.RestoreDefaults)
    defaultBtn.clicked.connect(lambda: loadSettings(window, 'jsonFiles/default.json'))

    callLooper(myDict)
    window.show()
    sys.exit(app.exec_())#nothing shall be behind this line!
#don"t even dare writing somethin here!

__Version__ = '3.2'

