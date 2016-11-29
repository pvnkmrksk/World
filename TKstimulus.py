from Tkinter import *
from classes import stimulus
s=stimulus.Stimulus(nReps=4,genTimeSeries=True)
root = Tk()

def restartStim(event):
    print s.restartStim()
    print "pressed", repr(event.char)
#
def replayStim(event):
    print s.repeatCurrentStim()
    print "pressed", repr(event.char)
#
def prevStim(event):
    print s.previousStim()
    print "pressed", repr(event.char)
#
def nextStim(event):
    s.nextStim()
    # print s.nextStim()
    print "pressed", repr(event.char)
#
#
def nextFrame(event):
    root.after_idle(do_one_iteration)
    # print s.nextFrame()
    # print "pressed", repr(event.char)
#
def callback(event):
    frame.focus_set()
    print "clicked at", event.x, event.y

frame = Frame(root, width=100, height=100)
frame.bind("<w>", restartStim)
frame.bind("<s>", replayStim)
frame.bind("<a>", prevStim)
frame.bind("<d>", nextStim)
frame.bind("<g>", nextFrame)

frame.bind("<Button-1>", callback)
frame.pack()

def do_one_iteration():
    s.nextFrame()
    root.after(6,do_one_iteration)
root.mainloop()
