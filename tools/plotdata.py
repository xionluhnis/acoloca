#!/usr/bin/env python

# Install pyqtgraph with:
#   sudo apt install python-pyqtgraph

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import os, sys
import collections
import random
import time
import math
import numpy as np
import Queue
import threading
import struct
import socket
import select

print(sys.argv[1:])

# plot source
if len(sys.argv) >= 2:
    src_file = sys.argv[1]
else:
    print('Usage: plotdata.py [file] <numlines=1> <ymin=-1> <ymax=1>')
    sys.exit(0)

# plot parameters
if len(sys.argv) >= 3:
    num_lines = int(sys.argv[2])
else:
    num_lines = 1
print('Using %d curves. Requires %d columns.' % (num_lines, num_lines+1))

if len(sys.argv) >= 4:
    y_min = float(sys.argv[3])
else:
    y_min = -1
if len(sys.argv) >= 5:
    y_max = float(sys.argv[4])
else:
    y_max = 1

class DynamicPlotter():
    def __init__(self, sampleinterval = 0.1, timewindow = 10.0, size=(1024, 640)):
        self._ncurve = num_lines
        self._interval = int(sampleinterval * 1000)
        self._bufsize = int(timewindow / sampleinterval)
        self.databuffers = [ collections.deque([0.0]*self._bufsize, self._bufsize) for i in range(self._ncurve)] 
        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        self.ys = [ np.zeros(self._bufsize, dtype=np.float) for i in range(self._ncurve)]

        # Set up plot
        self.app = QtGui.QApplication([])
        self.plt = pg.plot(title='Dynamic Plotting with PyQtGraph')
        self.plt.resize(*size)
        self.plt.showGrid(x=True, y=True)
        self.plt.setYRange(y_min, y_max)
        self.plt.setLabel('left', 'amplitude', 'V')
        self.plt.setLabel('bottom', 'time', 's')
        self.plt.setMouseEnabled(False, True)
        self.c0 = self.plt.plot(self.x, self.ys[0], pen=(255,0,0));
        self.c1 = self.plt.plot(self.x, self.ys[0], pen=(0  ,255,0));
        self.c2 = self.plt.plot(self.x, self.ys[0], pen=(0 ,0,255));
        self.c3 = self.plt.plot(self.x, self.ys[0], pen=(255,255,0));
        self.c4 = self.plt.plot(self.x, self.ys[0], pen=(0,255,255));
        self.c5 = self.plt.plot(self.x, self.ys[0], pen=(255,0,255));
        self.c6 = self.plt.plot(self.x, self.ys[0], pen=(255,255,255));
	# extruder temperature
        self.c7 = self.plt.plot(self.x, self.ys[0], pen=(0, 100, 0));
	# plotted curve
	self.curves = [ self.c0, self.c1, self.c2, self.c3, self.c4, self.c5, self.c6, self.c7];
        self.curves = self.curves[0:num_lines]

        # QTimer to refresh display
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

        # Producer thread that adds data from log file
        self.dataQueue = Queue.Queue()
        self.queueLock = threading.Lock()
        t = threading.Thread(target=self.producer, args=(self.dataQueue,self.queueLock))
        t.daemon = True
        t.start()

    def openLog(self):
        self.logfile = open(src_file, 'r')
        self.logfile.seek(0, os.SEEK_END)

    # This function uses a somewhat loosely defined heuristic. 
    # We read one line at a time from the log. If the file is written into,
    # We may sometimes get an incomplete line. In that case, we probably
    # got an EOF on the current file descriptor, so, we read the current
    # file position, reopen, seek to the position and keep reading.
    # If we notice that the file got smaller, we assume the log got
    # rotated underneath us, so, we simply throw away the partial line
    # we read, reopen the file and start afresh. If we successfuly
    # reopen and seek into the file, we simply try reading the rest of
    # the line.
    def readLogLine(self):
        line = ""
        while True:
            line = line + self.logfile.readline()
            if not line:
                # Empty line. See if the log got rotated
                currentPos = self.logfile.tell()
                while True:
                    try:
                        self.logfile = open(src_file, 'r')
                        fileSize = os.stat(src_file).st_size
                        if fileSize < currentPos:
                            # File got smaller which means log got rotated,
                            # so, we need to reopen and start from scratch
                            print "Log got rotated"
                            line = ""
                        else:
                            # More data (likely) got appended. Just continue 
                            # from where we left off. 
                            self.logfile.seek(currentPos)
                        break
                    except:
                        # If we got an exception, we couldn't open or stat
                        # the file, which means rotation is under progress
                        pass
                continue
            if line[len(line) - 1] == '\n':
                break
        return line

    def producer(self, dataQueue, queueLock):
        self.openLog()
        while True:
            line = self.readLogLine()
            num = line.split()
            try:
                with queueLock:
                    print('N=%d, line=%s' % (len(num), line))
                    if len(num) >= num_lines+1:
                        for i in range(0,num_lines):
                            self.databuffers[i].append(float(num[i+1]))
            except:
                pass

    def updateplot(self):
        filterWindow = 31
        for i in range(self._ncurve):
            with self.queueLock:
                self.ys[i][:] = self.databuffers[i]
            self.curves[i].setData(self.x, self.ys[i])

    def run(self):
        self.app.exec_()

if __name__ == '__main__':
    m = DynamicPlotter(sampleinterval = 0.05, timewindow = 20.0)
    m.run()
