
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

# Line Sensor Reads
LEFT_CENTER = ['0', '1']
LEFT_LEFT = ['1', '1']

CENTER_LEFT = [['1', '1', '0'], ['1', '0', '0']]
CENTER_CENTER = ['1', '1', '1']
CENTER_RIGHT = [['0', '1', '1'], ['0', '0', '1']]

RIGHT_CENTER = ['1', '0']
RIGHT_RIGHT = ['1', '1']

EMPTY_SIDE = ['0', '0']
EMPTY_CENTER = ['0', '0', '0']

# PID

KP = 0.09
KI = 0
KD = 0.09



class LineSensorFilter():
    def __init__(self, size, first):
      # size of the buffer
      self.size = size
      # circular buffer (init copies the first value to all buffer entries)
      self.buffer = [first for i in range(size)]
      # reference to the oldest buffer position
      self.last = 0

    """ Updates the buffer with a lineSensorRead. """
    def update(self, lineSensorRead):
        self.buffer[self.last] = lineSensorRead
        self.last = (self.last+1) % self.size

    """ Returns a filtered lineSensorRead as the average of the last 'size' updates."""
    def read(self):
        filtered = []
        for sensor in range(7):
            total = ""
            for read in self.buffer:
                total += read[sensor] 
            filtered.append(str(total.count('1') // ((self.size // 2) + 1)))
        #print(self.buffer, "buffer", sep=" <-> ")
        return filtered

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        # Initialize line sensor filter
        self.readSensors()  # with the first line sensor read
        self.lineSensorFilter = LineSensorFilter(3, self.measures.lineSensor)        
        self.last_error = 0
        self.error = 0
        self.p = 0
        self.i = 0
        self.d = 0

        
        self.count = 0


        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                #self.wander()
                self.drive()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                #self.wander()
                self.drive()
    
    def getPID(self):
        pidValue = (KP*self.p) + (KI*self.i) + (KD*self.d)
        return round(pidValue,2)
  
    def drive(self):
        self.count = self.count+1
        # Get the line sensor read
        lineSensorRead = self.measures.lineSensor
        # Send it to the filter
        self.lineSensorFilter.update(lineSensorRead)
        # Get the filtered line sensor read
        lineSensorFilteredRead = self.lineSensorFilter.read()
        # Extract the sensor values
        l2, l1, l0, c, r0, r1, r2 = list(map(int,lineSensorFilteredRead))
        # Compute the error
        # 00 - Lerr: 0 | Rerr: 0
        # 01 - Lerr: 1 | Rerr: 3
        # 10 - Lerr: 3 | Rerr: 1
        # 11 - Lerr: 2 | Rerr: 2
        leftError =  l1 + l2 + (l2 and not l1)
        rightError = r1 + r2 + (r2 and not r1)
        self.error = leftError - rightError
        # Get the PID control
        self.p = self.error
        self.i = self.i + self.error
        self.d = self.error - self.last_error
        pid = self.getPID()
        # Compute the powers of the motors
        lpow = round(0.1 - pid, 2)
        rpow = round(0.1 + pid, 2)
        # Send the drive command
        self.driveMotors(lpow, rpow)
        # Keep the last error
        self.last_error = self.error

        # debug
        #print(lineSensorRead, '<=>', lineSensorFilteredRead,'p:', self.p,'i:', self.i, 'd:', self.d, 'PID:', pid, 'motors:', lpow, rpow)
        
        # log
        # ti, error, p, i, d, pid, lpow, rpow    
        print(self.count, self.error, self.p, self.i, self.d, lpow, rpow, sep=',')

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
