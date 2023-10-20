
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
# monitoring
import csv

CELLROWS=7
CELLCOLS=14

# debug
class bcolors:
    RED = '\033[31m'
    GREEN = '\033[32m'
    ENDC = '\033[0m'

    def color(lineSensorSeq):
        return ''.join(map(
            lambda val: bcolors.RED+val+bcolors.ENDC if val == '0' else bcolors.GREEN+val+bcolors.ENDC, lineSensorSeq)
        )

# Filter
FILTER_SIZE = 3
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
    
# PID
KP = 0.0020
KI = 0.0
KD = 0.0015
CKP = 0.01
CKI = 0.0
CKD = 0
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.p = 0
        self.i = 0
        self.d = 0
        self.pid = 0

        self.lastError = 0

    def getPID(self, error):
        self.p = self.kp * (error)
        self.i = self.ki * (self.i + error)
        self.d = self.kd * (error - self.lastError)

        self.lastError = error

        self.pid = self.p + self.i + self.d
        return self.pid

# Displacement
class GPSFilter:
    def __init__(self,x,y):
        self.init_x = x
        self.init_y = y
        self.x = self.init_x
        self.y = self.init_y

    def update(self,x,y):
        self.x = round(x-self.init_x, 2)
        self.y = round(y - self.init_y, 2)

# Orientation
class dir:
    N = 90
    NE = 45
    L = 0
    SE = -45
    S = -90
    SO = -135
    O = -180
    NO = 135

class curv:
    cl_135 = '00011110'
    cr_135 = '00101101'

    cl_45 = cr_135
    cr_45 = cl_135

    cl_90 = '001100'
    cr_90 = cl_90

    cl_90_135 = '001110'
    cr_90_135 = '001101'

    cl_45_90 = '00101100'
    cr_45_90 = '00011100'

    cl_45_135 = '0010110100011110'
    cr_45_135 = '000111101101'

    cl_45_90_135 = '00101110'
    cr_45_90_135 = '00011101'


class SequenceCounter:
    def __init__(self, seq):
        self.seq = seq
        self.counter = 1

    def count(self):
        self.counter = self.counter + 1

    def __repr__(self):
        return f'({bcolors.color(self.seq)}, {str(self.counter)})'

class DirectionIdentifier:
    def __init__(self):
        self.foundLeft = []
        self.lastLeft = ''

        self.foundRight = []
        self.lastRight = ''

        self.foundCenter = []
        self.lastCenter = ''

    def push(self, lineSensor):
        left = lineSensor[:2]
        right = lineSensor[5:]
        center = lineSensor[2:5]

        if left == self.lastLeft:
            self.foundLeft[-1].count()
        else:
            self.lastLeft = left
            self.foundLeft.append(SequenceCounter(left))

        if right == self.lastRight:
            self.foundRight[-1].count()
        else:
            self.lastRight = right
            self.foundRight.append(SequenceCounter(right))

        if center == self.lastCenter:
            self.foundCenter[-1].count()
        else:
            self.lastCenter = center
            self.foundCenter.append(SequenceCounter(center))

    def getDirs(self):
        dirs = []

        curvesLeft = ''.join(map(lambda sc: ''.join(sc.seq), self.foundLeft))
        curvesRight = ''.join(map(lambda sc: ''.join(sc.seq), self.foundRight))
        center = ''.join(map(lambda sc: ''.join(sc.seq), self.foundCenter))

        # if curv.cl_135 in curvesLeft:
        #     dirs.append(dir.NE)
        # if curv.cl_90 in curvesLeft:
        #     dirs.append(dir.N)
        # if curv.cl_45 in curvesLeft:
        #     dirs.append(dir.NO)

        # if curv.cr_135 in curvesRight:
        #     dirs.append(dir.SE)
        # if curv.cr_90 in curvesRight:
        #     dirs.append(dir.S)
        # if curv.cr_45 in curvesRight:
        #     dirs.append(dir.SO)
        
        if curvesLeft == curv.cl_135:
            dirs.append(dir.NE)
        elif curvesLeft == curv.cl_45:
            dirs.append(dir.NO)
        elif curvesLeft == curv.cl_90:
            dirs.append(dir.N)
        elif curvesLeft == curv.cl_45_90:
            dirs.append(dir.N)
            dirs.append(dir.NO)
        elif curvesLeft == curv.cl_90_135:
            dirs.append(dir.N)
            dirs.append(dir.NE)
        elif curvesLeft == curv.cl_45_135:
            dirs.append(dir.NE)
            dirs.append(dir.NO)
        elif curvesLeft == curv.cl_45_90_135:
            dirs.append(dir.NE)
            dirs.append(dir.NO)
            dirs.append(dir.N)

        if curvesRight == curv.cr_135:
            dirs.append(dir.SE)
        elif curvesRight == curv. cr_45:
            dirs.append(dir.SO)
        elif curvesRight == curv. cr_90:
            dirs.append(dir.S)
        elif curvesRight == curv. cr_45_90:
            dirs.append(dir.SO)
            dirs.append(dir.S)
        elif curvesRight == curv. cr_90_135:
            dirs.append(dir.S)
            dirs.append(dir.SE)
        elif curvesRight == curv. cr_45_135:
            dirs.append(dir.SE)
            dirs.append(dir.SO)
        elif curvesRight == curv. cr_45_90_135:
            dirs.append(dir.SE)
            dirs.append(dir.S)
            dirs.append(dir.SO)

        if '000' not in center:
            dirs.append(dir.L)

        return dirs

    def reset(self):
        self.foundLeft = []
        self.lastLeft = ''

        self.foundRight = []
        self.lastRight = ''

        self.foundCenter = []
        self.lastCenter = ''

    
# Rob
BASE_SPEED = 0.05
MAX_SPEED = 0.15
MIN_SPEED = -0.15
OUTSIDE_LINE = -1 
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, base_speed=BASE_SPEED, max_speed=MAX_SPEED, min_speed=MIN_SPEED):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.baseSpeed = base_speed
        self.maxSpeed = max_speed
        self.minSpeed = min_speed
        self.outside = False
        self.state = "ident"
        # IDENT
        self.identifier = DirectionIdentifier()
        self.dirs = []
        # self.foundLeft = []
        # self.foundRight = []
        # self.foundCenter = []

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

        # make initial sensor read
        self.readSensors()
        # init line sensor filter
        self.lineSensorFilter = LineSensorFilter(FILTER_SIZE, self.measures.lineSensor)
        # init orientation
        self.orientation = int(self.measures.compass)
        # init PID controllers      
        self.controller = PIDController(KP, KI, KD)
        self.compassController = PIDController(CKP, CKI, CKD)
        # init time instant count
        self.ti = 0
        # init GPS
        self.gpsFilter = GPSFilter(self.measures.x,self.measures.y)
        # init next intersection
        self.nextIntersection = (0.0, 0.0)
            

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

    def debug_GO(self):
        coloredLineSensor = bcolors.color(self.lineSensorRead)
        coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)

        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} {:4.2f} error: {:5.2f} p: {:6.2f} i: {:6.2f} d: {:6.2f} PID: {:6.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy, self.dist,
                    self.error, self.controller.p, self.controller.i, self.controller.d, self.controller.pid,
                    self.lPow, self.rPow
                )
        )

    def getNextIntersection(self):
        if self.target == dir.NE:
            self.nextIntersection = (self.gpsFilter.x + 2, self.gpsFilter.y + 2)
        elif self.target == dir.N:
            self.nextIntersection = (self.gpsFilter.x, self.gpsFilter.y)
        elif self.target == dir.NO:
            self.nextIntersection = (self.gpsFilter.x - 2, self.gpsFilter.y + 2)
        elif self.target == dir.SE:
            self.nextIntersection = (self.gpsFilter.x + 2, self.gpsFilter.y - 2)
        elif self.target == dir.S:
            self.nextIntersection = (self.gpsFilter.x, self.gpsFilter.y - 2)
        elif self.target == dir.SO:
            self.nextIntersection = (self.gpsFilter.x-2, self.gpsFilter.y - 2)
        elif self.target == dir.L:
            self.nextIntersection = (self.gpsFilter.x + 2, self.gpsFilter.y)
        elif self.target == dir.O:
            self.nextIntersection = (self.gpsFilter.x - 2, self.gpsFilter.y)

    def determinePosition(self):
        ones = self.lineSensorFilteredReadNum[2:5].count(1)
        l2, l1, l0, c, r0, r1, r2 = self.lineSensorFilteredReadNum
        if ones:    # inside the line
            self.pos = ((l0*0) + (c*5) + (r0*10)) / ones
        else:       # outside the line
            self.pos = -1

    def calcError(self):
        if self.pos == OUTSIDE_LINE:
            if not self.outside:
                self.outside = True
                if self.controller.lastError < 0:
                    self.error = 5
                else:
                    self.error = -5
        else:
            if self.outside:
                self.outside = False
            self.error = 5 - self.pos       
        
    def setState(self):
        if self.state == "go":
            # if  self.dist >= 1.2 or self.dist < 0:
            if self.dist <= 0.8:
                self.identifier.reset()
                self.state = "ident"
        elif self.state == "ident":
            # if 0 <= self.dist < 1.2:
            if self.dist == 0:
                relDirs = self.identifier.getDirs()
                self.dirs = [self.orientation + d for d in relDirs]

                # decide
                self.target = self.dirs[0]

                # get next intersection
                self.getNextIntersection()

                # change state
                self.state = "go"

                print("Found Left: ", self.identifier.foundLeft)
                print("Found Right: ", self.identifier.foundRight)
                print("Found Center: ", self.identifier.foundCenter)
                print("Dirs: ", self.dirs)
                print("Next: ", self.nextIntersection)
        elif self.state == "turn":
            if int(self.measures.compass) == self.target:
                self.orientation = self.target
                self.state = "go"


    def drive(self):
        # Get the line sensor read
        self.lineSensorRead = self.measures.lineSensor
        # Send it to the filter
        self.lineSensorFilter.update(self.lineSensorRead)
        # Get the filtered line sensor read
        self.lineSensorFilteredRead = self.lineSensorFilter.read()  # as str
        self.lineSensorFilteredReadNum = list(map(int, self.lineSensorFilteredRead))   # as int
        # Update gps filter
        self.gpsFilter.update(self.measures.x,self.measures.y)
        # # Calc displacement from the last intersection: (Dx, Dy)
        # self.Dx = round(abs(self.gpsFilter.x) % 2, 1)
        # self.Dy = round(abs(self.gpsFilter.y) % 2, 1)
        # self.dist = round(sqrt(pow(self.Dx, 2) + pow(self.Dy, 2)),2)
        nextX, nextY = self.nextIntersection
        self.Dx = nextX - self.gpsFilter.x
        self.Dy = nextY - self.gpsFilter.y
        self.dist = round(sqrt(pow(self.Dx, 2) + pow(self.Dy, 2)),2)

        self.setState()

        if self.state == "go":
            self.go()
        elif self.state == "ident":
            self.ident()
        elif self.state == "turn":
            self.turn()

    def go(self):
        # Increment the time instant
        self.ti = self.ti+1
        
        # test new position calc
        self.determinePosition()
        self.calcError()

        # Get the PID control
        pid = self.controller.getPID(self.error)
        # Compute the powers of the motors
        self.lPow = round(self.baseSpeed - pid, 2)
        self.rPow = round(self.baseSpeed + pid, 2)
        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)

        # debug
        self.debug_GO()
        
    def ident(self):
        self.identifier.push(self.lineSensorFilteredRead)

        # calc compass position error
        self.identError = self.orientation - self.measures.compass

        # Get the PID control
        pid = self.compassController.getPID(self.identError)
        # Compute the powers of the motors
        self.lPow = round(0.01 - pid, 2)
        self.rPow = round(0.01 + pid, 2)
        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)
        
        # debug
        coloredLineSensor = bcolors.color(self.lineSensorRead)
        coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)
        
        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} {:4.2f} left: {} right: {} angle: {:4.2f} error: {:5.2f} p: {:6.2f} i: {:6.2f} d: {:6.2f} PID: {:6.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy, self.dist,
                    bcolors.color(self.identifier.lastLeft), bcolors.color(self.identifier.lastRight), self.measures.compass,
                    self.identError, self.compassController.p, self.compassController.i, self.compassController.d, self.compassController.pid,
                    self.lPow, self.rPow
                )
        )


    def turn(self):
        # calc compass position error
        self.identError = self.target - self.measures.compass

        # Get the PID control
        pid = self.compassController.getPID(self.identError)
        # Compute the powers of the motors
        self.lPow = round(-pid, 2)
        self.rPow = round(+pid, 2)
        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)

        # debug
        coloredLineSensor = bcolors.color(self.lineSensorRead)
        coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)
        
        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} {:4.2f} angle: {:4.2f} error: {:5.2f} p: {:6.2f} i: {:6.2f} d: {:6.2f} PID: {:6.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy, self.dist, self.measures.compass,
                    self.identError, self.compassController.p, self.compassController.i, self.compassController.d, self.compassController.pid,
                    self.lPow, self.rPow
                )
        )
        
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
