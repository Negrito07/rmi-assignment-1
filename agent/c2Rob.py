
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
GKP = 0.035
GKI = 0.015
GKD = 0.0

CKP = 0.005
CKI = 0.0
CKD = 0.0
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
    directions = [0, 45, 90, 135, -180, -135, -90, -45]
    L, NE, N, NO, O, SO, S, SE = [int(angle) // 45 for angle in directions]

    def fromRelative(orientation, relative):
        base = (orientation + relative) % len(dir.directions)
        return dir.directions[base]
    
    def fromAngle(angle):
        direction = int(angle) // 45
        return direction


class curv:
    cl_135 = '0001111000'
    cr_135 = '0010110100'

    cl_45 = cr_135
    cr_45 = cl_135

    cl_90 = '001100'
    cr_90 = cl_90

    cl_90_135 = '00111000'
    cr_90_135 = '00110100'

    cl_45_90 = '00101100'
    cr_45_90 = '00011100'

    cl_45_135 = '001011010001111000'
    cr_45_135 = '00011110110100'

    cl_45_90_135 = '0010111000'
    cr_45_90_135 = '0001110100'


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
        
        if left != ['0','0']:
            if left == self.lastLeft:
                self.foundLeft[-1].count()
            else:
                self.lastLeft = left
                self.foundLeft.append(SequenceCounter(left))

        if right != ['0','0']:
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
        dirRight = []
        dirLeft = []
        center = []
        dirs = set()

        
        dirRight.extend(list(map(lambda sc: ''.join(sc.seq),self.foundRight)))
        dirLeft.extend(list(map(lambda sc: ''.join(sc.seq),self.foundLeft)))
        center.extend(list(map(lambda sc: ''.join(sc.seq),self.foundCenter)))
        print(f'dirRight: {dirRight}')
        print(f'dirLeft: {dirLeft}')

        if dirRight:
            if dirRight[0] == '01':
                dirs.add(dir.SO)
            elif dirRight[0] == '11':
                dirs.add(dir.S)
            elif dirRight[0] == '10':
                dirs.add(dir.SE)

            if dirRight[-1] == '01':
                dirs.add(dir.SE)
            elif dirRight[-1] == '11':
                dirs.add(dir.S)
            elif dirRight[-1] == '10':
                dirs.add(dir.SO)
        
            if dirRight[-1] and dirRight[0] == '01':
                if dirRight[len(dirRight)//2] == '11':
                    dirs.add(dir.S)
        if dirLeft:
            if dirLeft[0] == '01':
                dirs.add(dir.NE)
            elif dirLeft[0] == '11':
                dirs.add(dir.N)
            elif dirLeft[0] == '10':
                dirs.add(dir.NO)

            if dirLeft[-1] == '01':
                dirs.add(dir.NO)
            elif dirLeft[-1] == '11':
                dirs.add(dir.N)
            elif dirLeft[-1] == '10':
                dirs.add(dir.NE)

            if dirLeft[-1] and dirLeft[0] == '10':
               if dirLeft[len(dirLeft)//2] == '11':
                    dirs.add(dir.N) 

        
        if '000' not in center:
            dirs.add(dir.L)
        else: 
            dirs.add(dir.O)

        # curvesLeft = ''.join(map(lambda sc: ''.join(sc.seq), self.foundLeft))
        # curvesRight = ''.join(map(lambda sc: ''.join(sc.seq), self.foundRight))
        # center = ''.join(map(lambda sc: ''.join(sc.seq), self.foundCenter))

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
        
        # if curvesLeft == curv.cl_135:
        #     dirs.append(dir.NE)
        # elif curvesLeft == curv.cl_45:
        #     dirs.append(dir.NO)
        # elif curvesLeft == curv.cl_90:
        #     dirs.append(dir.N)
        # elif curvesLeft == curv.cl_45_90:
        #     dirs.append(dir.N)
        #     dirs.append(dir.NO)
        # elif curvesLeft == curv.cl_90_135:
        #     dirs.append(dir.N)
        #     dirs.append(dir.NE)
        # elif curvesLeft == curv.cl_45_135:
        #     dirs.append(dir.NE)
        #     dirs.append(dir.NO)
        # elif curvesLeft == curv.cl_45_90_135:
        #     dirs.append(dir.NE)
        #     dirs.append(dir.NO)
        #     dirs.append(dir.N)

        # if curvesRight == curv.cr_135:
        #     dirs.append(dir.SE)
        # elif curvesRight == curv. cr_45:
        #     dirs.append(dir.SO)
        # elif curvesRight == curv. cr_90:
        #     dirs.append(dir.S)
        # elif curvesRight == curv. cr_45_90:
        #     dirs.append(dir.SO)
        #     dirs.append(dir.S)
        # elif curvesRight == curv. cr_90_135:
        #     dirs.append(dir.S)
        #     dirs.append(dir.SE)
        # elif curvesRight == curv. cr_45_135:
        #     dirs.append(dir.SE)
        #     dirs.append(dir.SO)
        # elif curvesRight == curv. cr_45_90_135:
        #     dirs.append(dir.SE)
        #     dirs.append(dir.S)
        #     dirs.append(dir.SO)

        # if '000' not in center:
        #     dirs.append(dir.L)
        
        return list(dirs)

    def reset(self):
        self.foundLeft = []
        self.lastLeft = ''

        self.foundRight = []
        self.lastRight = ''

        self.foundCenter = []
        self.lastCenter = ''

    
# Rob
MAX_SPEED = 0.15
MIN_SPEED = -0.15
OUTSIDE_LINE = -1 
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, max_speed=MAX_SPEED, min_speed=MIN_SPEED):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.maxSpeed = max_speed
        self.minSpeed = min_speed
        self.outside = False
        # states
        self.state = "ident"
        self.transition = 0
        # IDENT
        self.identifier = DirectionIdentifier()
        self.dirs = []

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
        # init GPS
        self.gpsFilter = GPSFilter(self.measures.x,self.measures.y)
        self.gpsFilter.update(self.measures.x,self.measures.y)
        # init target
        self.target = (self.gpsFilter.x, self.gpsFilter.y)
        # init orientation
        self.orientation = dir.fromAngle(self.measures.compass)
        # init PID controllers      
        self.gpsController = PIDController(GKP, GKI, GKD)
        self.compassController = PIDController(CKP, CKI, CKD)
            

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

    def getTarget(self):
        if self.orientation == dir.NE:
            self.target = (self.gpsFilter.x + 2, self.gpsFilter.y + 2)
        elif self.orientation == dir.N:
            self.target = (self.gpsFilter.x, self.gpsFilter.y + 2)
        elif self.orientation == dir.NO:
            self.target = (self.gpsFilter.x - 2, self.gpsFilter.y + 2)
        elif self.orientation == dir.SE:
            self.target = (self.gpsFilter.x + 2, self.gpsFilter.y - 2)
        elif self.orientation  == dir.S:
            self.target = (self.gpsFilter.x, self.gpsFilter.y - 2)
        elif self.orientation == dir.SO:
            self.target = (self.gpsFilter.x-2, self.gpsFilter.y - 2)
        elif self.orientation == dir.L:
            self.target = (self.gpsFilter.x + 2, self.gpsFilter.y)
        elif self.orientation == dir.O:
            self.target = (self.gpsFilter.x - 2, self.gpsFilter.y)      
        
    def setState(self):
        if self.state == "ident":
            # count transition condition
            if self.dist == 0:
                self.transition = self.transition + 1
            else:
                self.transition = 0
            
            # let transition condition stabilize
            stable = 10
            if self.transition == stable:
                # debug
                print("Found Left: ", self.identifier.foundLeft)
                print("Found Right: ", self.identifier.foundRight)
                print("Found Center: ", self.identifier.foundCenter)

                # init turn variables
                # identify possible turns
                relDirs = self.identifier.getDirs()
                self.dirs = [dir.fromRelative(self.orientation, relDir) for relDir in relDirs]

                # decide which turn to take
                self.orientation = dir.fromAngle(self.dirs[0])

                print("Dirs: ", self.dirs)
                print("Turn: ", dir.directions[self.orientation])
                
                # change to next state
                self.state = "turn"
                self.transition = 0
        elif self.state == "turn":
            # count transition condition
            if self.measures.compass == dir.directions[self.orientation]:
                self.transition = self.transition + 1
            else:
                self.transition = 0
            
            # let transition condition stabilize
            stable = 5
            if self.transition == stable:
                # init go variables
                self.getTarget()

                print("Next Target: ", self.target)

                # change to next state
                self.state = "go"
                self.transition = 0
        elif self.state == "go":
            if self.dist <= 0.85:
                # init ident variables
                self.identifier.reset()
                # change to ident state
                self.state = "ident"


    def drive(self):
        # sense
        # Get the line sensor read
        self.lineSensorRead = self.measures.lineSensor
        # Send it to the filter
        self.lineSensorFilter.update(self.lineSensorRead)
        # Get the filtered line sensor read
        self.lineSensorFilteredRead = self.lineSensorFilter.read()  # as str
        self.lineSensorFilteredReadNum = list(map(int, self.lineSensorFilteredRead))   # as int
        # Update gps filter
        self.gpsFilter.update(self.measures.x,self.measures.y)
        # Calc distance to target
        nextX, nextY = self.target
        self.Dx = nextX - self.gpsFilter.x
        self.Dy = nextY - self.gpsFilter.y
        self.dist = round(sqrt(pow(self.Dx, 2) + pow(self.Dy, 2)),2)

        
        # drive
        # calc distance error
        self.posError = self.dist
        # calc orientation error
        self.angError = dir.directions[self.orientation] - self.measures.compass

        # Get the PID control
        pidSpeed = self.gpsController.getPID(self.posError)
        pidTurn = self.compassController.getPID(self.angError)
        # Compute the powers of the motors
        self.lPow = round(pidSpeed - pidTurn, 2)
        self.rPow = round(pidSpeed + pidTurn, 2)
        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)

        # debug
        coloredLineSensor = bcolors.color(self.lineSensorRead)
        coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)

        print(
            '{} <=> {} {:4s} coord: {:4.2f} {:4.2f} dist: {:4.2f} error: {:5.2f} p: {:5.2f} i: {:5.2f} angle: {:4.2f} error: {:5.2f} p: {:5.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(),
                    self.gpsFilter.x, self.gpsFilter.y, self.dist, self.posError, self.gpsController.p, self.gpsController.i,
                    self.measures.compass, self.angError, self.compassController.p,
                    self.lPow, self.rPow
                )
        )
        
        # process state
        if self.state == "ident":
            self.identifier.push(self.lineSensorFilteredRead)
        self.setState()

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
