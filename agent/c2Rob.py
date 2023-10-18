
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
KP = 0.0025
KI = 0.0
KD = 0.002
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
        self.x = round(x-self.init_x, 1)
        self.y = round(y - self.init_y, 1)

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
        self.orientation = dir.L
        self.state = "ident"
        # IDENT
        self.dirs = []
        self.foundLeft = False
        self.foundRight = False
        self.LostCenter = False

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
        # init PID controller      
        self.controller = PIDController(KP, KI, KD)
        # init time instant count
        self.ti = 0
        # init GPS

        self.gpsFilter = GPSFilter(self.measures.x,self.measures.y)        

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
        coloredLineSensor = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == '0' else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorRead)
        )
        coloredLineSensorFiltered = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == 0 else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorFilteredRead)
        )

        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} error: {:5.2f} p: {:6.2f} i: {:6.2f} d: {:6.2f} PID: {:6.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy,
                    self.error, self.controller.p, self.controller.i, self.controller.d, self.controller.pid,
                    self.lPow, self.rPow
                )
        )

    def determinePosition(self):
        ones = self.lineSensorFilteredRead.count(1)
        l2, l1, l0, c, r0, r1, r2 = self.lineSensorFilteredRead
        if ones:    # inside the line
            self.pos = ((l2*0) + (l1*15) + (l0*25) + (c*30) + (r0*35) + (r1*45) + (r2*60)) / ones
        else:       # outside the line
            self.pos = -1
        
    def calcError(self):
        if self.pos == OUTSIDE_LINE:
            if not self.outside:
                self.outside = True
                if self.controller.lastError < 0:
                    self.error = 30
                else:
                    self.error = -30
        else:
            if self.outside:
                self.outside = False
            self.error = 30 - self.pos       

    def setState(self):
        if self.state == "go":
            if (self.Dx >= 1.3 and self.Dy == 0) or (self.Dx == 0 and self.Dy >= 1.3):
                self.state = "ident"
                self.dirs = []
                self.foundLeft = 0
                self.foundRight = 0
        elif self.state == "ident":
            if (0 < self.Dx < 1.3 ) or (0 < self.Dy <1.3):
                self.state = "go"
        #         self.state = "turn"
        # elif self.state == "turn":
        #     if (self.Dx > 0.5) or (self.Dy > 0.5):
        #         self.state = "go"
        

    def drive(self):
        # Get the line sensor read
        self.lineSensorRead = self.measures.lineSensor
        # Send it to the filter
        self.lineSensorFilter.update(self.lineSensorRead)
        # Get the filtered line sensor read
        self.lineSensorFilteredRead = list(map(int, self.lineSensorFilter.read()))
        # Update gps filter
        self.gpsFilter.update(self.measures.x,self.measures.y)
        # Calc displacement from the last intersection: (Dx, Dy)
        self.Dx = round(abs(self.gpsFilter.x) % 2, 1)
        self.Dy = round(abs(self.gpsFilter.y) % 2, 1)

        #print(self.gpsFilter.x,self.gpsFilter.y, self.measures.compass)
        
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
        left = self.lineSensorFilteredRead[:2]
        right = self.lineSensorFilteredRead[5:]
        center = self.lineSensorFilteredRead[2:5]


        if left != [0, 0]:
            if not self.foundLeft:
                if left == [0, 1]:
                    self.dirs.append("NE")
                elif left == [1, 0]:
                    self.dirs.append("NO")
                elif left == [1, 1]:
                    self.dirs.append("N")
                self.foundLeft = True

        if right != [0, 0]:
            if not self.foundRight:
                if right == [0, 1]:
                    self.dirs.append("SO")
                    self.foundLeft = 1
                elif right == [1, 0]:
                    self.dirs.append("SE")
                elif right == [1, 1]:
                    self.dirs.append("S")
                self.foundRight = True

        # if 1 in center:
        #     if not self.lostCenter:
        #         self.dirs.remove("?")
        #     self.lostCenter = True
        
        # if self.Dx <= 0:
        #     pass

        # Compute the powers of the motors
        self.lPow = 0.02
        self.rPow = 0.02
        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)
        
        # debug
        coloredLineSensor = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == '0' else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorRead)
        )
        coloredLineSensorFiltered = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == 0 else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorFilteredRead)
        )
        
        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} {} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy,
                    self.dirs,
                    self.lPow, self.rPow
                )
        )

    def turn(self):

        # Compute the powers of the motors
        if self.dirs == ["N"]:
            self.lPow, self.rPow = (-0.05, 0.05)
        elif self.dirs == ["NE"]:
            self.lPow, self.rPow = (0, 0.05)
        elif self.dirs == ["NO"]:
            self.lPow, self.rPow = (-0.05, 0.05)
        elif self.dirs == ["S"]:
            self.lPow, self.rPow = (0.05, -0.05)
        elif self.dirs == ["SE"]:
            self.lPow, self.rPow = (0.05, 0)
        elif self.dirs == ["SO"]:
            self.lPow, self.rPow = (0.05, -0.05)
        else:  
            self.lPow, self.rPow = (0.03, 0.03)

        # Send the drive command
        self.driveMotors(self.lPow, self.rPow)

        # debug
        coloredLineSensor = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == '0' else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorRead)
        )
        coloredLineSensorFiltered = ''.join(map(
            lambda val: bcolors.RED+str(val)+bcolors.ENDC if val == 0 else bcolors.GREEN+str(val)+bcolors.ENDC, self.lineSensorFilteredRead)
        )
        
        print(
            '{} <=> {} {} delta: {:4.2f} {:4.2f} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensor, coloredLineSensorFiltered, self.state.upper(), self.Dx, self.Dy,
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
