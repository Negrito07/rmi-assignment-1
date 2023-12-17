
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
# monitoring
import csv

CELLROWS=7
CELLCOLS=14

# DEBUG
class bcolors:
    RED = '\033[31m'
    GREEN = '\033[32m'
    ENDC = '\033[0m'

    def color(lineSensorSeq):
        return ''.join(map(
            lambda val: bcolors.RED+val+bcolors.ENDC if val == '0' else bcolors.GREEN+val+bcolors.ENDC, lineSensorSeq)
        )
    
class GPSFilter:
    def __init__(self,x,y):
        self.init_x = x
        self.init_y = y
        self.x = self.init_x
        self.y = self.init_y

    def update(self,x,y):
        self.x = x - self.init_x
        self.y = y - self.init_y

# PID
PKP = 0.09
PKI = 0.0
PKD = 0.0

OKP = 0.0015
OKI = 0.0
OKD = 0.0005

CKP = 0.0
CKI = 0.0
CKD = 0.0

LKP = 0.02
LKI = 0.0
LKD = 0.01
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
        self.accumError = 0

    def getPID(self, error):
        self.p = self.kp * (error)
        self.i = self.ki * (self.accumError + error)
        self.d = self.kd * (error - self.lastError)

        self.lastError = error
        self.accumError = self.accumError + error

        self.pid = self.p + self.i + self.d
        return self.pid
    
    def reset(self, startError=0):
        self.lastError = startError
        self.accumError = startError



# Orientation
class dir:
    directions = [0, 45, 90, 135, -180, -135, -90, -45]
    L, NE, N, NO, O, SO, S, SE = [int(angle) // 45 for angle in directions]
    chars = ['-', '/', '|', '\\']

    def fromRelative(orientation, relative):
        base = (orientation + relative) % len(dir.directions)
        return dir.directions[base]
    
    def fromGlobal(orientation, glob):
        base = (glob - orientation) % len(dir.directions)
        return dir.directions[base]
    
    def fromAngle(angle):
        direction = floor((angle+22.5)/45)
        return direction


class CompassFilter:
    def __init__(self, compass, margin=3, range=5):
        print(compass, dir.fromAngle(compass))
        self.base = dir.directions[dir.fromAngle(compass)]
        self.margin = margin
        self.range = range
        self.ang = self.base
        self.sum = self.ang
        self.average = self.ang
        self.n = 1
        # print("Compass filter initialized with base: ", self.base)

    def update(self, compass):
        direction = compass - self.base
        filtered = self.margin * floor((direction + (self.margin/2)) / self.margin)
    
        # # blocks with size n avg
        # self.n = (self.n % self.range) + 1
        # if(self.n == 1):
        #     self.sum = direction
        #     average = self.sum
        #     print("reset: ", self.sum)
        # else:
        #     self.sum = self.sum + direction
        #     average = floor((self.sum + 0.5)/self.n)

        # cumulative avg - filtering result
        self.average = (self.average + abs(direction)) / 2
        self.ang = self.margin * floor((self.average + (self.margin/2)) / self.margin)
        if direction < 0:
            self.ang = -self.ang
        # print("compass: ", compass, "avg: ", self.average, "ang: ", self.ang)

        # cumulative avg - filtering samples
        # self.ang = (self.ang + filtered) / 2
        # print("compass: ", compass, "filtered: ", filtered, "ang: ", self.ang)


# Identifier
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
        
        dirRight.extend(list(map(lambda sc: ''.join(sc.seq), self.foundRight)))
        dirLeft.extend(list(map(lambda sc: ''.join(sc.seq), self.foundLeft)))
        center.extend(list(map(lambda sc: ''.join(sc.seq), self.foundCenter)))
        # print(f'dirRight: {dirRight}')
        # print(f'dirLeft: {dirLeft}')
        
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
        
            if dirRight[-1] == '01' and dirRight[0] == '01':
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

            if dirLeft[-1] == '10' and dirLeft[0] == '10':
               if dirLeft[len(dirLeft)//2] == '11':
                    dirs.add(dir.N) 

        if '000' not in center:
            dirs.add(dir.L)

        return list(dirs)

    def reset(self):
        self.foundLeft = []
        self.lastLeft = ''

        self.foundRight = []
        self.lastRight = ''

        self.foundCenter = []
        self.lastCenter = ''


# Map
class Path:
    def __init__(self, dir):
        self.dir = dir
        self.visited = False

    def __repr__(self):
        return '({}, {})'.format(self.dir, self.visited)
    
    def __eq__(self, other):
        if isinstance(other, Path):
            return self.dir == other.dir
    

class Intersection:
    def __init__(self, pos, dirs):
        self.pos = pos
        self.paths = [Path(dir) for dir in dirs]
        self.visited = True
    
    def __repr__(self):
        return 'I('+repr(self.paths)+')'
    
    def __eq__(self, other):
        if isinstance(other, Intersection):
            return self.pos == other.pos
    

# Search
# Dominios de pesquisa
# Permitem calcular
# as accoes possiveis em cada estado, etc
class SearchDomain:
    def __init__(self, map):
        self.map = map

    # lista de acoes possiveis num estado
    def moves(self, state):
        inter = self.map[state]
        return [path.dir for path in inter.paths]

    # resultado de uma acao num estado, ou seja, o estado seguinte
    def result(self, state, action):
        x, y = state
        if action == dir.NE:
           return (x + 2, y + 2)
        elif action == dir.N:
            return (x, y + 2)
        elif action == dir.NO:
            return (x - 2, y + 2)
        elif action == dir.SE:
            return  (x + 2, y - 2)
        elif action  == dir.S:
            return (x, y - 2)
        elif action == dir.SO:
            return (x - 2, y - 2)
        elif action == dir.L:
            return  (x + 2, y)
        elif action == dir.O or action == - dir.O:
            return (x - 2, y)

    # custo de uma acao num estado
    def cost(self, state, action):
        if action == dir.N or action  == dir.S or action == dir.L or action == dir.O or action == - dir.O:
            return 2
        else:
            return hypot(2, 2)
        
    # custo estimado de chegar de um estado a outro
    def heuristic(self, state, goal):
        x, y = state
        x0, y0 = goal
        return hypot(x-x0, y-y0)

    # test if the given "goal" is satisfied in "state"
    def satisfies(self, state, goal):
        state == goal

# Problemas concretos a resolver
# dentro de um determinado dominio
class SearchProblem:
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal
    def goal_test(self, state):
        return self.domain.satisfies(state,self.goal)
    
# Nos de uma arvore de pesquisa
class SearchNode:
    def __init__(self, state, parent, depth, cost, heuristic, action=None): 
        self.state = state
        self.parent = parent
        self.depth = depth
        self.action = action      # guarda a acao que levou do estado anterior a este estado
        self.cost = cost          # guarda o custo da acao anterior
        self.heuristic = heuristic
        
    def __str__(self):
        return "(" + str(self.state) + "," + str(self.cost) + "," + str(self.heuristic) + ")"
    def __repr__(self):
        return str(self)
    # so queremos verificar se o estado é igual
    def __eq__(self, other):
        if isinstance(other, SearchNode):
            return self.state == other.state

    def in_parent(self, newstate):
        if self.parent == None:
            return False
        if self.parent.state == newstate:
            return True
        return self.parent.in_parent(newstate)
    
# Arvore de pesquisa
class SearchTree:
    # construtor
    def __init__(self, problem): 
        self.problem = problem
        root = SearchNode(problem.initial, parent=None, depth=0, cost=0, heuristic=problem.domain.heuristic(problem.initial, problem.goal))
        self.open_nodes = [root]
        self.closed_nodes = []
        self.solution = None
        self.terminals = 0
        self.non_terminals = 0

    @property 
    def length(self):
        if self.solution == None:
            return None
        return self.solution.depth

    @property
    def avg_branching(self):
        return ((self.terminals+self.non_terminals)-1)/self.non_terminals
    
    # 8) changes: @property returns the solution cost of a tree search
    @property
    def cost(self):
        if not self.solution:
            return None
        return self.solution.cost
    # end of 8)

    # obter o caminho (sequencia de estados) da raiz ate um no
    def get_path(self,node):
        if node.parent == None:
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return(path)

    # procurar a solucao
    def search(self,limit = None):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            self.closed_nodes.append(node)
            #print(node)          
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes) + 1
                return self.get_path(node)
            self.non_terminals += 1 
            lnewnodes = []
            union = self.open_nodes + self.closed_nodes
            for a in self.problem.domain.moves(node.state):
                newstate = self.problem.domain.result(node.state,a)
                # 8) changes:
                # >> reg action cost from node.state to newstate
                newstate_cost = self.problem.domain.cost(node.state,node.action,a)
                # end of 8)
                # 13) changes to support greedy search
                #  >> calc estimate cost from newstate to goal (heuristic)
                newstate_heur = self.problem.domain.heuristic(newstate,self.problem.goal)
                # end of 13)
                if not node.in_parent(newstate):
                    newnode = SearchNode(newstate, node, node.depth + 1, node.cost + newstate_cost, newstate_heur, a)
                    if limit == None or newnode.depth <= limit:
                        if newnode in union:
                            oldnode = union[union.index(newnode)]
                            if newnode.cost < oldnode.cost:
                                lnewnodes.append(newnode)
                        else:
                            lnewnodes.append(newnode)

            self.add_to_open(lnewnodes)
            
        return None

# juntar novos nos a lista de nos abertos de acordo com a estrategia A* search
    def add_to_open(self,lnewnodes):
        self.open_nodes.extend(lnewnodes)
        self.open_nodes = sorted(self.open_nodes, key=lambda node: node.heuristic + node.cost)

    
# Rob
MAX_SPEED = 0.15
MIN_SPEED = -0.15
MAP_ROWS = 21
MAP_COLS = 49 
MAP_CENTER = (MAP_ROWS//2 + 1, MAP_COLS//2 + 1)
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, max_speed=MAX_SPEED, min_speed=MIN_SPEED):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.maxSpeed = max_speed
        self.minSpeed = min_speed
        self.outside = False
        # states
        self.state = "init"
        self.transition = 0
        # IDENT
        self.identifier = DirectionIdentifier()
        self.dirs = []
        # map
        self.map = {}
        self.openNodes = []
        self.filename = 'mapping.out'
        self.charMap = [[' ' for cols in range(MAP_COLS)] for rows in range(MAP_ROWS)]
        self.targets = []
        # motors
        self.lPow = 0
        self.rPow = 0
        # position
        self.x = 0
        self.y = 0
        self.roundX = 0
        self.roundY = 0
        self.Dx = 0
        self.Dy = 0
        self.lastDx = 0
        self.lastDy = 0
        self.ang = 0
        # init target
        self.target = (self.x, self.y)
        self.source = self.target
        self.dtarget = dist(self.target, self.source)
        robot = (self.roundX, self.roundY)
        self.drobot = dist(robot, self.source)


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
        # init orientation
        self.orientation = dir.fromAngle(self.ang)
        self.turn = self.orientation
        self.filteredCompass = CompassFilter(self.measures.compass)
        # init PID controllers      
        self.positionController = PIDController(PKP, PKI, PKD)
        self.orientationController = PIDController(OKP, OKI, OKD)
        self.compassController = PIDController(CKP, CKI, CKD)
        self.lineController = PIDController(LKP, LKI, LKD)
        # update gps
        self.gps = GPSFilter(self.measures.x, self.measures.y)    # DEBUG only
        self.gps.update(self.measures.x, self.measures.y)         # DEBUG only
        # init targets array
        self.targets = [None for i in range(int(self.nBeacons))]

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

    
    def driveMotorsExt(self, lPow, rPow):
        # apply movement model
        outL = (lPow + self.lPow) / 2
        outR = (rPow + self.rPow) / 2

        lin = (outR + outL) / 2

        theta = self.ang*pi / 180 

        self.x = self.x + lin*cos(theta)
        self.y = self.y + lin*sin(theta)
        self.roundX = round(self.x, 1)
        self.roundY = round(self.y, 1)

        rot = (outL - outR)*180 / pi
        self.ang = self.ang - rot
        # send driveMotors
        self.driveMotors(lPow, rPow)

    def getTarget(self, orientation):
        if orientation == dir.NE:
           return (self.roundX + 2, self.roundY + 2)
        elif orientation == dir.N:
            return (self.roundX, self.roundY + 2)
        elif orientation == dir.NO:
            return (self.roundX - 2, self.roundY + 2)
        elif orientation == dir.SE:
            return  (self.roundX + 2, self.roundY - 2)
        elif orientation  == dir.S:
            return (self.roundX, self.roundY - 2)
        elif orientation == dir.SO:
            return (self.roundX-2, self.roundY - 2)
        elif orientation == dir.L:
            return  (self.roundX + 2, self.roundY)
        elif orientation == dir.O or orientation == - dir.O:
            return (self.roundX - 2, self.roundY) 
        
    
    def setCharMap(self, inter, dirs):
        # convert intersection xy coord to ij coord
        i, j = MAP_CENTER
        dx, dy = inter
        irow, icol = (i-dy, j+dx)

        # get char position and value for every possible path
        for ang in dirs:
            d = dir.fromAngle(ang)
            
            row, col = irow, icol
            if d == dir.NE:
                row, col = irow-1, icol+1
            elif d == dir.N:
                row = irow-1
            elif d == dir.NO:
                row, col = irow-1, icol-1
            elif d == dir.SE:
                row, col = irow+1, icol+1
            elif d  == dir.S:
                row = irow+1
            elif d == dir.SO:
                row, col = irow+1, icol-1
            elif d == dir.L:
                col = icol+1
            elif d == dir.O:
                col = icol-1
            
            # set char map
            self.charMap[row][col] = dir.chars[d]

    
    def writeCharMap(self):
        with open(self.filename, 'w') as mapFile:
            for row in self.charMap:
                mapFile.write(''.join(row)+'\n')

        
    def setState(self):
        if self.state == "init":
            # debug
            # print("Found Left: ", self.identifier.foundLeft)
            # print("Found Right: ", self.identifier.foundRight)
            # print("Found Center: ", self.identifier.foundCenter)

            # init turn variables
            # identify possible turns
            self.relDirs = self.identifier.getDirs()
            # get global dirs for the map
            self.globalDirs = [dir.fromRelative(self.orientation, relDir) for relDir in self.relDirs]
            
            # print("Orientation: ", dir.directions[self.orientation])
            # print("Dirs: ", list(map(lambda rel: dir.directions[rel], self.relDirs)))

            # save initial intersection to the map
            inter = (int(self.x), int(self.y))
            self.map[inter] = Intersection(inter, self.globalDirs)
            # save initial target
            self.targets[self.measures.ground] = self.map[inter]

            # init char map
            i, j = MAP_CENTER
            dx, dy = inter
            row, col = (i-dy, j+dx)
            self.charMap[row][col] = 'I'
            # update char map
            self.setCharMap(inter, self.globalDirs)
            # update map file
            self.writeCharMap()

            # expand intersection node
            for path in self.map[inter].paths:
                node = (inter, path)
                self.openNodes.append(node)
            
            # print("Open Nodes:", self.openNodes)

            # decide which turn to take
            inter, path = self.openNodes.pop(0)
            self.turn = dir.fromAngle(dir.fromGlobal(self.orientation, dir.fromAngle(path.dir)))

            # print("Turn: ", dir.directions[self.turn])
            # print("Map", self.map)

            # change to next state
            self.state = "turn"
            self.transition = 0
            self.first = True
        elif self.state == "scan":
            # count transition condition
            if self.distError <= 0:
                self.transition = self.transition + 1
            else:
                self.transition = 0
            
            # let transition condition stabilize
            stable = 1
            if self.transition == stable:
                # change to next state
                self.state = "ident"
                self.transition = 0
                self.first = True
        elif self.state == "ident":
            # count transition condition
            if self.lineSensorFilteredRead[:2] == ['0', '0'] and self.lineSensorFilteredRead[5:] == ['0', '0']:
                    self.transition = self.transition + 1
            else:
                self.transition = 0
            
            # let transition condition stabilize
            stable = 3
            if self.transition == stable:
                # debug
                # print("Found Left: ", self.identifier.foundLeft)
                # print("Found Right: ", self.identifier.foundRight)
                # print("Found Center: ", self.identifier.foundCenter)
                # correct intersection coordinates
                x, y = self.target
                self.x = x
                self.y = y
                self.roundX = round(x, 1)
                self.roundY = round(y, 1)

                # init turn variables
                # identify possible turns
                self.relDirs = self.identifier.getDirs()
                # back is always possible
                self.relDirs.append(dir.O)
                # get global dirs for the map
                self.globalDirs = [dir.fromRelative(self.orientation, relDir) for relDir in self.relDirs]
                # update char map
                inter = (int(self.roundX), int(self.roundY))
                self.setCharMap(inter, self.globalDirs[:-1])
                self.writeCharMap()

                # save new intersection to the search map
                if not (inter in self.map):
                    self.map[inter] = Intersection(inter, self.globalDirs)
                    # check if intersection is a beacon
                    if self.measures.ground != -1:
                        print("Target", self.measures.ground, "at", inter)
                        self.targets[self.measures.ground] = self.map[inter]

                    # expand nodes
                    newNodes = []
                    for path in self.map[inter].paths:
                        node = (inter, path)
                        target = self.getTarget(dir.fromAngle(path.dir))
                        print(target, path.dir)
                        # only consider the return path and path to unvisited intersection
                        if dir.fromGlobal(self.orientation, dir.fromAngle(path.dir)) == dir.directions[dir.O] or not (target in self.map):
                            newNodes.append(node)
                        else:
                            # found a visited intersection from another path, remove itself from openNodes
                            remove = None
                            i = 0
                            for tInter, tPath in self.openNodes:
                                if tInter == target and dir.fromRelative(dir.fromAngle(tPath.dir), dir.O) == path.dir:
                                    remove = i
                                i = i+1
                            if remove:
                                self.openNodes.pop(remove)

                    self.openNodes[:0] = newNodes
            

                # decide which turn to take
                inter, path = self.openNodes.pop(0)
                self.turn = dir.fromAngle(dir.fromGlobal(self.orientation, dir.fromAngle(path.dir)))

                print("Open Nodes:", self.openNodes)
                # print("Orientation: ", dir.directions[self.orientation])
                # print("Dirs: ", list(map(lambda rel: dir.directions[rel], self.relDirs)))
                # print("Node: ", (inter, path))
                # print("Turn: ", dir.directions[self.turn])
                
                # show map
                # print(self.map)
                for row in self.charMap:
                    print(''.join(row))

                # change to next state
                self.state = "turn"
                self.transition = 0
                self.first = True
        elif self.state == "turn":
            if self.first:
                # reset compass controller
                self.compassController.reset(self.angError)
                # reset flag
                self.first = False

            # count transition condition
            if self.angError == 0:
                self.transition = self.transition + 1
            else:
                self.transition = 0
            
            # let transition condition stabilize
            stable = 2
            if self.transition == stable:
                # init go variables
                ang = round(self.ang)
                self.orientation = dir.fromAngle(ang)
                self.turn = 0
                
                self.target = self.getTarget(self.orientation)
                self.source = (self.roundX, self.roundY)
                self.dtarget = dist(self.target, self.source)

                # print("Next Target: ", self.target)

                # change to next state
                self.state = "go"
                self.transition = 0
                self.first = True
        elif self.state == "go":
            if self.first:
                # reset gps controller
                self.positionController.reset(self.posError)
                # reset flag
                self.first = False

            if self.posError <= 0.85:
                # init ident variables
                self.identifier.reset()
                # change to scan state
                self.state = "scan"


    def drive(self):
        # sense
        # Get the filtered line sensor read
        self.lineSensorFilteredRead = self.measures.lineSensor  # as str
        self.lineSensorFilteredReadNum = list(map(int, self.lineSensorFilteredRead))   # as int
        # TODO: compass filter
        # Get the filtered compass read
        self.filteredCompass.update(self.measures.compass)
        self.ang = self.filteredCompass.ang
        # update gps
        self.gps.update(self.measures.x, self.measures.y)     # DEBUG only


        # Calc distance to target
        nextX, nextY = self.target
        # print(nextX, nextY, self.roundX, self.roundY)
        # self.dist = round(sqrt(pow(self.Dx, 2) + pow(self.Dy, 2)), 2)
        # self.posError = self.dist
        # update dx, dy
        self.Dx = nextX - self.roundX
        self.Dy = nextY - self.roundY
        robot = (self.roundX, self.roundY)
        self.drobot = dist(robot, self.source)
        self.distError = self.dtarget - self.drobot

        # calc distance error
        absDx, absDy = abs(self.Dx), abs(self.Dy)
        if self.Dx != 0 and self.Dy != 0:
            self.posError = (absDx + absDy) / 2
        else:
            self.posError = absDx + absDy

        
        # calc orientation error
        ang = round(self.ang)
        dAng = dir.directions[self.turn] - (ang - dir.directions[self.orientation])
        # dRad = dAng * pi / 180
        # self.angError = abs(cos(dRad) - 1) * 50
        self.angError = abs(dAng)
        if self.angError > 180:
            self.angError = 360 - self.angError
            dAng = - dAng
        # calc center line error
        center = self.lineSensorFilteredReadNum[2:5]
        ones = center.count(1)
        l0, c, r0 = center
        if ones:
            linePos = ((l0*0) + (c*1) + (r0*2)) / ones
        else:
            linePos = 1
        self.lineError = 1 - linePos
        
        # Get the PID control
        pidSpeed = self.positionController.getPID(self.posError)
        pidOrient = self.orientationController.getPID(self.angError)
        # pidCompass = pidSpeed * self.compassController.getPID(self.angError)
        pidLine = self.lineController.getPID(self.lineError)

        if dAng < 0:
            pidOrient = - pidOrient

        if self.state=="init" or self.state == "scan" or self.state == "ident":
            self.identifier.push(self.lineSensorFilteredRead)
        if self.state == "ident":
            baseSpeed = 0.01
            # Compute the powers of the motors
            lPow = round((baseSpeed + pidSpeed) - pidOrient, 2)
            rPow = round((baseSpeed + pidSpeed) + pidOrient, 2)
        elif self.state == "go":
            # Compute the powers of the motors
            lPow = round(pidSpeed - pidLine, 2)
            rPow = round(pidSpeed + pidLine, 2)
        else:
            # Compute the powers of the motors
            lPow = round(pidSpeed - pidOrient, 2)
            rPow = round(pidSpeed + pidOrient, 2)

        # normalize powers
        if lPow > 0.15:
            lPow = 0.15
        elif lPow < -0.15:
            lPow = -0.15
        if rPow > 0.15:
            rPow = 0.15
        elif rPow < -0.15:
            rPow = -0.15

        # keep last motor powers
        self.lPow = lPow
        self.rPow = rPow

        # debug
        # coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)
        # print(
        #     '{} {:4s} lineError: {:4.2f} coord: {:4.2f} {:4.2f} gps: {:4.2f} {:4.2f} target: {:4.2f} {:4.2f} dX: {:4.2f} dY: {:4.2f} posError: {:5.2f} distError: {:5.2f} orient: {:3d} turn: {:3d} ang: {:3d} compass: {:4.2f} angError: {:4d} motors: {:5.2f} {:5.2f}'
        #         .format(
        #             coloredLineSensorFiltered, self.state.upper(), self.lineError,
        #             self.roundX, self.roundY, self.gps.x, self.gps.y, self.target[0], self.target[1],
        #             self.Dx, self.Dy, self.posError, self.distError,
        #             dir.directions[self.orientation], dir.directions[self.turn], ang, self.measures.compass, self.angError,
        #             self.lPow, self.rPow
        #         )
        # )
        coloredLineSensorFiltered = bcolors.color(self.lineSensorFilteredRead)
        print(
            '{} {:4s} lineError: {:4.2f} orient: {:3d} turn: {:3d} ang: {:3d} compass: {:4.2f} angError: {:4d} motors: {:5.2f} {:5.2f}'
                .format(
                    coloredLineSensorFiltered, self.state.upper(), self.lineError,
                    dir.directions[self.orientation], dir.directions[self.turn], ang, self.filteredCompass.ang, self.angError,
                    self.lPow, self.rPow
                )
        )

        # Change state
        self.setState()
        # Send drive command
        self.driveMotorsExt(lPow, rPow)
        

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
