#! /usr/bin/env python
import math
import numpy


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Cell:
    '''
    Generic class for cell that can be consumed by the actual A-star algorithm
    '''
    def __init__(self, row, col, obs=False):
        '''
        i - denotes the row of the cell
        j - denotes the column of the cell
        obs - denotes whether the cell has an obstacle or not based on map
        '''
        self.col = col
        self.row = row
        self.obstacle = obs
        self.f = 0
        self.g = 0
        self.h = 0
        self.neighbors = set()
        self.previous = None

    def __hash__(self):
        '''
        Override the hash based on obstacle status, so that openSet.pop()
        is always unique but deterministic
        '''
        return hash((self.obstacle))

    def __str__(self):
        return "(Row: {} Col: {} IsObstacle: {} Cost:{})".format(self.row, self.col, self.obstacle, self.f)

    def addNeighbors(self, grid):
        '''
        Add the neighbors of a cell to self.neighbours
        '''
        if self.row < grid.rows-1:
            self.neighbors.add(grid.grid[self.row+1][self.col]) # Bottom
            if self.col < grid.cols-1:
                self.neighbors.add(grid.grid[self.row+1][self.col+1]) # Bottom-Right

        if self.row > 0:
            self.neighbors.add(grid.grid[self.row-1][self.col]) # Top
            if self.col > 0:
                self.neighbors.add(grid.grid[self.row-1][self.col-1]) # Top-Left

        if self.col < grid.cols-1:
            self.neighbors.add(grid.grid[self.row][self.col+1]) # Right
            if self.row > 0:
                self.neighbors.add(grid.grid[self.row-1][self.col+1]) # Top Right

        if self.col > 0:
            self.neighbors.add(grid.grid[self.row][self.col-1]) # Left
            if self.row < grid.rows - 1:
                self.neighbors.add(grid.grid[self.row+1][self.col-1]) # Top-Left

    def obstacleNeighborCount(self):
        count = 0
        for n in self.neighbors:
            if (self.row == n.row and self.col != n.col) or (self.row != n.row and self.col != n.col):
                count += 1
            # if n.obstacle == True:
            #     count += 1
                # if (n.row == self.row) or (n.col == self.col):
                #     count += 1
        return count


class Grid:
    '''
    A generic grid class that abstracts the whole map and helps make
    initialization of maps more generic
    '''
    def __init__(self, arr, cols=18, rows=20):
        '''
        arr - 1-D array pre-supplied that spots the cells of the global grid
            to denote 0 if empty or 1 if has obstacle (can be avoided)
        '''
        self.rows = rows
        self.cols = cols
        self.grid = []
        temp_row = []
        for r in range(self.rows):
            for c in range(self.cols):
                temp_row.append(Cell(r, c))
            self.grid.append(temp_row)
            temp_row = []

        temp = numpy.reshape(arr, (rows, cols))

        for i in range(len(temp)):
            for j in range(len(temp[i])):
                if temp[i][j] == 1:
                    self.grid[i][j].obstacle = True

        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                self.grid[i][j].addNeighbors(self)

    def A_star(self, s=Cell(11, 1), g=Cell(1, 13)):
        # (11,1) (1,13) -> Test default
        '''
        Basic A-star implementation based on heuristics control
        '''

        # Initialize require components
        openSet = set() # Set of elements that need to be checked for new paths
        closedSet = set() # Set of elements that are not in optimal path
        path = list() # List of nodes in the path
        start = self.grid[s.row][s.col] # Set the start node as passed
        goal = self.grid[g.row][g.col] # Set the goal node as passed
        print "Start: "+str(start)
        print "Goal: "+str(goal)

        openSet.add(start) #Add the start node to the Open Set
        count = 0
        while len(openSet) > 0:
            current = openSet.pop()
            # Actual A-star logic
            count += 1
            for cell in openSet:
                if cell.f < current.f:
                    current = cell
            if current == goal:
                print "Destination reached!"
                temp = current
                path.append(temp)

                while temp.previous is not None:
                    path.append(temp.previous)
                    temp = temp.previous
                path.reverse()
                return path
            openSet.discard(current)
            closedSet.add(current)
            # Iterate over neighbors of current node
            for neighbor in current.neighbors:
                if neighbor in closedSet:
                    continue
                elif (neighbor not in closedSet) and (not neighbor.obstacle):
                    neighbor.h = self.heuristic(neighbor, goal)
                    tempG = current.g + self.cost(current, neighbor)
                    newPath = False # To check if previous needs to be updated
                    if neighbor in openSet:
                        if tempG < neighbor.g:
                            neighbor.g = tempG
                            neighbor.f = neighbor.g + neighbor.h
                            newPath = True
                    else:
                        neighbor.g = tempG
                        neighbor.f = neighbor.g + neighbor.h
                        newPath = True
                        openSet.add(neighbor)
                    if newPath:
                        neighbor.f = neighbor.g + neighbor.h
                        neighbor.previous = current


        # In case A-star fails to return
        print "A-star failed to find a path after processing {} nodes".format(count)
        return None

    def heuristic(self, node1, node2, euc=False):
        '''
        euc flag determines whether to choose Manhattan or Euclidean for h-val
        True = Euclidean as heuristic
        False = Manhattan as heuristic
        '''
        if euc:
            return self.__euclidean(node1, node2)
        else:
            return self.__manhattan(node1, node2)

    def __euclidean(self, cell1, cell2):
        return math.sqrt((cell1.row - cell2.row)**2 + (cell1.col - cell2.col)**2)

    def __manhattan(self, cell1, cell2):
        return (abs(cell1.row - cell2.row) + abs(cell1.col - cell2.col))

    def cost(self, cell1, cell2):
        temp = 0
        if cell1.row != cell2.row:
            temp += 1

        if cell1.col != cell2.col:
            temp += 1

        if cell2.obstacleNeighborCount() > 1:
            temp += 5

        # Implement higher penalty in cost for nodes that might be false positives
        # near thin obstacles
        if temp == 0:
            return 0 # Return 0 if same node
        elif temp == 1:
            return 1 # Return 1 if horizontal or vertical neighbor
        elif temp == 2:
            return 1.4 # Return 1.4 (sqrt(2)) if diagonal neighbor
        elif temp > 2:
            return 6 # Return 5 if neighbor has more than 2 obstacles


if __name__ == "__main__":
    '''
    Define basic visualization for viewing the map, obstacles
    and the generated path from the A* algorithm
    '''
    map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
            0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
            0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

    g = Grid(map)
    p = g.A_star()
    if p is None:
        exit()
    print "Map and path visualization:"
    for i in range(len(g.grid)):
        for j in range(len(g.grid[0])):
            if (g.grid[i][j]).obstacle == True:
                print bcolors.FAIL + "1" + bcolors.ENDC,
            elif (g.grid[i][j]) in p:
                print bcolors.OKGREEN + "+" + bcolors.ENDC,
            else:
                print bcolors.OKBLUE + "0" + bcolors.ENDC,
        print ""
    for node in p:
        print node
