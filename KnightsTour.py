import networkx as nx
import math
import numpy as np
import matplotlib.pyplot as plt
import time as tm

#node class
#each node keeps track of: its own position,
#whether or not the knight has landed on that square before (valid),
#and a list of valid jumps from this square (jumps that we can get to and haven't been to)
class TourNode():
    def __init__(self, pos):
        self.cord = pos
        self.validJumps = []
        self.valid = True

def createGrid(gridW, gridH):
    #We use an array of tuples to represent a grid
    grid = []
    for y in range(1, gridH+1):
        for x in range(1, gridW+1):
            grid.append((x,y))
    return grid


def possibleJumps(p, width, height):
    # A knight can jump 2 squares in the x direction and 1 in the y direction
    # OR 1 square in the x direciton and 2 in the y direction
    pos_x = [2, 1, 2, 1, -2, -1, -2, -1]
    pos_y = [1, 2, -1, -2, 1, 2, -1, -2]
    poss = []
    for i in range(len(pos_x)):
        if p[0] + pos_x[i] >= 1 and p[0] + pos_x[i] <= width and p[1] + pos_y[i] >= 1 and p[1] + pos_y[i] <= height:
            poss.append((p[0] + pos_x[i], p[1] + pos_y[i]))

    return poss

def makeGridGraph(grid):
    #Converts each square in our grid into a node and adds it to a graph
    graph = nx.Graph()
    for coordinates in grid:
        node = TourNode(coordinates)
        graph.add_node(coordinates, data =node)
    return graph

def getValidJumps(gridW, gridH, graph, start):
    possible = possibleJumps(start, gridW, gridH)
    #of the possible jumps, get the valid squares (squares we haven't been to before)
    for j in possible:
        if graph.nodes[j]['data'].valid:
            if j not in graph.nodes[start]['data'].validJumps:
                graph.nodes[start]['data'].validJumps.append(j)
                graph.add_edge(start, j)


# Warndorff's algorithm runs into a situation where there are multiple positions
# with the same amount of least possible moves
# Ira Pohl proposed that instead of arbitrarily picking one of these position, we
# get the sum of each of the positions' possible positions and jump to the position with the lowest sum
# Pohl's proposition also states that if there are duplicate sums, pick an arbitrary position
def pohlPro(gridW, gridH, graph, currPoint, minPoints):
    # geting the valid jumps of minPoint
    # then getting the sum of the valid jumps for each point.
    lenOfMinPts = {}
    for i in minPoints:
        graph.nodes[i]['data'].valid = False
        getValidJumps(gridW, gridH, graph, i)

    for i in minPoints:
        length = 0
        for j in graph.nodes[i]['data'].validJumps:
            getValidJumps(gridW, gridH, graph, j)
            length += len(graph.nodes[j]['data'].validJumps) - 1
        lenOfMinPts[i] = length

    # earlier in code we set our min points to False, indicating that the knight
    # has been to that square before, so now we reset the min points to True
    for i in minPoints:
        graph.nodes[i]['data'].valid = True

    toReturn = min(lenOfMinPts, key=lenOfMinPts.get)
    return toReturn


def pohlKnightTour(gridW, gridH, graph, point):
    # We set the start point to False to indicate we have landed here
    graph.nodes[point]['data'].valid = False
    grid = np.zeros((gridW, gridH))
    jumpNum = 1
    grid[point[1] - 1][point[0] - 1] = jumpNum
    # Then we set the valid jumps of our starting point
    getValidJumps(gridW, gridH, graph, point)
    counter = 0
    # going to be in a while loop:
    while counter < (gridW * gridH):
        nextJump = {}
        for n in graph.nodes[point]['data'].validJumps:
            # if the point is not already in the valid jumps then:
            getValidJumps(gridW, gridH, graph, n)

        # store the number/length of the valid jumps of the point's valid jumps in the nextJump dictionary
        for n in graph.nodes[point]['data'].validJumps:
            nextJump[graph.nodes[n]['data'].cord] = len(graph.nodes[n]['data'].validJumps)

        falseJumps = []
        # checking to see if we have been to the valid jumps before
        # if we have, remove them from the nextJump dictionary and
        # remove them from the point's valid jumps array
        for k in nextJump:
            if graph.nodes[k]['data'].valid == False:
                falseJumps.append(graph.nodes[k]['data'].cord)

        # if a move is a false jump, then remove it from the possible next jumps (the nextJump dictionary)
        for i in falseJumps:
            del nextJump[i]
            graph.nodes[point]['data'].validJumps.remove(i)

        # if there is no next jump then we're done, otherwise get the square with
        # the min number of possible jumps and jump there
        if len(nextJump) > 0:
            # If there are is more than one min point, get the valid jumps of those min points
            # then sum up their number of valid jumps
            minPts = []
            minPt = min(nextJump.values())
            for key in nextJump.keys():
                if nextJump[key] == minPt:
                    minPts.append(key)
            if len(minPts) > 1:
                point = pohlPro(gridW, gridH, graph, point, minPts)
            else:
                # else there is only one min so set our point to that min
                point = min(nextJump, key=nextJump.get)

            graph.nodes[point]['data'].valid = False
            getValidJumps(gridW, gridH, graph, point)
            jumpNum += 1
            grid[point[1] - 1][point[0] - 1] = jumpNum
            counter += 1
        else:
            return grid

def pohlKnightTourSolver(gridW, gridH, point):
    grid = createGrid(gridW, gridH)
    graph = makeGridGraph(grid)
    tourGrid = pohlKnightTour(gridW, gridH, graph, point)
    if tourFound(graph):
        print(tourGrid)
        return True
    print("Pohl's Proposition Knight Tour Not Found for Starting Point:", point)
    return False

def tourFound(graph):
    for i in graph:
        if graph.nodes[i]['data'].valid == True:
            #then we did not visit this square
            return False
    return True

#Arnd Roth proposed that if there are multiple positions with the same length of possible moves,
#pick the position that is furthest away from the center of the board. Again, if multiple positions
#that are the same distance from the center, arbitrarily pick a position
def rothProp(graph, minPoints, gridW, gridH):
    mid = (gridW//2, gridH//2)
    distance = {}
    for i in minPoints:
        x = graph.nodes[i]['data'].cord[0]
        y = graph.nodes[i]['data'].cord[1]
        d = (x-mid[0])**2 + (y-mid[1])**2
        d = math.sqrt(d)
        distance[i] = d
    return max(distance, key=distance.get)


def rothKnightTour(gridW, gridH, graph, point):
    grid = np.zeros((gridW, gridH))
    jumpNum = 1
    grid[point[1] - 1][point[0] - 1] = jumpNum
    graph.nodes[point]['data'].valid = False
    getValidJumps(gridW, gridH, graph, point)
    # get count
    counter = 0

    # going to be in a while loop:
    while counter < (gridW * gridH):
        nextJump = {}
        for n in graph.nodes[point]['data'].validJumps:
            # if the point is not in the valid jumps then:
            getValidJumps(gridW, gridH, graph, n)

        for n in graph.nodes[point]['data'].validJumps:
            nextJump[graph.nodes[n]['data'].cord] = len(graph.nodes[n]['data'].validJumps)

        falseJumps = []
        # checking to see if we have been to the valid jumps
        for k in nextJump:
            if graph.nodes[k]['data'].valid == False:
                falseJumps.append(graph.nodes[k]['data'].cord)

        # if a move is a false jump, then remove it from the possible next jumps
        for i in falseJumps:
            del nextJump[i]
            graph.nodes[point]['data'].validJumps.remove(i)

        # if there is no next jump then we're done, otherwise get the square with
        # the min number of possible jumps and jump there
        if len(nextJump) > 0:
            # Trying to apply roth's proposition:
            # If there are is more than one min, get the valid jumps of those two
            # then sum up the lenghth

            minPts = []
            minPt = min(nextJump.values())
            for key in nextJump.keys():
                if nextJump[key] == minPt:
                    minPts.append(key)
            if len(minPts) > 1:
                point = rothProp(graph, minPts, gridW, gridH)
            else:
                point = min(nextJump, key=nextJump.get)

            graph.nodes[point]['data'].valid = False
            getValidJumps(gridW, gridH, graph, point)
            jumpNum += 1
            grid[point[1] - 1][point[0] - 1] = jumpNum
            counter += 1
        else:
            return grid


def warnsdorffKnightTour(gridW, gridH, graph, point):
    grid = np.zeros((gridW, gridH))
    jumpNum = 1
    grid[point[1] - 1][point[0] - 1] = jumpNum
    graph.nodes[point]['data'].valid = False
    getValidJumps(gridW, gridH, graph, point)
    # get count
    counter = 0

    # going to be in a while loop:
    while counter < (gridW * gridH):
        nextJump = {}
        for n in graph.nodes[point]['data'].validJumps:
            # if the point is not in the valid jumps then:
            getValidJumps(gridW, gridH, graph, n)

        for n in graph.nodes[point]['data'].validJumps:
            nextJump[graph.nodes[n]['data'].cord] = len(graph.nodes[n]['data'].validJumps)

        falseJumps = []
        # checking to see if we have been to the valid jumps
        for k in nextJump:
            if graph.nodes[k]['data'].valid == False:
                falseJumps.append(graph.nodes[k]['data'].cord)

        # if a move is a false jump, then remove it from the possible next jumps
        for i in falseJumps:
            del nextJump[i]
            graph.nodes[point]['data'].validJumps.remove(i)

        # if there is no next jump then we're done, otherwise get the square with
        # the min number of possible jumps and jump there
        if len(nextJump) > 0:
            # Warnsdorff's algorithm takes the valid jump with the least number of valid jumps
            point = min(nextJump, key=nextJump.get)
            graph.nodes[point]['data'].valid = False
            getValidJumps(gridW, gridH, graph, point)
            graph.nodes[point]['data'].valid = False
            jumpNum += 1
            grid[point[1] - 1][point[0] - 1] = jumpNum
            counter += 1
        else:
            return grid

def validJump(grid, gridW, gridH, x, y ):
    if x < gridW and x >=0 and y < gridH and y >= 0 and grid[y][x] == 0 :
        return True


def setValidJumps(grid, gridW, gridH, currX, currY, possibleMoves):
    validJumps = []
    for i, j in possibleMoves:
        if validJump(grid, gridW, gridH, currX + j, currY + i):
            validJumps.append((i, j))
    return validJumps

def getNextSmallest(grid, gridW, gridH, currX, currY, possMoves):
    nextValidJumps = {}
    validJumps = setValidJumps(grid, gridW, gridH, currX, currY, possMoves)
    for (i, j) in validJumps:
        length = len(setValidJumps(grid, gridW, gridH, currX + j, currY + i, possMoves))
        nextValidJumps[(currY + i, currX + j)] = length

    return sorted(nextValidJumps, key=nextValidJumps.get)

def backTracking(grid, gridW, gridH, startX, startY, jumpCounter, possMoves):
    if jumpCounter >= (gridW*gridH)+1:
        return True
    for i,j in getNextSmallest(grid, gridW, gridH, startX, startY, possMoves):
        newX = j
        newY = i
        if validJump(grid, gridW, gridH, newX, newY):
            grid[newY][newX] = jumpCounter
            if backTracking(grid, gridW, gridH, newX, newY, jumpCounter+1, possMoves):
                return True
            grid[newY][newX] = 0
    return False

def solveKnightTour(gridWidth, gridHeight, startX, startY):
    grid = np.zeros((gridWidth, gridHeight))
    possibleMoves = [(1,2), (1,-2), (2,1), (2,-1), (-1,2), (-1,-2), (-2,1), (-2,-1)]
    jumpCounter = 2
    grid[startY][startX] = 1
    if backTracking(grid, gridWidth, gridHeight, startX, startY, jumpCounter, possibleMoves):
        for i in grid:
            print(i)
    else:
        print("No knight tour was found :(")



