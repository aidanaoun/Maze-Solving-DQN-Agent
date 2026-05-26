import random
import os
import numpy

class CellWall:
    def __init__(self):
        self.up = False
        self.down = False
        self.right = False
        self.left = False

        self.starting_point = False
        self.ending_point = False
        self.dead_end = False
        self.assigned = False
        self.isPath = False

class Maze:
    def __init__(self, columns, rows, wall_width, wall_seperation):
        self.COLUMNS = columns
        self.ROWS = rows
        self.wall_width = wall_width
        self.wall_seperation = wall_seperation
        self.grid = numpy.array([[CellWall() for _ in range(self.COLUMNS)]for _ in range(self.ROWS)])

        self.end_x = None
        self.end_y = None
        self.start_x = None
        self.start_y = None
        self.deadEndCoordinates = []
        self.pathCoordinates = []


    def indexToX(self, index):
        return index % self.COLUMNS
    def indexToY(self, index):
        return index // self.COLUMNS


    def assignCells(self, xCurr, yCurr, updateFlatList=False, unassignedList=None):
        indexRangeY = [yCurr - 1, yCurr, yCurr + 1]
        indexRangeX = [xCurr - 1, xCurr, xCurr + 1]
        for yCurr in indexRangeY:
            for xCurr in indexRangeX:
                if (0 <= yCurr < self.ROWS) and (0 <= xCurr < self.COLUMNS):
                    self.grid[yCurr][xCurr].assigned = True
                    if updateFlatList:
                        surroundingIndex = (self.COLUMNS * yCurr) + xCurr
                        unassignedList[surroundingIndex] = False
        if updateFlatList:
            return unassignedList
        else:
            return None


    def assignStart(self):
        possibleStartsX = numpy.array([2, 3, 4, 5, 6, 7])
        possibleStartsY = numpy.array([6, 7, 8, 9])
        self.start_x = random.choice(possibleStartsX)
        self.start_y = random.choice(possibleStartsY)
        self.grid[self.start_y][self.start_x].starting_point = True
        self.assignCells(self.start_x, self.start_y)


    # Sets end point but only if not assigned already
    def assignEnd(self):
        self.end_x = random.randrange(self.COLUMNS)
        self.end_y = 0
        while self.grid[self.end_y][self.end_x].assigned:
            self.end_x = random.randrange(self.COLUMNS)
        self.grid[self.end_y][self.end_x].ending_point = True
        self.grid[self.end_y][self.end_x].assigned = True
        self.assignCells(self.end_x, self.end_y)


    def assignDeadEnds(self):
        i = 0
        unassigned = [False] * (self.COLUMNS * self.ROWS)
        for row in self.grid:
            for cell in row:
                if cell.assigned:
                    unassigned[i] = False
                else:
                    unassigned[i] = True
                # print(unassigned[i])
                i += 1
        while any(unassigned):
            deadEndIndex = random.randrange(len(unassigned))
            if unassigned[deadEndIndex]:
                unassigned[deadEndIndex] = False
                x = self.indexToX(deadEndIndex)
                y = self.indexToY(deadEndIndex)
                self.grid[y][x].dead_end = True
                unassigned = self.assignCells(x, y, True, unassigned)


    # Traverse function walks one step closer to the goal each loop iteration
    def traverse(self, currX, currY, deadEndX, deadEndY):
        done = False
        while not done:
            currCell = self.grid[currY][currX]
            deadEndCell = self.grid[deadEndY][deadEndX]
            blockedPathY = False
            blockedPathX = False
            nextToDeadEnd = False
            deltaY = deadEndY - currY
            deltaX = deadEndX - currX

            if (currY, currX) not in self.pathSet:
                currCell.isPath = True
                self.pathCoordinates.append((currY, currX))
                self.pathSet.add((currY, currX))

            # Checking if path is blocked
            if deltaY != 0:
                nextYcell = self.grid[currY + (deltaY//abs(deltaY))][currX]
            else:
                nextYcell = self.grid[currY][currX]
            if deltaX != 0:
                nextXcell = self.grid[currY][currX + (deltaX // abs(deltaX))]
            else:
                nextXcell = self.grid[currY][currX]
            if (nextYcell.dead_end or nextYcell.ending_point):
                blockedPathY = True
            if (nextXcell.dead_end or nextXcell.ending_point):
                blockedPathX = True

            # Check and update Y values first
            if (deltaY != 0) and (not blockedPathY):
                currY += deltaY//abs(deltaY)
            elif (deltaX != 0) and (not blockedPathX):
                currX += deltaX//abs(deltaX)
            elif (deltaX == 1 or deltaX == -1 or deltaX == 0) and (deltaY == 1 or deltaY == -1 or deltaY == 0):
                nextToDeadEnd = True
                done = True
            else:
                if blockedPathY and (currX + 1) < self.COLUMNS:
                    currX += 1
                elif blockedPathY:
                    currX -= 1
                if blockedPathX and (currY - 1) >= 0:
                    currY -= 1
                    self.grid[currY][currX].isPath = True
                    self.pathCoordinates.append((currY, currX))
                    currX += deltaX//abs(deltaX)
                elif blockedPathX:
                    currY += 1
                    self.grid[currY][currX].isPath = True
                    self.pathCoordinates.append((currY, currX))
                    currX += deltaX//abs(deltaX)
            # Update dead end
            if nextToDeadEnd:
                if deltaY == -1:
                    deadEndCell.up = True
                    deadEndCell.right = True
                    deadEndCell.left = True
                elif deltaY == 1:
                    deadEndCell.down = True
                    deadEndCell.right = True
                    deadEndCell.left = True
                elif deltaX == 1:
                    deadEndCell.up = True
                    deadEndCell.down = True
                    deadEndCell.right = True
                elif deltaX == -1:
                    deadEndCell.up = True
                    deadEndCell.down = True
                    deadEndCell.left = True


    def assignWalls(self):
        for curY in range(self.ROWS):
            for curX in range(self.COLUMNS):
                cell = self.grid[curY][curX]
                if (not cell.isPath) or cell.dead_end or cell.ending_point:
                    continue

                # Assigning outer borders
                if curX == 0 and not cell.ending_point:
                    cell.left = True
                if curX == self.COLUMNS-1 and not cell.ending_point:
                    cell.right = True
                if curY == 0 and not cell.ending_point:
                    cell.up = True
                if curY == self.ROWS-1 and not cell.ending_point:
                    cell.down = True

                # Considering blank adjacent cells
                leftCell = CellWall()
                rightCell = CellWall()
                upCell = CellWall()
                downCell = CellWall()
                
                if curX - 1 >= 0:
                    leftCell = self.grid[curY][curX - 1]
                if curX + 1 < self.COLUMNS:
                    rightCell = self.grid[curY][curX + 1]
                if curY - 1 >= 0:
                    upCell = self.grid[curY - 1][curX]
                if curY + 1 < self.ROWS:
                    downCell = self.grid[curY + 1][curX]

                # Update walls if adjacents are blank and not dead ends
                if (not leftCell.isPath) and (not leftCell.dead_end) and (not leftCell.ending_point):
                    cell.left = True
                if (not rightCell.isPath) and (not rightCell.dead_end) and (not rightCell.ending_point):
                    cell.right = True
                if (not upCell.isPath) and (not upCell.dead_end) and (not upCell.ending_point):
                    cell.up = True
                if (not downCell.isPath) and (not downCell.dead_end) and (not downCell.ending_point):
                    cell.down = True

        self.grid[self.end_y][self.end_x].up = False
        self.grid[self.end_y][self.end_x].down = False
        self.grid[self.end_y][self.end_x].left = False
        self.grid[self.end_y][self.end_x].right = False

    def createPaths(self):
        # Determines a starting point in grid
        self.grid[self.start_y][self.start_x].isPath = True
        self.pathCoordinates.append((self.start_y, self.start_x))

        # Determines distances of each dead end and put them in a list
        node_distances = []
        for yCurr in range(self.ROWS):
            for xCurr in range(self.COLUMNS):
                if (self.grid[yCurr][xCurr].dead_end or self.grid[yCurr][xCurr].ending_point):
                    currDist = abs(yCurr - self.start_y) + abs(xCurr - self.start_x)
                    node_distances.append(currDist)
                    self.deadEndCoordinates.append((yCurr, xCurr))

        # Each iteration of loop gets a dead end and creates a path to it by calling the traverse function
        while node_distances:
            currIndex = node_distances.index(min(node_distances))
            currDeadEndX = self.deadEndCoordinates[currIndex][1]
            currDeadEndY = self.deadEndCoordinates[currIndex][0]

            i = 0
            distsPathToDead = []
            for _ in self.pathCoordinates:
                stepsY = self.pathCoordinates[i][0]
                stepsX = self.pathCoordinates[i][1]
                xdist = abs(currDeadEndX - stepsX)
                ydist = abs(currDeadEndY - stepsY)
                distsPathToDead.append(xdist + ydist)
                i += 1

            closestXindex = distsPathToDead.index(min(distsPathToDead))
            closestX = self.pathCoordinates[closestXindex][1]
            closestY = self.pathCoordinates[closestXindex][0]

            self.traverse(closestX, closestY, currDeadEndX, currDeadEndY)
            node_distances.pop(currIndex)
            self.deadEndCoordinates.pop(currIndex)
        self.assignWalls()
        return



    def constructWalls(self):
        xGridStart = str((-self.COLUMNS * self.wall_seperation) // 2)
        yGridStart = str((-self.ROWS * self.wall_seperation) // 2)
        newMazeSDFparts = []
        newMazeSDFparts.append(f"<sdf version=\"1.9\"> <model name=\"maze_walls\"><static>true</static><pose>{xGridStart} {yGridStart} 0 0 0 0</pose>")
        
        widthText = str(self.wall_width)
        collisionWidthText = str(self.wall_width + 0.0) #FIXME this value may need to be nonzero
        seperationText = str(self.wall_seperation)


        i = 0
        for curY in range(self.ROWS): 
            for curX in range(self.COLUMNS):
                curCell = self.grid[curY][curX]
                countText = str(i)

                if curCell.left:
                    xPos = str((self.wall_seperation * curX) - (self.wall_seperation/2))
                    yPos = str(self.wall_seperation * (self.ROWS - curY - 1))
                    newMazeSDFparts.append(f"<link name=\"leftWall{countText}\"> <pose>{xPos} {yPos} 0 0 0 0</pose> <collision name=\"leftWall{countText}collision\"> <geometry> <box> <size>{collisionWidthText} {seperationText} 2.0</size> </box> </geometry> </collision> <visual name=\"leftWall{countText}visual\"> <geometry> <box> <size>{widthText} {seperationText} 2.0</size> </box> </geometry> </visual> </link>")
                    
                if curCell.right:
                    xPos = str((self.wall_seperation * curX) + (self.wall_seperation/2))
                    yPos = str(self.wall_seperation * (self.ROWS - curY - 1))
                    newMazeSDFparts.append(f"<link name=\"rightWall{countText}\"> <pose>{xPos} {yPos} 0 0 0 0</pose> <collision name=\"rightWall{countText}collision\"> <geometry> <box> <size>{collisionWidthText} {seperationText} 2.0</size> </box> </geometry> </collision> <visual name=\"rightWall{countText}visual\"> <geometry> <box> <size>{widthText} {seperationText} 2.0</size> </box> </geometry> </visual> </link>")

                if curCell.down: 
                    xPos = str(self.wall_seperation * curX)
                    yPos = str((self.wall_seperation * (self.ROWS - curY - 1)) - (self.wall_seperation/2))
                    newMazeSDFparts.append(f"<link name=\"downWall{countText}\"> <pose>{xPos} {yPos} 0 0 0 0</pose> <collision name=\"downWall{countText}collision\"> <geometry> <box> <size>{seperationText} {collisionWidthText} 2.0</size> </box> </geometry> </collision> <visual name=\"downWall{countText}visual\"> <geometry> <box> <size>{seperationText} {widthText} 2.0</size> </box> </geometry> </visual> </link>")
                    
                if curCell.up:
                    xPos = str(self.wall_seperation * curX)
                    yPos = str((self.wall_seperation * (self.ROWS - curY - 1)) + (self.wall_seperation/2))
                    newMazeSDFparts.append(f"<link name=\"upWall{countText}\"> <pose>{xPos} {yPos} 0 0 0 0</pose> <collision name=\"upWall{countText}collision\"> <geometry> <box> <size>{seperationText} {collisionWidthText} 2.0</size> </box> </geometry> </collision> <visual name=\"upWall{countText}visual\"> <geometry> <box> <size>{seperationText} {widthText} 2.0</size> </box> </geometry> </visual> </link>")
                    
                i += 1

        newMazeSDFparts.append("</model></sdf>")
        return "".join(newMazeSDFparts)



    def can_move(self, cx, cy, nx, ny):
        if not (0 <= nx < self.COLUMNS and 0 <= ny < self.ROWS):
            return False

        cur = self.grid[cy, cx]
        nxt = self.grid[ny, nx]

        if not (nxt.isPath or nxt.starting_point or nxt.ending_point):
            return False

        if nx == cx and ny == cy - 1:
            return (not cur.up) and (not nxt.down)
        if nx == cx and ny == cy + 1:
            return (not cur.down) and (not nxt.up)
        if nx == cx - 1 and ny == cy:
            return (not cur.left) and (not nxt.right)
        if nx == cx + 1 and ny == cy:
            return (not cur.right) and (not nxt.left)

        return False



    def assignPossiblePaths(self, cx, cy, tx, ty, origin):
    # If current cell is already invalid, stop
        if not (0 <= cx < self.COLUMNS and 0 <= cy < self.ROWS):
            return []

        curCell = self.grid[cy, cx]
        dx = tx - cx
        dy = ty - cy

        # direction -> (blocked_flag, nx, ny, origin_name)
        cand = [
            ("up",    curCell.up,    cx,     cy-1, origin == "up"),
            ("down",  curCell.down,  cx,     cy+1, origin == "down"),
            ("left",  curCell.left,  cx-1,   cy,   origin == "left"),
            ("right", curCell.right, cx+1,   cy,   origin == "right"),
        ]

        legal = []
        for d, blocked, nx, ny, is_origin in cand:
            if blocked or is_origin:
                continue
            # bounds check
            if not (0 <= nx < self.COLUMNS and 0 <= ny < self.ROWS):
                continue
            
            if not self.can_move(cx, cy, nx, ny):
                continue
            
            legal.append(d)

        def score(direction):
            if direction == "up":    return 1 if dy < 0 else 0
            if direction == "down":  return 1 if dy > 0 else 0
            if direction == "left":  return 1 if dx < 0 else 0
            if direction == "right": return 1 if dx > 0 else 0
            return 0

        legal.sort(key=score, reverse=True)
        return legal



    def find_waypoints(self, curX, curY, targetX, targetY, origin="none", visited=None):

        if visited is None:
            visited = set()

        if (curX, curY) in visited:
            return None
        visited.add((curX, curY))

        if (curX == targetX) and (curY == targetY):
            return [(curX, curY)]

        possiblePaths = self.assignPossiblePaths(curX, curY, targetX, targetY, origin)

        for path in possiblePaths:
            if path == "up":
                nextX, nextY, nextOrigin = curX, curY - 1, "down"
            elif path == "down":
                nextX, nextY, nextOrigin = curX, curY + 1, "up"
            elif path == "right":
                nextX, nextY, nextOrigin = curX + 1, curY, "left"
            else:  # "left"
                nextX, nextY, nextOrigin = curX - 1, curY, "right"

            waypoints = self.find_waypoints(nextX, nextY, targetX, targetY, nextOrigin, visited)
            if waypoints is not None:
                waypoints.append((curX, curY))
                return waypoints

        return None



    def generate(self):
        self.grid = numpy.array([[CellWall() for _ in range(self.COLUMNS)] for _ in range(self.ROWS)])
        self.deadEndCoordinates = []
        self.pathCoordinates = []
        self.pathSet = set()

        self.assignStart()
        self.assignEnd()
        self.assignDeadEnds()
        self.createPaths()
        newMazeSDF = self.constructWalls()

        # Write SDF content to a temporary file and replace
        directory = "/home/aidan24/maze_ws/src/mazeRobot_bringup/models/maze_walls"
        filename = directory + ".sdf"
        temp_filename = filename + "_tmp.sdf"
        with open(temp_filename, 'w') as f:
           f.write(newMazeSDF)
        os.replace(temp_filename, filename)