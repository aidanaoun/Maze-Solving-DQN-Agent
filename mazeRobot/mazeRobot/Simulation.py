#!/usr/bin/env python3
import numpy as np
import rclpy
from custom_interfaces.srv import MoveMotor, ReadSensors, ReadPose
from mazeRobot.Rewards import Rewards
from mazeRobot.Maze import Maze
import subprocess
import time
import math
import random


def cosDeg(degrees):
    return math.cos(math.radians(degrees))

def sinDeg(degrees):
    return math.sin(math.radians(degrees))


def initUnexploredSet(ROWS, COLUMNS, maze):
    unexploredSet = set()
    for y in range(0, ROWS):
        for x in range(COLUMNS):
            cell = maze.grid[y, x]
            if cell.isPath or cell.starting_point or cell.ending_point:
                unexploredSet.add((x, y))
    return unexploredSet


class Simulation:
    def __init__(self, agent, brainNode):
        self.brainNode = brainNode
        self.agent = agent
        self.STATE_DIM = 11

        self.xPos = None
        self.yPos = None
        self.theta = None

        self.curAction = 5
        self.prevAction = 5
        self.didOscillate = False
        self.steeredWell = False
        self.prevAngDiff = 0.0
        self.curThetaDiff = 0.0

        self.explored = False
        self.actionsTaken = 0
        self.MAX_ALLOWED_ACTIONS = 20000  
        self.CHUNK_RADIUS = 0.27
        self.exploredMazeCoords = set()
        self.unexploredMazeCoords = set()
        self.aimedAtYwypnt = False
        self.aimedAtXwaypnt = False
        self.goodAim = False
        self.optimalAngle = 90.0
        self.reachedWaypoint = False
        self.purposefulActions = False

        self.frontSensor = 0.0
        self.rightSensor = 0.0
        self.leftSensor = 0.0
        self.backSensor = 0.0
        self.frontleftSensor = 0.0
        self.frontrightSensor = 0.0
        self.backleftSensor = 0.0
        self.backrightSensor = 0.0

        self.currStateArr = np.zeros(self.STATE_DIM, dtype=np.float32)
        self.prev1StateArr = np.zeros(self.STATE_DIM, dtype=np.float32)
        self.prev2StateArr = np.zeros(self.STATE_DIM, dtype=np.float32)
        self.prev1coords = np.zeros(2, dtype=np.float32)

        self.waypoints = None
        self.curXwaypoint = 0
        self.curYwaypoint = 0
        self.closerToWaypntX = False
        self.closerToWaypntY = False
        self.movedaway = False

        self.ROWS = 10
        self.COLUMNS = 10
        self.WALL_SEPERATION = 3
        self.WALL_THICKNESS = 0.4
        self.maze = Maze(self.ROWS, self.COLUMNS,
                         self.WALL_THICKNESS, self.WALL_SEPERATION)
        self.maze.generate()
        self.maze_count = 0

        self.unexploredMazeCoords = initUnexploredSet(self.ROWS, self.COLUMNS, self.maze)

        self.respawnEntities()
        self.updateDistances()

    def reset(self):
        self.maze.generate()
        self.respawnEntities()
        self.updateCoordinates()
        self.updateDistances()

        self.exploredMazeCoords = set()
        self.unexploredMazeCoords = initUnexploredSet(self.ROWS, self.COLUMNS, self.maze)
        self.actionsTaken = 0
        self.updateStates(reset=True)
        self.resetWaypointList()

        totalStateArr = np.concatenate((self.currStateArr, self.prev1StateArr, self.prev2StateArr))
        totalStateArr = np.append(totalStateArr, [self.curXwaypoint / 15.0, self.curYwaypoint / 15.0, self.curThetaDiff / 180.0])

        return totalStateArr
    


    def gazeboToMazeCoords(self, x, y):
        mazeX = self.roundNum((x + 15) / self.WALL_SEPERATION)
        mazeY = self.roundNum((self.ROWS - 1) - ((y + 15) / self.WALL_SEPERATION))
        mazeX = max(0, min(self.COLUMNS - 1, mazeX))
        mazeY = max(0, min(self.ROWS - 1, mazeY))
        return mazeX, mazeY

    def mazeToWorldCoords(self, mazeX, mazeY):
        x = (self.WALL_SEPERATION * mazeX) - 15
        y = ((self.ROWS - mazeY - 1) * self.WALL_SEPERATION) - 15
        return x, y



    def updateStates(self, reset=False):
        compressionFactor = 6
        if reset:
            self.prev1coords = np.zeros(2, dtype=np.float32)
            self.prev1StateArr = np.zeros(self.STATE_DIM, dtype=np.float32)
            self.prev2StateArr = np.zeros(self.STATE_DIM, dtype=np.float32)
            self.updateDistances()
            
            self.currStateArr = np.array(
                [self.frontSensor / compressionFactor, self.rightSensor / compressionFactor, self.leftSensor / compressionFactor, self.backSensor / compressionFactor,
                self.frontleftSensor / compressionFactor, self.frontrightSensor / compressionFactor, self.backleftSensor / compressionFactor, self.backrightSensor / compressionFactor,
                self.theta, self.xPos, self.yPos], dtype=np.float32, )    # put these here if you want in future self.xPos / 15.0, self.yPos / 15.0
            return

        self.prev2StateArr = self.prev1StateArr
        self.prev1StateArr = self.currStateArr

        self.updateDistances()
        self.currStateArr = np.array(
                [self.frontSensor / compressionFactor, self.rightSensor / compressionFactor, self.leftSensor / compressionFactor, self.backSensor / compressionFactor,
                self.frontleftSensor / compressionFactor, self.frontrightSensor / compressionFactor, self.backleftSensor / compressionFactor, self.backrightSensor / compressionFactor,
                self.theta / 360.0, self.xPos / 15.0, self.yPos / 15.0], dtype=np.float32, )    # put these here if you want in future self.xPos / 15.0, self.yPos / 15.0



    def moveMotor(self, request: MoveMotor.Request):
        future = self.brainNode.motorClient.call_async(request)
        # Light-weight wait instead of full spin
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.brainNode, timeout_sec=0.001)

        return future.result()


    def updateCoordinates(self):
        time.sleep(0.01)
        readPoseRequest = ReadPose.Request()
        future = self.brainNode.poseClient.call_async(readPoseRequest)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.brainNode, timeout_sec=0.001)

        response = future.result()
        self.xPos = response.x
        self.yPos = response.y
        self.theta = response.theta



    def updateDistances(self):
        req = ReadSensors.Request()
        future = self.brainNode.sensorClient.call_async(req)
        # Light-weight wait
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.brainNode, timeout_sec=0.001)

        res = future.result()
        if res is not None:
            self.frontSensor = float(res.front)
            self.rightSensor = float(res.right)
            self.leftSensor = float(res.left)
            self.backSensor = float(res.back)

            self.frontleftSensor = float(res.frontleft)
            self.frontrightSensor = float(res.frontright)
            self.backleftSensor = float(res.backleft)
            self.backrightSensor = float(res.backright)



    def checkOscillation(self):
        if self.prevAction == 1 and self.curAction == 0:
            self.didOscillate = True
        elif self.prevAction == 0 and self.curAction == 1:
            self.didOscillate = True
        elif self.prevAction == 3 and self.curAction == 2:
            self.didOscillate = True
        elif self.prevAction == 2 and self.curAction == 3:
            self.didOscillate = True
        else:
            self.didOscillate = False



    def updateExploredSet(self):
        self.updateCoordinates()
        mazeX, mazeY = self.gazeboToMazeCoords(self.xPos, self.yPos)

        cell = self.maze.grid[mazeY, mazeX]
        if cell.isPath or cell.starting_point or cell.ending_point:
            coord = (mazeX, mazeY)
            self.exploredMazeCoords.add(coord)
            self.unexploredMazeCoords.discard(coord)
    


    def getNewTarget(self, curXmaze, curYmaze):
        candidates = []

        for x, y in self.unexploredMazeCoords:
            cell = self.maze.grid[y, x]
            if (x, y) in self.exploredMazeCoords:
                continue
            if cell.isPath or cell.starting_point or cell.ending_point:
                d = abs(x - curXmaze) + abs(y - curYmaze)
                candidates.append((d, (x, y)))

        if not candidates:
            return None
        minVal = min(candidates)
        indexOfMin = candidates.index(minVal)
        target = candidates[indexOfMin][1]
        return target



    def roundNum(self, num):
        if (num - math.floor(num)) < 0.5:
            num = math.floor(num)
        else:
            num = math.ceil(num)
        return int(num)



    def resetWaypointList(self, attempts=20):
        self.updateCoordinates()
        self.waypoints = []
        curXmaze, curYmaze = self.gazeboToMazeCoords(self.xPos, self.yPos)
        curXmaze = max(0, min(self.COLUMNS - 1, curXmaze))
        curYmaze = max(0, min(self.ROWS - 1, curYmaze))

        for _ in range(attempts):
            target = self.getNewTarget(curXmaze, curYmaze)
            if target is None:
                continue

            targetXmaze, targetYmaze = target
            if not (0 <= targetXmaze < self.COLUMNS and 0 <= targetYmaze < self.ROWS):
                continue

            mazeWaypoints = self.maze.find_waypoints(curXmaze, curYmaze, targetXmaze, targetYmaze)
            if not mazeWaypoints:
                continue

            mazeWaypoints.reverse()
            self.waypoints = []
            for (mx, my) in mazeWaypoints:
                gx = (self.WALL_SEPERATION * mx) - 15
                gy = ((self.ROWS - my - 1) * self.WALL_SEPERATION) - 15
                self.waypoints.append((gx, gy))

            if self.waypoints:
                self.curXwaypoint, self.curYwaypoint = self.waypoints[0]
                return

        self.curXwaypoint, self.curYwaypoint = self.xPos, self.yPos
        self.waypoints = []



    def updateWaypointStatus(self):
        # If we don't currently have a valid list, make one and exit
        if not self.waypoints:
            self.resetWaypointList()
            return
        xDiff = abs(self.xPos - self.curXwaypoint)
        yDiff = abs(self.yPos - self.curYwaypoint)
        xDiffPrev = abs(self.prev1coords[0] - self.curXwaypoint)
        yDiffPrev = abs(self.prev1coords[1] - self.curYwaypoint)
        rDiff = math.sqrt((xDiff ** 2) + (yDiff ** 2))
        rDiffPrev = math.sqrt((xDiffPrev ** 2 + yDiffPrev ** 2))

        if self.curAction == 0 or self.curAction == 1:
            straightAction = True
        else:
            straightAction = False
            self.movedaway = False
            self.closerToWaypnt = False
            
        if rDiff < rDiffPrev and straightAction:
            self.closerToWaypnt = True
        else:
            self.closerToWaypnt = False
            
        if rDiff > rDiffPrev and straightAction:
            self.movedaway = True
        else:
            self.movedaway = False

        if xDiff < 1.3 and yDiff < 1.3:
            self.reachedWaypoint = True
            waypntXmaze, waypntYmaze = self.gazeboToMazeCoords(self.waypoints[0][0], self.waypoints[0][1])

            self.exploredMazeCoords.add((waypntXmaze, waypntYmaze))    
            self.unexploredMazeCoords.discard((waypntXmaze, waypntYmaze))               
            self.waypoints.pop(0)

            # if there's another waypoint, advance to it
            if self.waypoints:
                self.curXwaypoint, self.curYwaypoint = self.waypoints[0]
            else:
                # no more waypoints -> get a new plan
                self.resetWaypointList()
        else:
            self.reachedWaypoint = False



    def updateOptimalAngle(self):
        self.updateCoordinates()
        yDiff = self.curYwaypoint - self.yPos
        xDiff = self.curXwaypoint - self.xPos

        if xDiff == 0 and yDiff >= 0:  # Divide by 0 edge case
            self.optimalAngle = 90.0
            return
        elif xDiff ==0 and yDiff < 0: # Divide by 0 edge case
            self.optimalAngle = 270.0
            return

        angle = math.atan(yDiff / xDiff)
        angle = ((angle * 180) / math.pi)
        angle = abs(angle)

        if xDiff < 0 < yDiff:  # quadrant 2
            angle = 180.0 - angle
        elif xDiff < 0 and yDiff < 0:  # quadrant 3
            angle += 180
        elif xDiff > 0 > yDiff:  # quadrant 4
            angle = 360.0 - angle

        self.optimalAngle = angle % 360

    def updateThetaDiff(self):
        theta = self.theta % 360
        optimal = self.optimalAngle % 360
        self.curThetaDiff = (optimal - theta + 180) % 360 - 180
        self.goodAim = abs(self.curThetaDiff) <= 32

    def checkIfSteeredWell(self):
        self.updateOptimalAngle()
        self.updateThetaDiff()

        if self.curAction == 0 or self.curAction == 1:
            self.steeredWell = False
            return

        if abs (self.curThetaDiff) < abs(self.prevAngDiff):
            self.steeredWell = True
        else:
            self.steeredWell = False

        self.prevAngDiff = self.curThetaDiff



    def performAction(self, motorRequest: MoveMotor.Request):
        time.sleep(0.05)
        #self.brainNode.get_logger().info(f"Aimed at new place: {self.aimedAtNewPlaceFwd}")
        if self.actionsTaken > 0:
            self.prevAction = self.curAction
        self.curAction = int(motorRequest.action)
        self.prev1coords = np.array([self.xPos, self.yPos], dtype=np.float32)
        self.moveMotor(motorRequest)

        self.actionsTaken += 1
        self.updateExploredSet()
        self.updateStates(reset=False)
        self.updateWaypointStatus()
        self.checkOscillation()
        self.checkIfSteeredWell()
        

        rewardCalculator = Rewards(self)
        reward = rewardCalculator.calculateReward()
        done = bool(rewardCalculator.hasReachedEnd() or (self.actionsTaken >= self.MAX_ALLOWED_ACTIONS) or (rewardCalculator.hasHitWall()) or rewardCalculator.erroneousSample == True)
        nextState = np.concatenate((self.currStateArr, self.prev1StateArr, self.prev2StateArr))
        nextState = np.append(nextState, [self.curXwaypoint / 15.0, self.curYwaypoint / 15.0, self.curThetaDiff / 180.0])

        return nextState, reward, done



    def respawnEntities(self):
        # Removes previous maze wall
        cmd = [
            "gz", "service",
            "-s", "/world/empty/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--req", 'name: "maze_walls" type: MODEL']
        result = subprocess.run(cmd, capture_output=True, text=True)
        spawnX = (self.WALL_SEPERATION * self.maze.start_x) - 15
        spawnY = ((self.ROWS - self.maze.start_y - 1) * self.WALL_SEPERATION) - 15

        # Resets position of robot
        finalCommandString = "name: \"maze_robot\" position {x: " + f"{spawnX} y: {spawnY}" + " z: 0.25 } orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }"
        cmd = [
            "gz", "service",
            "-s", "/world/empty/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--req", finalCommandString]
        result = subprocess.run(cmd)

        # Spawns the newly generated maze
        sdf_path = "/home/aidan24/maze_ws/src/mazeRobot_bringup/models/maze_walls.sdf"
        cmd = [
            "gz", "service",
            "-s", "/world/empty/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "500",
            "--req", f'sdf_filename: "{sdf_path}" name: "maze_walls" allow_renaming: false']
        result = subprocess.run(cmd, capture_output=True, text=True)

        self.maze_count += 1
        time.sleep(1.0)
        return
