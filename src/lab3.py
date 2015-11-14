#!/usr/bin/env python
import gridCells

import rospy
import roslib
import time
import math

from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
#from AStar import AStar
import time
from tf.transformations import euler_from_quaternion

xInit = 0
yInit = 0
thetaInit = 0

xEnd = 0
yEnd = 0
thetaEnd = 0

def readGoal(msg):
    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi

	
def startCallBack(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi


class Cell:
    def __init__(self,x,y, f, g, h, blocked):
        self.point = Point(x,y)
        self.fScore = f
        self.gScore = g
        self.hScore = h
        self.blocked = blocked
        self.cameFrom = None

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class GridMap:
    def __init__(self, width, height):
	self.width = width
	self.height = height
	self.openSet = []
	self.closedSet = []
	self.map = []
	#populate the map with cell objects
	self.map = [[Cell(-1,-1,-1,-1,-1, 0) for x in range(width)] for x in range(height)]
	#properly assign coordinates to the cell objects
	for y in range(self.height):
	    for x in range(self.width):
	        self.map[y][x] = Cell(x,y,-1,-1,-1,0)

    # def __init__(self, width, height,data2D):
    #     self.width = width
    #     self.height = height
    #     self.openSet = []
    #     self.closedSet = []
    #     self.map = []
    #     self.map = [[Cell(-1,-1,-1,-1,-1, 0) for x in range(width)] for x in range(height)]
    #     for y in range(self.height):
    #         for x in range(self.width):
    #             self.map[y][x] = Cell(x,y,-1,-1,-1,data2D[x][y])

    def updateFScore(self, xPos, yPos, score):
        self.map[yPos][xPos].fScore = score

    def calculateFScore(self, xPos, yPos):
        self.map[yPos][xPos].fScore = self.map[yPos][xPos].gScore + self.map[yPos][xPos].hScore


    def updateGScore(self, xPos, yPos, score):
        self.map[yPos][xPos].gScore = score

    def updateHScore(self, xPos, yPos, score):
        self.map[yPos][xPos].hScore = score

    def updateScores(self, xPos, yPos, f, g, h):
        self.updateFScore(xPos, yPos, f)
        self.updateGScore(xPos, yPos, g)
        self.updateHScore(xPos, yPos, h)

    def printScores(self):
        for y in range(self.height):
            for x in range(self.width):
                print("[ %5d %5d %5d]" % (self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore)),
                #print("[",self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore, "]",
            print " "

    def printObstacles(self):
        for y in range(self.height):
            for x in range(self.width):
                print "[",self.map[y][x].blocked, "]",
            print " "

    def printCoords(self):
        for y in range(self.height):
            for x in range(self.width):
                print "[",self.map[y][x].point.x, self.map[y][x].point.y, "]",
            print " "
    def aStarSearch(self, startX, startY, goalX, goalY):
        self.closedSet = []
        self.openSet = []
        self.openSet.append(self.map[startY][startX])
        cameFrom = []

        for y in range(self.height):
            for x in range(self.width):
                #initialize fScores and gScores to 'infinity'
                self.updateGScore(x,y,99999)
                self.updateFScore(x,y,99999)

                #calculate the heuristic score for each block
                #use manhattan distance for now
                manhat_dist = abs(x-goalX) + abs(y-goalY)
                self.updateHScore(x,y, manhat_dist)

        #set the gScore of start position to 0
        self.updateGScore(startX, startY, 0)
        self.calculateFScore(startX, startY)

        #while openSet is not empty...
        while(len(self.openSet) != 0):
            print "Open set length: " ,len(self.openSet)
            #sort the list in order of increase fScore (lowest first)
            self.openSet.sort(key=lambda x: x.fScore)
            #pop the lowest off the open set and add it to the closed set
            currentCell = self.openSet.pop(0)

            print "Currently exploring:", currentCell.point.x, currentCell.point.y
            self.closedSet.append(currentCell)


            #if currentCell is the goal....
            if(currentCell.point.x == goalX and currentCell.point.y == goalY):
                return 0

            #check neighbors
            validNeighbors = self.getValidNeighbors(currentCell.point.x, currentCell.point.y)

            #expand each neighbor
            for neighbor in validNeighbors:
                #if the neighbor hasn't been expanded yet
                if(not self.isPointInClosedSet(neighbor)):
                    #cost to move one square over (cardinal directions)
                    tentativeGScore = self.map[currentCell.point.y][currentCell.point.x].gScore + 1
                    if(not self.isPointInOpenSet(neighbor)):
                        self.openSet.append(neighbor)
                        self.map[neighbor.point.y][neighbor.point.x].cameFrom = currentCell
                        self.updateGScore(neighbor.point.x, neighbor.point.y, tentativeGScore)
                        self.calculateFScore(neighbor.point.x, neighbor.point.y)
                    elif not (tentativeGScore >= self.map[neighbor.point.y][neighbor.point.x].gScore):
                        #continue
                        self.map[neighbor.point.y][neighbor.point.x].cameFrom = currentCell
                        self.updateGScore(neighbor.point.x, neighbor.point.y, tentativeGScore)
                        self.calculateFScore(neighbor.point.x, neighbor.point.y)






    #returns if a given point is in the closed set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isPointInClosedSet(self, p):
        for cell in self.closedSet:
            if(cell.point.x == p.point.x and cell.point.y == p.point.y):
                return True
        return False


    #returns if a given point is in the open set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isPointInOpenSet(self, p):
        for cell in self.openSet:
            if(cell.point.x == p.point.x and cell.point.y == p.point.y):
                return True
        return False


    #returns a list of valid neighbors that arent out of bounds
    #and arent blocked
    def getValidNeighbors(self, currentX, currentY):
        validNeighbors = []

        #check node above
        #make sure there's not a boundary issue
        if(currentY - 1 >= 0):
            #is this tile empty space? (== 0)
            if(self.map[currentY-1][currentX].blocked == 0):
                # print "This neighbor is marked as valid:", currentX, currentY-1
                validNeighbors.append(self.map[currentY-1][currentX]);

        #check node below
        if(currentY + 1 < self.height):
            if(self.map[currentY+1][currentX].blocked == 0):
                # print "This neighbor is marked as valid:", currentX, currentY+1
                validNeighbors.append(self.map[currentY+1][currentX]);

        #check node left
        if(currentX - 1 >= 0):
            if(self.map[currentY][currentX-1].blocked == 0):
                # print "This neighbor is marked as valid:", currentX-1, currentY
                validNeighbors.append(self.map[currentY][currentX-1]);

        #check node right
        if(currentX + 1 < self.width):
            if(self.map[currentY][currentX+1].blocked == 0):
                # print "This neighbor is marked as valid:", currentX+1, currentY
                validNeighbors.append(self.map[currentY][currentX+1]);

        return validNeighbors




g = GridMap(5,7)
g.map[0][1].blocked = 100
g.map[1][1].blocked = 100
g.map[2][1].blocked = 100
g.map[3][1].blocked = 100
g.map[5][1].blocked = 100
print g.map[5][1].blocked
g.printScores()
g.printObstacles()
g.printCoords()
g.aStarSearch(1,1,4,5)
print "\n\n\n"
g.printScores()

    # resolution and offset of the map

    # create a new instance of the map

    # generate a path to the start and end goals

    # for each node in the path, process the nodes to generate GridCells and Path messages
  
    # transform coordinates for map resolution and offset

    # continue making messages

    # do not stop publishing
'''
if __name__ == '__main__':
    try:
        global worldMap
        global target
        global openPub
        #rospy.sleep(5)
        AMap = 0
        worldMap = 0
        path = 0

        rospy.init_node('lab3')
        gridCells.initGridCell()
        markerSub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, readGoal)
        #pathPub = rospy.Publisher('/path_path', Path)

        target = 0
        start = 0
        end = 0
        # print gridCells.mapData
        rospy.sleep(4)
        filledMap = gridCells.map1Dto2D(gridCells.height,gridCells.width,gridCells.mapData)
        rospy.sleep(1)
        #print filledMap, gridCells.height
        while not rospy.is_shutdown():
            
            g = GridMap(gridCells.height, gridCells.width, filledMap)
            #		g.printScores()
            #		g.printObstacles()
            #		g.printCoords()
            #g.aStarSearch(2,2,4,5)
            #		print "\n\n\n"
            #		g.printScores()
            #gridCells.publishGridCells()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass   
'''