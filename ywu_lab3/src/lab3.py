#!/usr/bin/env python

import rospy
import roslib
import time
import math
import tf

from ywu_lab3.srv import aStar, aStarResponse
from tf.transformations import euler_from_quaternion
from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
#from AStar import AStar
import time
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

#init globals
xInit = 0
yInit = 0
thetaInit = 0

xEnd = 0
yEnd = 0
thetaEnd = 0
totalPath=[]
aStarList = []

scale = 2

#Kobuki Dimensions
wheel_rad  = 3.5  #cm
wheel_base = 23.0 #cm

#Odometry Data Variables
xPos = 2;
yPos = 2;
theta = 0;


#map callback
def realWorldMap(data):
# map listener
    global mapData, grid
    global width
    global height
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height

#goal pose message callback
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

#init pose message callback
def readInitPose(initpose):
    px = initpose.pose.pose.position.x
    py = initpose.pose.pose.position.y
    quat = initpose.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi
    print xInit,xInit,thetaInit

#callback for odom data @ start	
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

#prints the total path to the terminal
def printTotalPath():
    global resolution
    global scale
    #adjust for scaling
    fscale = float(scale)
    xyscale = 1/(resolution*scale)
    pathList = []
    #print each cell that occurs in the final path
    for cell in totalPath:
        print cell.point.x, cell.point.y
        p = Point()
        #need to put these in a new list for path msg
        p.x=cell.point.x
        p.y=cell.point.y
        p.z=0
        pathList.append(p)    
    #send the new path msg
    publishGridCellList(pathList,2)
    pth = publishTotalPathMsg(totalPath)
    #for pnt in pathList:
    #    navToPosePoint(float(pnt.x/xyscale)+1/(2*xyscale),float(pnt.y/xyscale)+1/(2*xyscale))
    #wayPoints(totalPath)
    # PublishGridCellPath(totalPath)
    #return pathList
    return pth

#converts a set of points into a set of poses to be published
def publishTotalPathMsg(pntPath):
    global resolution
    global scale
    global pathPub
    #new path message
    pth = Path()
    pth.header.frame_id = "/map"
    #time stamp
    pth.header.stamp = rospy.Time.now()
    poseList = []
    #scale adjustment
    xyscale = 1.0/(resolution*scale)
    for pnt in pntPath:  # list of cells?
        p = PoseStamped()
        #create a poseStamped message for each point in the collection
        # p.header.frame_id = "/map"
        # p.header.stamp = rospy.Time.now()
        p.pose.position.x = float(pnt.point.x/xyscale)+1/(2*xyscale)
        p.pose.position.y = float(pnt.point.y/xyscale)+1/(2*xyscale)
        p.pose.position.z = 0
        poseList.append(p)
    #pushlish me
    pth.poses = poseList
    pathPub.publish(pth)
    return pth


#represents instance on the 2D array of GridCells
#each cell stores its XY coordinate of the 2D array
#its 3 scores needed for the A* algorithm
#if there is an obstacle at the given location
#   (0 == No Obstacle, 100 == Obstacle, -1 == Unknown)
#cameFrom indicates which node led to this for path
#reconstruction
class Cell:
    def __init__(self,x,y, f, g, h, blocked):
        self.point = MyPoint(x,y)
        self.fScore = f
        self.gScore = g
        self.hScore = h
        self.blocked = blocked
        self.cameFrom = None

    def printCell(self):
        print self.point.x, self.point.y, self.fScore, self.gScore, self.hScore

#defines an x,y coordinate as an object
class MyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

#grid map is a 2D array of a given width and height
#w and h correspond to columns and rows of 2D array
#map is the 2D array of cells
#open set and closed set represent the frontier and
#expanded nodes, respectively
class GridMap:
    # def __init__(self, width, height):
    #     self.width = width
    #     self.height = height
    #     self.openSet = []
    #     self.closedSet = []
    #     self.map = []
    #     #populate the map with cell objects
    #     self.map = [[Cell(-1,-1,-1,-1,-1, 0) for x in range(width)] for x in range(height)]
    #     #properly assign coordinates to the cell objects
    #     for y in range(self.height):
    #         for x in range(self.width):
    #             self.map[y][x] = Cell(x,y,-1,-1,-1,0)
    
    def __init__(self, width, height, data2D):
    	#init variables from constructor
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
                self.map[y][x] = Cell(x,y,-1,-1,-1,data2D[x][y])
        #service to actually run A*
        self.performAstar = rospy.Service('/a_star_server', aStar, self.aStarSearch)

    #update fScore to a given fScore
    def updateFScore(self, xPos, yPos, score):
        self.map[yPos][xPos].fScore = score

    #calculate new fScore and store this in fScore,
    #based on cell's gScore and hScore
    def calculateFScore(self, xPos, yPos):
        self.map[yPos][xPos].fScore = self.map[yPos][xPos].gScore + self.map[yPos][xPos].hScore

    #update a node a position x,y gScore
    def updateGScore(self, xPos, yPos, score):
        self.map[yPos][xPos].gScore = score
    #update a cell @ X,Y with new hScore
    def updateHScore(self, xPos, yPos, score):
        self.map[yPos][xPos].hScore = score

    #update all 3 scores @ once
    def updateScores(self, xPos, yPos, f, g, h):
        self.updateFScore(xPos, yPos, f)
        self.updateGScore(xPos, yPos, g)
        self.updateHScore(xPos, yPos, h)

    #formatted print of scores in grid
    def printScores(self):
        for y in range(self.height):
            for x in range(self.width):
                # print x,("[ %3d %3d %3d]" % (self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore)),
                #print the f score if the node has actually been visited (non-visted will not be printed)
                if (self.map[y][x].fScore != 99999):
                    print ("[ %3d %3d %3d]" % (self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore)),
                #print("[",self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore, "]",
            print " "


    #formatted print of obstcles
    def printObstacles(self):
        for y in range(self.height):
            for x in range(self.width):
                print("[ %3d ]" % (self.map[y][x].blocked))
            print " "

    #formatted print of gridCells for verification
    def printCoords(self):
        for y in range(self.height):
            for x in range(self.width):
                print "[",self.map[y][x].point.x, self.map[y][x].point.y, "]",
            print " "

    #AStar search for a path from a start XY to a goal XY
    #returns a list of grid cells on successful completion
    #req is the service request
    def aStarSearch(self, req):
        global totalPath
        #initialize open and closed sets to 0
        self.closedSet = []
        self.openSet = []
        #get the start and end from req
        startX = req.x0
        startY = req.y0
        goalX = req.x1
        goalY = req.y1
        #debug print
        print "x0 %f" %startX + "x1 %f" %startY+"y0 %f" %goalX + "y1 %f" %goalY
        # add the start node to the open set
        self.openSet.append(self.map[startY][startX])

	#initialize f and g scores to inifinity (AKA 99999)
        for y in range(self.height):
            for x in range(self.width):
                #initialize fScores and gScores to 'infinity'
                self.updateGScore(x,y,99999)
                self.updateFScore(x,y,99999)

                # calculate the heuristic score for each block
                # use diagonal distance (Euclidean)
                diagonalDistance=sqrt( ((x-goalX)**2) + ((y-goalY)**2) )
                self.updateHScore(x,y, diagonalDistance)

        #set the gScore of start position to 0
        self.updateGScore(startX, startY, 0)
        self.calculateFScore(startX, startY)

        #while openSet is not empty...
        while(len(self.openSet) != 0):
            #print "Open set length: " ,len(self.openSet)
            #sort the list in order of increase fScore (lowest first)
            self.openSet.sort(key=lambda x: x.fScore)
            #pop the lowest off the open set and add it to the closed set
            currentCell = self.openSet.pop(0)
            
            #print the frontier
            for o in self.openSet:
                o.printCell()

            #print "Currently exploring:", currentCell.point.x, currentCell.point.y
            self.closedSet.append(currentCell)

            # publish currentCell as 'astar' cell to GridCells in Rviz
            pp=Point()
            pp.x=currentCell.point.x
            pp.y=currentCell.point.y
            pp.z=0
            aStarList.append(pp)
            publishGridCellList(aStarList,3)


            #if currentCell is the goal....
            if(currentCell.point.x == goalX and currentCell.point.y == goalY):
            	#reconstruct the path and add the path to RViz
                self.reconstructPath(currentCell)
                ret = aStarResponse()
                ret.path = printTotalPath()
                del totalPath[:]
                del aStarList[:]
                return ret

            #check neighbors
            #neighbor is valid if it is in the world map grid and is a free space (blocked == 0)
            validNeighbors = self.getValidNeighbors(currentCell.point.x, currentCell.point.y)

            #expand each neighbor
            for neighbor in validNeighbors:
                #if the neighbor hasn't been expanded yet
                if(not self.isMyPointInClosedSet(neighbor)):
                    # manhattan distance from start to currentCell
                    tentativeGScore = self.map[currentCell.point.y][currentCell.point.x].gScore + 0.5
                    # tentativeGScore = abs(x-goalX) + abs(y-goalY)
                    #add the neighbor to openset and update scores
                    if(not self.isMyPointInOpenSet(neighbor)):
                        self.openSet.append(neighbor)
                        self.map[neighbor.point.y][neighbor.point.x].cameFrom = currentCell
                        self.updateGScore(neighbor.point.x, neighbor.point.y, tentativeGScore)
                        self.calculateFScore(neighbor.point.x, neighbor.point.y)
                    #only update if this is a better path to the node
                    elif (tentativeGScore >= self.map[neighbor.point.y][neighbor.point.x].gScore):
                        continue

    #returns a list of the cells that needed to be visted to reach a goal
    #basically just recurse backwards through the list until you reach the 
    #first cell (which has Nothing in its camefrom field)
    def reconstructPath(self, currentCell):
        totalPath.append(currentCell)
        while(currentCell.cameFrom != None):
            currentCell = currentCell.cameFrom
            totalPath.append((currentCell))
        totalPath.reverse()
        for cell in totalPath:
            print "X:", cell.point.x, "Y:", cell.point.y


    #returns if a given point is in the closed set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isMyPointInClosedSet(self, p):
        for cell in self.closedSet:
            if(cell.point.x == p.point.x and cell.point.y == p.point.y):
                return True
        return False


    #returns if a given point is in the open set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isMyPointInOpenSet(self, p):
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
                #print "This neighbor is marked as valid:", currentX, currentY-1
                validNeighbors.append(self.map[currentY-1][currentX]);

        #check node below
        if(currentY + 1 < self.height):
            if(self.map[currentY+1][currentX].blocked == 0):
                #print "This neighbor is marked as valid:", currentX, currentY+1
                validNeighbors.append(self.map[currentY+1][currentX]);

        #check node left
        if(currentX - 1 >= 0):
            if(self.map[currentY][currentX-1].blocked == 0):
                #print "This neighbor is marked as valid:", currentX-1, currentY
                validNeighbors.append(self.map[currentY][currentX-1]);

        #check node right
        if(currentX + 1 < self.width):
            if(self.map[currentY][currentX+1].blocked == 0):
                #print "This neighbor is marked as valid:", currentX+1, currentY
                validNeighbors.append(self.map[currentY][currentX+1]);

        return validNeighbors


# @typ: 0=open 1=closed 2=path
#publish a single grid cell at the specified point
def publishGridCellPoint(pnt,typ):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution)
    p = Point()
    p.x= float(pnt.x/xyscale)
    p.y= float(pnt.y/xyscale)
    gridCells.cells=[p]
    #publish based on type
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)

#publishes a list of Point messages as GridCells
def publishGridCellList(lst,typ):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    
    #populate the point list
    pntList=[]
    for pnt in lst:
        p = Point()
        p.x= float(pnt.x/xyscale)+1/(2*xyscale)
        p.y= float(pnt.y/xyscale)+1/(2*xyscale)
        p.z=0
        pntList.append(p)

    #publish the correct type of point
    gridCells.cells=pntList
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)

    # resolution and offset of the map

    # create a new instance of the map

    # generate a path to the start and end goals

    # for each node in the path, process the nodes to generate GridCells and Path messages
  
    # transform coordinates for map resolution and offset

    # continue making messages

    # do not stop publishing

#callback for map data
def readWorldMap(data):
# map listener
    global mapData, grid
    global width
    global height
    global resolution
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution

#open all the necessary publishers and subscribers for grid cells
def initGridCell():
    global openPub
    global closedPub
    global pathVizPub
    global astarVizPub
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    openPub = rospy.Publisher('/cell_path/open', GridCells, queue_size=10)
    closedPub = rospy.Publisher('/cell_path/closed', GridCells, queue_size=10)
    pathVizPub = rospy.Publisher('/cell_path/path', GridCells, queue_size=10)
    astarVizPub = rospy.Publisher('/cell_path/astar', GridCells, queue_size=10)

# takes in a 2D map and scales it
def shrinkMap(width, height, mapData2D):
    newMap = [[0 for x in range(width/scale)] for x in range(height/scale)]
    #loops for days
    #properly adjust scaling factors for the map
    for x in range(width/scale):
        for y in range(height/scale):
            for j in range(scale):
                for k in range(scale):
                    if(mapData2D[x*scale+j][y*scale+k] == 100):
                        newMap[x][y] = 100
    return newMap         


# converts 1 d array of map data into a 2d array using height and width params
# of the given map
def map1Dto2D(width,height,data):
    map2D = [[0 for x in range(width)] for x in range(height)]
    i = 0
    for y in range(height):
        for x in range(width):
            map2D[x][y] = data[i]
            i = i + 1
    return map2D

#similar to other grid cell functions
#make a grid
def createOpenGrid():
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(1,width):
        for y in range(1,height):
            p = MyPoint()
            p.x = float(x/xyscale)
            p.y = float(y/xyscale)
            p.z = 0
            pointList.append(p)
    gridCells.cells = pointList
    openPub.publish(gridCells)

# publishes all closed cells in the given GridMap
def publishClosedCells(g):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(1,width):
        for y in range(1,height):
            if(g.map[x][y].blocked == 100):
                p = Point()#float(x/xyscale),float(y/xyscale))
                p.x = float(x/xyscale)
                p.y = float(y/xyscale)
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)

def publishClosedCellsShrink(map2D):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(width/scale):
        for y in range(height/scale):
            if(map2D[x][y] == 100):
                p = Point()#float(x/xyscale),float(y/xyscale))
                p.x = float(x/xyscale)+1/(2*xyscale)
                p.y = float(y/xyscale)+1/(2*xyscale)
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)

#def nav():
    #sub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, navToPose)
    #initposeSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readInitPose)

#publishTwist: publishes the Twist message to the cmd_vel_mux/input/teleop topic using the given linear(u) and angular(w) velocity

#move to a given point based on the robots current position
#based on function from lab2
def navToPosePoint(goal_x,goal_y):
    global xPos
    global yPos
    global theta
    #print "goals x %f" %(goal_x) + "goals y %f" %(goal_y) + "theta %f" %(theta)
    init_dist_x = xPos
    init_dist_y = yPos
    init_theta = theta
    #print "init x %f" %(init_dist_x) + "init y %f" %(init_dist_y) + "init theta %f" %(init_theta)
    rel_x = goal_x-init_dist_x
    rel_y = goal_y-init_dist_y
    goal_theta = math.atan2(rel_y,rel_x) * (180/3.14)
    #print "goal theta %f" %(goal_theta)
    distance = math.sqrt(pow(goal_x-init_dist_x,2) + pow(goal_y-init_dist_y,2))
    #print "spin!"
    rotate(goal_theta-init_theta)
    driveStraight(0.2, distance)

#send a twist message with given params
def publishTwist(u,w):
    global pub
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    pub.publish(twist)

#odometry callback imported from lab2
def odomCallback(data):
    global pose
    #pose = Pose()
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    x=position[0]
    y=position[1]
    w = orientation
    q = [w[0], w[1], w[2], w[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    global xPos
    global yPos
    global theta
    xPos = x
    yPos = y
    theta = math.degrees(yaw)
    #print xPos,yPos,theta

#imported from lab2
#driveStraight: drives thr robot forward at a desired speed for a certain amount of time
def driveStraight(maxspeed, distance):
    u = maxspeed;
    w = 0;
    minspeed = 0.05
    init_dist_x = xPos
    init_dist_y = yPos
    d_traveled = 0
    d_seg1 = 0.2 * distance
    d_seg2 = 0.8 * distance 

    while((d_traveled  < distance) and not rospy.is_shutdown()):
        #determine the distance and if you have gone far enough
        d_traveled = math.sqrt(pow(xPos-init_dist_x,2) + pow(yPos-init_dist_y,2))
        if (d_traveled < d_seg1):
            vel = (d_traveled/d_seg1)*maxspeed + minspeed
        elif (d_traveled < d_seg2):
            vel = maxspeed
        else:
            vel = ((distance - d_traveled)/(distance - d_seg2))*maxspeed + minspeed
        publishTwist(vel, 0)
        time.sleep(0.15)
    publishTwist(0, 0)

#imported from lab2
#rotate: rotates the robot around its center by a certain angle (in degrees)
#known to be buggy 
def rotate(angle):
    init_angle = theta
    desired_angle = init_angle + angle
    p = 0.015
    error = 0
    errorband = 2 
    if(desired_angle < -180) or (desired_angle >= 180):
        if(angle > 0):
            desired_angle = desired_angle - 360
        else:
            desired_angle = desired_angle + 360
    while(theta > desired_angle + errorband) or (theta < desired_angle - errorband):
        error = theta-desired_angle
        publishTwist(0,-error*p)
        time.sleep(0.05) 
    publishTwist(0, 0)


#main execution
if __name__ == '__main__':
    rospy.init_node('a_star_server')
    try:
    	#init the globals
        global worldMap
        global target
        global cellPub
        global scale
        global resolution
        global xInit, yInit, xEnd, yEnd
        global pose
        global odom_tf
        global odom_list

	#get odometry setup
        odom_list = tf.TransformListener()
	#variable initializations
        AMap = 0
        worldMap = 0
        path = 0
        scale = 8
        #setup publishers/subs
        sub = rospy.Subscriber('/odom', Odometry, odomCallback)
        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 5)
        markerSub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, readGoal)
        pathPub = rospy.Publisher('/path_path', Path)
        initposeSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readInitPose)
	
	#more pubs/subs
        initGridCell()
        # allow subscriber time to callback
        rospy.sleep(1)
        #get the map data from the map message
        filledMap = map1Dto2D(width, height,mapData)
        #shrink the map to a desired size/scale
        shrinkedMap = shrinkMap(width,height,filledMap)
        #publish the obstacles map to rviz
        publishClosedCellsShrink(shrinkedMap)
        ratio = 1.0/(resolution)
        #while (yEnd == 0 or xEnd == 0):
        #    print "waiting for start and goal"
        #setup gridMap and then wait for service messages/subscribed messages
        #to come in
        g = GridMap(width/scale, height/scale,shrinkedMap)
        rospy.spin()
        #a_star_server()
        #g.aStarSearch(int(xPos*ratio/scale),int(yPos*ratio/scale),int(xEnd*ratio/scale),int(yEnd*ratio/scale))
        #printTotalPath()
    except rospy.ROSInterruptException:
        pass
