import lab3
import rospy
import roslib

from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
#from AStar import AStar
import time
from tf.transformations import euler_from_quaternion
#init globals
resolution = 1
width = 0
height = 0
mapData = 0

#setup subs/pubs for map and path
def initGridCell():
    global openPub
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    openPub = rospy.Publisher('/cell_path/open', GridCells, queue_size=10)

#convert the 1d input list to a 2d array
#need the given height/width to determine size of array
def map1Dto2D(height,width,data):
    h = height
    w = width
    #propagate new array initialized to 0
    map2D = [[0 for x in range(w)] for x in range(h)]
    i = 0
    #actually assign vals to new array
    for y in range(h):
	for x in range(w):
		map2D[x][y] = data[i]
		i = i + 1
    return map2D
		
#callback for reading map
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

#publishes gridcells to ROS
def publishGridCells():
    global resolution
    gridCells = GridCells()
    #attach a timestamp to the message
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    #need to properly scale cells to make them fit
    #correctly in Rviz
    scale = 1
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution)
    global openPub
    pointList = []
    #populate the list of points to be sent to rviz
    for x in range(1,width):
        for y in range(1,height):
            p = Point()
            p.x = float(x/xyscale)
            p.y = float(y/xyscale)
            p.z = 0
            pointList.append(p)
    #assign and send
    gridCells.cells = pointList
    openPub.publish(gridCells)
