import sys
import rospy
#from beginner_tutorials.srv import *
from ywu_lab3.srv import aStar, aStarResponse

#wait for the service to tell A* it should begin looking for a path
def astarclient(x0,y0,x1,y1):
    rospy.wait_for_service('/a_star_server')
    try:
        astarsearch = rospy.ServiceProxy('/a_star_server',aStar)
        #run A* on the given coords
        resp1 = astarsearch(x0,y0,x1,y1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#show the correct usage of this
def usage():
    return "%s [x y]"%sys.argv[0]

#get cmd line args corrsponding to start and end coords
if __name__ == "__main__":
    if len(sys.argv) == 5:
        x0 = int(sys.argv[1])
        y0 = int(sys.argv[2])
        x1 = int(sys.argv[3])
        y1 = int(sys.argv[4])

    else:
        print usage()
        sys.exit(1)
    #actually make the call to astar
    print "Requesting %f, %f, %f, %f"%(x0,y0,x1,y1)
    print astarclient(x0,y0,x1,y1)
