import sys
import rospy
#from beginner_tutorials.srv import *
from ywu_lab3.srv import aStar, aStarResponse

def astarclient(x0,y0,x1,y1):
    rospy.wait_for_service('/a_star_server')
    try:
        astarsearch = rospy.ServiceProxy('/a_star_server',aStar)
        resp1 = astarsearch(x0,y0,x1,y1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        x0 = int(sys.argv[1])
        y0 = int(sys.argv[2])
        x1 = int(sys.argv[3])
        y1 = int(sys.argv[4])

    else:
        print usage()
        sys.exit(1)
    print "Requesting %f, %f, %f, %f"%(x0,y0,x1,y1)
    print astarclient(x0,y0,x1,y1)