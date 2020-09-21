#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from ibx0020_control import Diag
import tf    # for a function to transform quaternion into euler
import math  # for sqrt and atan2 functions


# utility function to normalyze angles (-pi <= a <= pi)
def normalizeAngle(a):
    while a < -math.pi:
        a += 2.0*math.pi
    while a > +math.pi:
        a -= 2.0*math.pi
    return a

# global variable to store the last received odometry reading
lastOdomReading = None


# callback function to process data from subscribed Odometry topic
def odometryReceived(data):
    global lastOdomReading
    lastOdomReading = data


# main function of the node
def control():
    global lastOdomReading

    # subscribing to odometry and announcing published topics
    rospy.Subscriber("base_pose_ground_truth", Odometry, odometryReceived)
    pub = rospy.Publisher('cmd_vel', Twist)
    pub_p = rospy.Publisher('p', Float64)
    abd_pub = rospy.Publisher('diag',10000)

    r = rospy.Rate(10)  # an object to maintain specific frequency of a control loop - 10hz

    # getting coefficients from Parameter Server (with defaults, if no coefficient is set)
    Kp = rospy.get_param('Kp', 0.3)
    Ka = rospy.get_param('Ka', 0.8)
    Kb = rospy.get_param('Kb', -0.15)
    rospy.loginfo('Coefficients: Kp='+str(Kp)+', Ka='+str(Ka)+', Kb='+str(Kb))

    # goal pose: coordinates (x,y) in metres and orintation (th) in radians
    goal_x = rospy.get_param('~x', 0)
    goal_y = rospy.get_param('~y', 0)
    goal_th = 0  # ~th is 0 for now, since control equation has hardcoded final orientation of 0
    rospy.loginfo('Goal: x='+str(goal_x)+', y='+str(goal_y)+', th='+str(goal_th))

    cmd = Twist()  # command that will be sent to Stage (published)
    abd_diag = Diag()
    
    while not rospy.is_shutdown():
        if lastOdomReading is None:  # we cannot issue any commands until we have our position
            print 'waiting for lastOdomReading to become available'
            r.sleep()
            continue

        # current robot 2D coordinates and orientation
        pose = lastOdomReading.pose.pose  # robots current pose (position and orientation)
        x = pose.position.x
        y = pose.position.y
        # euler_from_quaternion returns array of [pitch, roll, yaw], we need only yaw (rotation around Z axis)
        th = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

        # deltas
        dx = goal_x - x
        dy = goal_y - y
        dth = goal_th - th

        print x, y, th  # outputing current x, y anf th to standard output
        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and math.fabs(dth) < 0.10:
            # if we are acceptably close to the goal, we can exit
            print 'Reached destination'
            break

        # control equations (slide 25)
        p = math.sqrt(dx*dx + dy*dy)
        a = normalizeAngle(-th + math.atan2(dy, dx))
        b = normalizeAngle(-th - a)
        # control equations (slide 26)
        v = Kp*p  # translational speed in m/s
        w = normalizeAngle(Ka*a + Kb*b)  # rotational speed in rad/s
        # setting command fields
        cmd.linear.x = v
        cmd.angular.z = w
        abd_diag.x = x
        abd_diag.y = y
        abd_diag.th = th
        abd_diag.dy = dy
        abd_diag.dx = dx
        abd_diag.dth = dth
        abd_diag.p = p
        abd_diag.a = a
        abd_diag.b = b
        abd_diag.v = v
        abd_diag.w = w
        abd_diag.Kp = Kp
        abd_diag.Ka = Ka
        abd_diag.Kb = Kb
        abd_diag.student_name = "Abhinand Dathatri"
        abd_diag.position_error = sqrt(pow((goal_x - x), 2)+pow((goal_y - y), 2))
        abd_diag.orientation_error = goal_th - th
        # publishing command to a robot
        pub.publish(cmd)
        abd_pub.publish(abd_diag)
        # publishing p for rqt_plot
        p_ = Float64()
        p_.data = p
        pub_p.publish(p_)

        # sleeping so, that the loop won't run faster than r's frequency
        r.sleep()
        # end of loop
    # end of function

# entry point of the executable
# calling the main node function of the node only if this .py file is executed directly, not imported
if __name__ == '__main__':
    rospy.init_node('control')
    control()
