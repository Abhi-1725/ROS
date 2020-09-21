#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from tut_ibx0020.msg import Diag

import tf  # for a function to transform quaternion into euler
import math  # for sqrt and atan2 functions
import array as arr


# utility function to normalyze angles (-pi <= a <= pi)
def normalizeAngle(a):
    while a < -math.pi:
        a += 2.0 * math.pi
    while a > +math.pi:
        a -= 2.0 * math.pi
    return a


# global variable to store the last received odometry reading
lastOdomReading = None

# Minimum distances to obstacles set coliders true ( for easy editing)

f_min = 1.0
l_min = 1.50
r_min = 1.50
l45_min= 1.50
r45_min=1.50

def rgb2hsv(r, g, b):
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = df/mx
    v = mx
    return h, s, v

# callback function to process data from subscribed Odometry topic
def odometryReceived(data):
    global lastOdomReading
    lastOdomReading = data


# global variable to store the last received odometry reading
LaserReading = None


# callback function to process data from subscribed Odometry topic
def LaserReceived(data):
    global LaserReading
    LaserReading = data


callback = None #Global variable to store odometry
def Odom(data):
    global callback
    callback = data



def GetOdometry():
    global callback

    rospy.Subscriber("base_pose_ground_truth", Odometry, Odom)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if callback is None:

            r.sleep()
            continue

        pose = callback.pose.pose
        #rospy.loginfo('NU SHO TAM'+ str(pose))
        break






    return pose







def LaserColliders(): #returns two arrays 1) boolean coliders F,L,R,l45,R45
    #2) Distances at those those angles

    global LaserReading
    global f_min,r_min,l_min,r45_min,l45_min


    rospy.Subscriber("base_scan", LaserScan, LaserReceived)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():



        if LaserReading is None:
            r.sleep()
            continue
            ###########LASER functions ###############
        number_of_readings= len(LaserReading.ranges)
        view_angle_min = LaserReading.angle_min

        view_angle_max = LaserReading.angle_max
        #angle_per_reading= (view_angle_max -view_angle_min )/number_of_readings
        angle_per_reading=LaserReading.angle_increment

        #now find distances to objects measuring frontal , left right orientations

        f_angl=view_angle_max  # frontal distance measurement angle
        r_angl = 0
        l_angl =view_angle_min
        l45_angl = view_angle_max+0.5
        r45_angl = 0.5
        
   
   

        
#DEBUG
        #rospy.loginfo(str(LaserReading))
        
        #rospy.loginfo("NUM OF READINGS "+str(angle_per_reading))
        
        #END DEBUG



        #find correcponding indexes of ranges array for angles

        f_indx = f_angl / angle_per_reading
        f_indx = int(f_indx)


        #f_indx = abs(f_indx)


        l_indx = l_angl / angle_per_reading
        l_indx = int(l_indx)


        r_indx = r_angl / angle_per_reading
        r_indx = int(r_indx)


        l45_indx = l45_angl / angle_per_reading
        l45_indx = int(l45_indx)


        r45_indx = r45_angl / angle_per_reading
        r45_indx = int(r45_indx)
        
        f_indx = 6
        r_indx = 0
        l_indx = 12
        l45_indx = 9
        r45_indx = 3



        #find correcponding distances to obstacles


        f_dist = LaserReading.ranges[f_indx]
        l_dist = LaserReading.ranges[l_indx]
        r_dist = LaserReading.ranges[r_indx]
        l45_dist = LaserReading.ranges[l45_indx]
        r45_dist = LaserReading.ranges[r45_indx]

        f_colider= False
        l_colider= False
        r_colider= False
        l45_colider= False
        r45_colider= False

        if f_dist < f_min:
            f_colider=True

        if l_dist < l_min:
            l_colider=True
        if r_dist < r_min:
            r_colider=True


        if l45_dist < l45_min:
            l45_colider=True

        if r45_dist < r45_min:
            r45_colider=True

        collider_array = [f_colider,r_colider,l_colider,l45_colider,r45_colider]
        distance_array = [f_dist,r_dist,l_dist,l45_dist,r45_dist]

        break
    return collider_array,distance_array
    ##END OF LASER COLIDERS FUNCTION







def gotogoal(x,y,th,disable_goal_th,stop_before_obstacle):
    global lastOdomReading
    global LaserReading
    global callback

    # subscribing to odometry and announcing published topics
    #rospy.Subscriber("base_pose_ground_truth", Odometry, odometryReceived)

    ##############################
    #rospy.Subscriber("base_scan", LaserScan, LaserReceived)


    #############################################

    pub = rospy.Publisher('cmd_vel', Twist)

    ############################
    pub_Diag = rospy.Publisher('diag', Diag)

    DiagMsg = Diag()  # create diag message object
    ####################################

    r = rospy.Rate(10)  # an object to maintain specific frequency of a control loop - 10hz








    # goal pose: coordinates (x,y) in metres and orintation (th) in radians
    goal_x = x
    goal_y = y
    goal_th = th

    #goal_x = rospy.get_param('~x', 0)
    #goal_y = rospy.get_param('~y', 0)
    #goal_th = rospy.get_param('~th', 0)


    rospy.loginfo('Goal: x=' + str(goal_x) + ', y=' + str(goal_y) + ', th=' + str(goal_th))

    cmd = Twist()  # command that will be sent to Stage (published)

    while not rospy.is_shutdown():
        #if lastOdomReading is None:  # we cannot issue any commands until we have our position
            #print
            #'waiting for lastOdomReading to become available'
            #r.sleep()
            #continue

        # current robot 2D coordinates and orientation
        #pose = lastOdomReading.pose.pose  # robots current pose (position and orientation)

        pose = GetOdometry()
        x = pose.position.x
        y = pose.position.y

        # euler_from_quaternion returns array of [pitch, roll, yaw], we need only yaw (rotation around Z axis)
        th = tf.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

        #rospy.loginfo('MOVIG LINEARLY . Position x=' + str(x) + ', y=' + str(y) + ', th=' + str(th))

        #rospy.loginfo('LASER debugs angle min '+ str(LaserReading.angle_min) +'anglemax'+str(LaserReading.angle_max))
        #rospy.loginfo('range of array '+str(len(LaserReading.ranges)))
        # deltas
        dx = goal_x - x
        dy = goal_y - y
        dth = goal_th - th  # angle between current orientation and goal orientation


        #########DEBUG TEST##########


        #image = GetImage()
        #rospy.loginfo(str(image))

        ######END OF TEST DEBUG
        ###########LASER functions ###############

        colider_array,distance_array = LaserColliders()
        image = GetImage()



        '''

        number_of_readings= len(LaserReading.ranges)
        view_angle_min = LaserReading.angle_min
        view_angle_max = LaserReading.angle_max
        angle_per_reading= (view_angle_max -view_angle_min )/number_of_readings

        #now find distances to objects measuring frontal , left right orientations

        f_angl=view_angle_max  # frontal distance measurement angle
        l_angl = view_angle_max+1.57
        r_angl =view_angle_max -1.57
        l45_angl = view_angle_max+ 0.785
        r45_angl = view_angle_max - 0.785



        #find correcponding indexes of ranges array for angles

        f_indx = f_angl / angle_per_reading
        f_indx = int(f_indx)


        #f_indx = abs(f_indx)


        l_indx = l_angl / angle_per_reading
        l_indx = int(l_indx)


        r_indx = r_angl / angle_per_reading
        r_indx = int(r_indx)


        l45_indx = l45_angl / angle_per_reading
        l45_indx = int(l45_indx)


        r45_indx = r45_angl / angle_per_reading
        r45_indx = int(r45_indx)



        #find correcponding distances to obstacles


        f_dist = LaserReading.ranges[f_indx]
        l_dist = LaserReading.ranges[l_indx]
        r_dist = LaserReading.ranges[r_indx]
        l45_dist = LaserReading.ranges[l45_indx]
        r45_dist = LaserReading.ranges[r45_indx]


        #rospy.loginfo('front index '+str(f_indx))

        #rospy.loginfo('INITIALISING LASER angle_per_reading = '+ str(angle_per_reading))
        #rospy.loginfo('INITIALISING LASER view_angle_max = '+ str(view_angle_max))
        #rospy.loginfo('INITIALISING LASER view_angle_min = '+ str(view_angle_min))
        rospy.loginfo('Laser distance readings F ='+ str(f_dist)+' l='+ str(l_dist)+' r='+ str(r_dist)+' l45='+ str(l45_dist)+' r45='+ str(r45_dist))
        '''
        ############END OF LASER FUNCTIONS#######################
        
        if(image != False and stop_before_obstacle == True):
            rospy.loginfo("Red light ahead")
            break
            
        
        if((colider_array[0] == True or colider_array[1]== True or colider_array[2]== True and colider_array[3]== True and colider_array[4]==True ) and stop_before_obstacle == True ):
            rospy.loginfo("Obstacle ahead : STOP!!!!!!!!!!!!!!!!!!")
            break


        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and math.fabs(dth) < 0.10:
            # if we are acceptably close to the goal, we can exit
            rospy.loginfo('Destination Reached !!')
            break

        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and disable_goal_th == True:
            rospy.loginfo('Destination Reached !!')
            rospy.loginfo('No Orientation corrections were applied upon arrival !!')
            break



        # control equations
        p = math.sqrt(dx * dx + dy * dy)  # linear distance to goal
        a = normalizeAngle(-th + math.atan2(dy, dx))  # angle between current orientation th and p(dx,dy) vector
        b = normalizeAngle(-th - a)
        # control equations

        Kp = 0.5
        Ka = 0.5
        Kb = 0.5
        v = Kp * p  # translational speed in m/s

        # w = normalizeAngle(Ka*a + Kb*b)  # rotational speed in rad/s
        w = 0

        if math.fabs(dx) > 0.01 and math.fabs(dy) > 0.01 and math.fabs(a) > 0.1:
            #rospy.loginfo(' Setting proper orientation before linear movement dA = ' + str(a))
            v = 0
            w = 0.5 * a

        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and math.fabs(dth) > 0.03 and disable_goal_th == False:

            #rospy.loginfo(' X ,Y goal reached  , Applying TH orientation corrections')
            v = 0
            w = 0.5 * dth

        # setting command fields
        cmd.linear.x = v
        cmd.angular.z = w
        # publishing command to a robot
        pub.publish(cmd)

        # publishing Diag message
        # pub_Diag.

        DiagMsg.a = a
        DiagMsg.b = goal_th - th
        DiagMsg.dx = dx
        DiagMsg.dy = dy
        DiagMsg.Ka = Ka
        DiagMsg.Kp = Kp
        DiagMsg.Kb = Kb
        DiagMsg.x = x
        DiagMsg.y = y
        DiagMsg.student_name = "Jevgeni Potulov"
        DiagMsg.p = p
        DiagMsg.th = th
        DiagMsg.v = v
        DiagMsg.w = w
        DiagMsg.dth = dth

        DiagMsg.f_d = distance_array[0]
        DiagMsg.r_d = distance_array[1]
        DiagMsg.l_d = distance_array[2]
        DiagMsg.l45_d = distance_array[3]
        DiagMsg.r45_d = distance_array[4]

        pub_Diag.publish(DiagMsg)

        # sleeping so, that the loop won't run faster than r's frequency
        r.sleep()
        # end of loop
    # end of function




# main function of the node







callbackImage = None #Global variable to store odometry
def Imgcallback(data):
    global callbackImage
    callbackImage = data



def GetImage():
    global callbackImage
    #pixels = ["str","str"]
    pose = GetOdometry()
    th = tf.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    pixels = []
    i = None

    rospy.Subscriber("image", Image , Imgcallback)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if callbackImage is None:

            rospy.loginfo('WHatt?%!@')
            ##DEBUG

            ##

            r.sleep()
            continue

        image = callbackImage
        #rospy.loginfo('NU SHO TAM'+ str(image))
        break

    for index in range(image.width):
        pixel = image.data[index*4:index*4+4]
        r=ord(pixel[0])
        g=ord(pixel[1])
        b=ord(pixel[2])
        a= ord(pixel[3])

        #rospy.loginfo('R = '+str(r)+ ' G= '+str(g)+ ' B= '+str(g)+ ' a ='+str(a))

        if(r==128 and g==0 and b==0):
            pixels +="R"
            i= index #if red pixel found record value of index
            
        else:
            pixels +=" "
            
    if i==None:
        rospy.loginfo("red pixel NOT FOUND ")
        return False
    #rospy.loginfo("%s",pixels)
    rospy.loginfo(pixels)
    #Now calculate angle , based on index of red pixel
    angl= 2.0944* i*4/480
    # recalculate angle with respect to global frame
    if angl <=1.0472:
        red_pixel_angle = th+ (1.0472 -angl)
        rospy.loginfo("TYT")
    if angl >  1.0472:
        red_pixel_angle = th-(angl-1.0472)
    red_pixel_angle = normalizeAngle(red_pixel_angle)
    rospy.loginfo("RED PIXEL FOUND angle = "+str(red_pixel_angle))
    #rospy.loginfo(" angle = "+str(angl))
    #rospy.loginfo(" i = "+str(i*4))
    return red_pixel_angle



def obstacle_avoid(goal_x,goal_y,goal_th,enable_goal_th): #HARDCODED FOR 45 DEG bearing goal x,y, th disregarded
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        colider_array,distance_array = LaserColliders()
        front_colider = colider_array[0]
        right_colider = colider_array[1]
        left_colider = colider_array[2]
        r45_colider = colider_array[3]
        l45_colider = colider_array[4]
        
        red_pixel_angl= GetImage()
        
        #rospy.loginfo('FRONT ='+str(front_colider)+ ' RIGHT = '+str(right_colider)+ " LEFT= "+str(left_colider) )
    
        rospy.loginfo(str( colider_array))
        #rospy.loginfo(str( distance_array))
        ###
        pose = GetOdometry()
    
        x=pose.position.x
        y = pose.position.y
        th = tf.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
        dx = goal_x-x
        dy= goal_y-y
        dth = goal_th-th
        if(red_pixel_angl != False):
            rospy.loginfo('RED LIGHT')
            break
        
        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and enable_goal_th== False:
            # if we are acceptably close to the goal, we can exit
            rospy.loginfo('Destination Reached with obstacle avoidance!!')
            rospy.loginfo('Goal TH disregarded')
            break
        
        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and enable_goal_th == True:
            # if we are acceptably close to the goal, we can exit
            gotogoal(goal_x,goal_y,goal_th,False,False)
            rospy.loginfo('Destination Reached with obstacle avoidance!!')
            break

        
        
        '''
        if(front_colider == False):
            gotogoal(goal_x,goal_y,goal_th,True,True) # if nothing blocks way - go to goal 
            # (x,y,goal_th,False,True)   4th parameter(True) - disables goal_th
            # 5 parameter(True) means stop in front of obstacle 
            rospy.loginfo('GOING TO A GOAL')
        
        '''
        if (front_colider == True and right_colider == True and left_colider == False): # DEVIATE LEFT
            #gotogoal(x,y,th+0.3,False,False) #rotate and change orientation 
            #untill alligned with the wall 
            #Enable goal_th orientation , Disregard obstacles
            RotateLeft()
            rospy.loginfo('DEVIATE LEFT')
            
            
            
        if (left_colider == True and front_colider== True and right_colider== False): # DEVIATE RIGHT
            #gotogoal(x,y,th-0.3,False,False)
            RotateRight()
            rospy.loginfo('DEVIATE RIGHT')
                
        
        
        if(front_colider == False and right_colider == True and left_colider == False):# GO LEFT ALONG THE WALL (GO TO GOAL
            #gotogoal(goal_x,goal_y,goal_th,True,True)
            #gotogoal(x-0.4,y,th,True,True) #Disable goal orientation , Enable collider
            GoForwardRight()
            rospy.loginfo('coliderRight go ALONG THE WALL ')
            
            
        if(front_colider == False and left_colider == True and right_colider==False):# GO RIGHT ALONG THE WALL
            #gotogoal(goal_x,goal_y,goal_th,True,True)
            #gotogoal(x+0.4,y,th,True,True) #Disable goal orientation , Enable collider
            GoForwardLeft()
            rospy.loginfo('coliderLeft go ALONG THE WALL/')
        if(front_colider == True and right_colider == False and left_colider == False):
            #TURN LEFT
            #gotogoal(x,y,th+1.57,False,False)
            GoBackRIGHT()
            rospy.loginfo('F=t R=f L =f  Right')
            
        
        if(front_colider == True and right_colider == True and left_colider == True):
            #BACK
            GoBackRIGHT()
            #gotogoal(x,y,th+1.57,False,False)
            rospy.loginfo('F=t R=t L =t  GO BACK LEFT')
            
            
        if(front_colider == False and right_colider == False and left_colider == False):
            rospy.loginfo('Clear way - go to goal')
            gotogoal(x+3,y+3,goal_th,True,True) # GO to 45 deg bearing 
        
        if (front_colider == False and right_colider == True and left_colider == True):
            
            GoForwardCMD()
            rospy.loginfo('WALLs on both right and left side => going straight')
            
        
        if(front_colider == True and right_colider == True and left_colider == True):
            
            GoBackLEFT()
            rospy.loginfo('CORNER - turn arround')
        
        if(front_colider == False and right_colider == False and left_colider == False and r45_colider == True and l45_colider == False):
            GoBackRIGHT()
            
            
        if(front_colider == False and right_colider == False and left_colider == False and r45_colider == False and l45_colider == True):
            GoBackLEFT()
            
        
        
        '''
        else:
            colider_array,distance_array = LaserColliders()
            front_colider = colider_array[0]
            right_colider = colider_array[1]
            left_colider = colider_array[2]
            #RotateLeft()
            rospy.loginfo('CONDITIONS NOT SET')
            rospy.loginfo('FRONT ='+str(front_colider)+ ' RIGHT = '+str(right_colider)+ " LEFT= "+str(left_colider) )
          '''  
            
        r.sleep()
    
    ###
        
        

    
    
    
def GoBackCMD():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x =-0.2
    cmd.angular.z = 0
    
    
    pub.publish(cmd)    

def GoBackLEFT():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x =-0.5
    cmd.angular.z = 0.6
    
    
    pub.publish(cmd) 

def GoBackRIGHT():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x =-0.2
    cmd.angular.z = -0.6
    
    
    pub.publish(cmd) 


def GoForwardCMD():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0
    
    
    pub.publish(cmd)
    
def GoForwardRight():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.4
    pub.publish(cmd)    
    
def GoForwardLeft():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = -0.4
    pub.publish(cmd) 
 
 
 
def RotateLeft():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0.4
    pub.publish(cmd) 


def RotateRight():
    pub = rospy.Publisher('cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = -0.4
    pub.publish(cmd)   
# entry point of the executable
# calling the main node function of the node only if this .py file is executed directly, not imported
if __name__ == '__main__':


    #############MAIN FUNCTION######################
    rospy.init_node('control')

    rate = rospy.Rate(10) # 10hz
    test = 1
    
    #gotogoal(3,3,1.57,False,False)
    ##########################
    
    '''
    #gotogoal(10,16,0,False,False)
    obstacle_avoid(10,10,0,True)
    
    GoForwardCMD()
    
    
    red_pixel_angl = float(GetImage())
    pose = GetOdometry()
    x = pose.position.x
    y= pose.position.y
    
    
    
    #gotogoal(x,y,red_pixel_angl,False,False)
    
    rospy.loginfo(str( red_pixel_angl))

'''

    #gotogoal(3.5,2,red_pixel_angl,False)
    #colider_array,distance_array = LaserColliders()
    #rospy.loginfo(str( colider_array))
    #rospy.loginfo(str( distance_array))

    #obstacle_avoid()
    '''

    image = GetImage()
    rospy.loginfo('Height '+str(image.height))
    rospy.loginfo('width '+ str(image.width))
    rospy.loginfo('length of array '+ str(len(image.data)))
    '''

    '''
    for index in range(len(image.data)):
        pixel = image.data[index:index+3]
        r= ord(pixel[0])
        g=ord(pixel[1])
        b=ord(pixel[2])
        rospy.loginfo('index ='+ str(index))

        if r==255 and g==0 and b == 0:
            rospy.loginfo('RED FOUND !!! index ='+ str(index))

        #rospy.loginfo('r = '+str(r)+ ' g= '+str(g)+ ' b= '+str(g))



    '''
    
   


    #colider_array,distance_array = LaserColliders()
    #rospy.loginfo(str( colider_array))
    #poss = GetOdometry()

    #rospy.loginfo('MAIN LOOP GetODOMETRY'+str(poss))



    while not rospy.is_shutdown():

        ########MAIN WHILE LOOP ######
        ###### DO STUFF HERE ################
        #image = GetImage()
        #rospy.loginfo(str(image))

        #gotogoal(test,test,0,True)
        #rospy.loginfo('alala')

        colider_array,distance_array = LaserColliders()
        rospy.loginfo(str( distance_array))

        #poss = GetOdometry()

        #rospy.loginfo('MAIN LOOP GetODOMETRY'+str(poss))

        #obstacle_avoid()
        
        
        #gotogoal(9,-5,1.57,False)
    
        red_pixel_angl = float(GetImage())
        
        pose = GetOdometry()
        x = pose.position.x
        y= pose.position.y
        
        
        if(red_pixel_angl == False ):
            obstacle_avoid(10,10,0,True)
            
        
       
        
        
        if(red_pixel_angl != False):
            gotogoal(x,y,red_pixel_angl,False,False)
            
            
            
            
       
    
   
    
        #gotogoal(x,y,red_pixel_angl,False)
    
        #rospy.loginfo(str( red_pixel_angl))



        test = test+0.5
        rate.sleep()
        ## END OF MAIN WHILE LOOP


  ############END OF MAIN#################


