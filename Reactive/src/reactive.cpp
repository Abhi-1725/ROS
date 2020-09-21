#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cmath> //for sqrt and atan2 functions
#include <algorithm>
#include <tf/tf.h> //to transform quaternion into euler

//global variable to store the last received odometry reading
nav_msgs::OdometryConstPtr lastOdomReading;
sensor_msgs::ImageConstPtr lastImageReading;
sensor_msgs::LaserScanConstPtr lastLaserReading;

float largest(float x, float y, float z){
   return std::max({x, y, z});
} 
    
float smallest(float x, float y, float z){
    return std::min({x, y, z}); 
}

void rgb2hsv(float l, float m, float n)
{
for(int i = 0; i < lastImageReading->width; i++)
    {
      red = lastImageReading->data[(i * 4) + 0]/255.0;
      green = lastImageReading->data[(i * 4) + 1]/255.0;
      blue = lastImageReading->data[(i * 4) + 2]/255.0;

        max = largest(red,green,blue);
        min = smallest(red,green,blue);
        delta = max - min;
		  
        if(max == min)
		{
			h = 0;
			}
		else if(max == red)
		{
			h = 60*((green - blue)/delta);
			}
		else if(max == green)
		{
			h = ((60*(blue - red )/delta) + 120);
			}		
		else if(max == blue)
		{
			h = ((60*(red - green)/delta) + 360);
			}
		if(max == 0)
		{
			s = 0;
			}
		else
		{
        s = delta/max;
        v = max;
        }
		return h, s, v;
	}		
}
//callback function to process data from subscribed Odometry topic
void odometryReceived(nav_msgs::OdometryConstPtr msg){
  lastOdomReading = msg;
}
void imageReceived(sensor_msgs::ImageConstPtr img){
  lastImageReading = img;
}
void rangeReceived(sensor_msgs::LaserScanConstPtr lsr){
  lastLaserReading = lsr;
}

void go_reverse()
{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x =-0.2;
    cmd.angular.z = 0;
    pub.publish(cmd) ;
}
void GoBackLEFT()
	{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x =-0.5;
    cmd.angular.z = 0.6;
    pub.publish(cmd);
	}
void GoBackRIGHT()
	{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x =-0.2;
    cmd.angular.z = -0.5;
    pub.publish(cmd) 
	}
	
void go_forward()
	{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x = 0.2;
    cmd.angular.z = 0;
    pub.publish(cmd)
	}
void Go_Right()
{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x = 0.2;
    cmd.angular.z = 0.4;
    pub.publish(cmd);    
}  
void go_Left()
{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x = 0.2;
    cmd.angular.z = -0.4;
    pub.publish(cmd) ;
 
}
void rotate_left()
{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x = 0;
    cmd.angular.z = 0.4;
    pub.publish(cmd) ;
}
void rotate_right()
{
    pub = rospy.Publisher('cmd_vel', Twist);
    cmd = Twist();
    cmd.linear.x = 0;
    cmd.angular.z = -0.4;
    pub.publish(cmd);   
}

//main function of the node
void control(){
  //creating node handles for subscription/publishing and parameter server queries
  ros::NodeHandle node;
  ros::NodeHandle pnode("~"); //used to get private parameters ~x, ~y and ~th

  //subscribing to odometry and announcing published topics
ros::Subscriber sub_o = node.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",10, odometryReceived);
ros::Subscriber sub_l = node.subscribe<sensor_msgs::LaserScan>("base_scan",10, rangeReceived);
ros::Subscriber sub_i = node.subscribe<sensor_msgs::Image>("image",10, imageReceived);

ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
  
ros::Rate r(10); 

	float scan_th = 0;
    const static float MIN_SCAN_ANGLE = -59.5*M_PI /180;
    const static float MAX_SCAN_ANGLE = +59.5*M_PI /180;
    float red, green, blue, h, s, v, min_range, max_range, max, min ;
    float max_laser_range = 1.5;
	float dist_front = 1.5;
	float dist_left = 1.5;
	float dist_right = 1.5;
	float dist_r_45 = 1.5;
	float dist_l_45 = 1.5;
    float laser_scans = 13;
	
  geometry_msgs::Twist cmd; //command that will be sent to Stage (published)
  
  while(ros::ok())
  {
    ros::spinOnce();

    if (!lastImageReading || !lastOdomReading || !lastImageReading){
      std::cout<<"waiting for lastImageReading to become available"<<std::endl;
      r.sleep();
      continue;
    }
    geometry_msgs::Pose pose = lastOdomReading->pose.pose;
    double th = tf::getYaw(pose.orientation);
    ROS_INFO("Goal:  th=%f", scan_th);
	
	 if(image != False and stop_before_obstacle == True)
	 {
            ROS_INFO("Red light ahead");
            break
	 }
        
        if((colider_array[0] == True or colider_array[1]== True or colider_array[2]== True and colider_array[3]== True and colider_array[4]==True ) and stop_before_obstacle == True ):
            ROS_INFO("Obstacle ahead : STOP")
            break


        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and math.fabs(dth) < 0.10:
            # if we are acceptably close to the goal, we can exit
            ROS_INFO('Destination Reached !!')
            break

        if math.fabs(dx) < 0.01 and math.fabs(dy) < 0.01 and disable_goal_th == True:
           ROS_INFO('Destination Reached !!')
           ROS_INFO('No Orientation corrections were applied upon arrival !!')
            break


	        if(image != False and stop_before_obstacle == True):
           ROS_INFO("Red light ahead")
            break
            
        
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
           ROS_INFO('F=t R=t L =t  GO BACK LEFT')
            
            
        if(front_colider == False and right_colider == False and left_colider == False):
            rospy.loginfo('Clear way - go to goal')
            gotogoal(x+3,y+3,goal_th,True,True) # GO to 45 deg bearing 
        
        if (front_colider == False and right_colider == True and left_colider == True):
            
            GoForwardCMD()
           ROS_INFO('WALLs on both right and left side => going straight')
            
        
        if(front_colider == True and right_colider == True and left_colider == True):
            
            GoBackLEFT()
            ROS_INFO('CORNER - turn arround')
        
        if(front_colider == False and right_colider == False and left_colider == False and r45_colider == True and l45_colider == False):
            GoBackRIGHT()
            
            
        if(front_colider == False and right_colider == False and left_colider == False and r45_colider == False and l45_colider == True):
            GoBackLEFT()
            

         // cmd.linear.x = 0.5; 
         // cmd.angular.z = 0.2;
  

    pub.publish(cmd);


    //sleeping so, that the loop won't run faster than r's frequency
    r.sleep();
  }
}

//entry point of the executable
int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  control();
  return 0;
}
