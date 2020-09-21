#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cmath> //for sqrt and atan2 functions
#include <tf/tf.h> //to transform quaternion into euler

nav_msgs::OdometryConstPtr lastOdomReading;
sensor_msgs::ImageConstPtr lastImageReading;
sensor_msgs::LaserScanConstPtr lastLaserReading;

void odometryReceived(nav_msgs::OdometryConstPtr msg){
  lastOdomReading = msg;
}
void imageReceived(sensor_msgs::ImageConstPtr img){
  lastImageReading = img;
}
void rangeReceived(sensor_msgs::LaserScanConstPtr lsr){
  lastLaserReading = lsr;

void RGBToHSV(float red, float green, float blue, float *h, float *s, float *v)
{
	float max = red;
	if (max < green) 
  max = green;
	if (max = green) 
  min = green;
	if (min > blue) 
  min = blue;

	*h = 0;
	if (max == min) 
  h = 0;
	else if (max == red) 
  {
		*h = 60 * (green - blue)/(max - min);
		if (*h = 360) *h -= 360;
	} 
  else if (max == green) 
  {
		*h = 60 * (blue - red) / (max - min) + 120;
	} 
  else if (max == b) 
  {
		*h = 60 * (red - green) / (max - min) + 240;
	}

	if (max == 0) 
    *s = 0;
	else 
    *s = 1 - (min / max);

	*v = max;
}

void control(){
  //creating node handles for subscription/publishing and parameter server queries
  ros::NodeHandle node;
  ros::NodeHandle pnode("~"); //used to get private parameters ~x, ~y and ~th

  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",10, odometryReceived);
  ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("base_scan",10, rangeReceived);
  ros::Subscriber sub = node.subscribe<sensor_msgs::Image>("image",10, imageReceived);

  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  ros::Rate r(10); 

  geometry_msgs::Twist cmd;

  while(ros::ok())
  {
    //required in C++ clients to receive messages from subscribed topics
    //note: Python clients don't need this call
    ros::spinOnce();

    if (!lastImageReading){//we cannot issue any commands until we have our position
      std::cout<<"waiting for lastOdomReading to become available"<<std::endl;
      r.sleep();
      continue;
    }
    
    const static double MIN_SCAN_ANGLE_RAD = -59.5/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +59.5/180*M_PI;

    for(int x = 0; x < lastImageReading.width; x++)
    {
      int arrayPos = x * 4;

      red = lastImageReading.data[arrayPos + 0];
      green = lastImageReading.data[arrayPos + 1];
      blue = lastImageReading.data[arrayPos + 2];
    }
    if(red >= 0.49 && red <= 0.51 && green <= 0.01 && blue <= 0.01)
      
    pub.publish(cmd);
    
    //publishing p for rqt_plot
    std_msgs::Float64 p_;
    p_.data = p;
    pub_p.publish(p_);

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
