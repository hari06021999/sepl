/*********************************************************************************************************
*this node subscribes to (joy) topic from the TeleopeTurtle node (one of ROS example packages)          *
*then publish it as the standerd Twist message on topic (/cmd_vel),                                     * 
(this is the same topic of the teleope twist keboard)                                                   *
********************************************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <stdint.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <stdbool.h>
#include <unistd.h>


 
	
class RemoteTeleop {

    private:
   
    ros::Publisher pub_light;
    ros::Publisher pub_multicolor;
    ros::Publisher pub;
    ros::Publisher stop_flag;
    ros::Subscriber joystick_subscriber;
    ros::Subscriber button1_subscriber;
    ros::Subscriber button2_subscriber;
   
      int count_1=0;
      int count_2=0;
      int count_3=0;
      int flag=0;
      float l=0;
      float r=0;
      uint16_t lPwm = 0;
      uint16_t rPwm = 0;
    public:
  
        
     
      RemoteTeleop(ros::NodeHandle *nh) {
        
	pub_light = nh->advertise<std_msgs::Bool>("front/light",10);
        pub_multicolor = nh->advertise<std_msgs::Int16>("front/light/multi",10);
        pub = nh->advertise<geometry_msgs::Twist>("/linear/angular", 100); 
        stop_flag= nh-> advertise<std_msgs::Bool>("/stop/flag",2);  
        joystick_subscriber = nh->subscribe("/cmd_vel", 2, &RemoteTeleop::callback_joy, this);
        button1_subscriber = nh->subscribe("btn1topic", 2, &RemoteTeleop::callback_button1, this);
	button2_subscriber = nh->subscribe("btn2topic", 2, &RemoteTeleop::callback_button2, this);
	float mapPwm(float x, float out_min, float out_max);
       
           
    }
    float mapPwm(float x, float out_min, float out_max)
    {
       return x * (out_max - out_min) + out_min;
    }
    void callback_joy(const geometry_msgs::Twist& msg) {

       
		
 	ROS_INFO("Linear:%lf , Angular:%lf\n",msg.linear.x,msg.angular.z);

  if(msg.linear.x>0 || msg.linear.x<0)
  {
     pub.publish(msg);
     l = (msg.linear.x - msg.angular.z) / 2;
         r = (msg.linear.x + msg.angular.z) / 2;
   lPwm = mapPwm(fabs(l), 80,120);
         rPwm = mapPwm(fabs(r), 80,120);
  ROS_INFO("left PWM: %d \n",lPwm);
  ROS_INFO("right PWM: %d \n",rPwm);
  if(l<0)
    ROS_INFO("left reverse\n");
  else
    ROS_INFO("left forward\n");
  if(r<0)
    ROS_INFO("right reverse\n");
  else
    ROS_INFO("right forward\n");  
  }
  else  {
      pub.publish(msg);
    sleep(0.5);
    pub.publish(msg);
     ROS_INFO("STOP");
     
  }
         
    }
    
    
   void callback_button1(const std_msgs::Bool& msg) 
   {
        	
     bool light_1=false;
     std_msgs::Bool value;
      light_1= msg.data;
     if(light_1==true)
     {
       count_1++;
     } 
     if(count_1==1 && light_1==true)
     {
      light_1=true;
      value.data=true;
      pub_light.publish(value);
      ROS_INFO("Front top Light :%d",light_1);
     }
     if(count_1==2 && light_1==true)
     {
       
       light_1=false;
        value.data=false;
       pub_light.publish(value);
      ROS_INFO("Front top Light:%d",light_1);
      count_1=0;
     }
    
 
   }
   void callback_button2(const std_msgs::Bool& msg) 
   {
        	
     bool light_2=false;
     std_msgs::Int16 value;
      light_2= msg.data;
     if(light_2==true)
     {
       count_2++;
     } 
     if(count_2==1 && light_2==true)
     {
      light_2=true;
      value.data=1;
      pub_multicolor.publish(value);
      ROS_INFO("White color :%d",light_2);
     }
     if(count_2==2 && light_2==true)
     {
       
       light_2=true;
        value.data=2;
       pub_multicolor.publish(value);
      ROS_INFO("Yellow color :%d",light_2);
     
     }
    if(count_2==3 && light_2==true)
     {
       
       light_2=false;
        value.data=0;
       pub_multicolor.publish(value);
      ROS_INFO("Lights off:%d",light_2);
      count_2=0;
     }
   }  
};

int main (int argc, char **argv)
{ 
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    RemoteTeleop nc = RemoteTeleop(&nh);
    ros::spin();
}

