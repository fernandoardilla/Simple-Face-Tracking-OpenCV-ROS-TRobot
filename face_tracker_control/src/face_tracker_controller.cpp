/*
Value to dynamixel controller

Center = 0  // Dynamixel value = 512
Right = 1   // Dynamixel value = 708
Left = -1   // Dyanmixel value = 316

Differenece is 196 unit from center in Dynamixel
Optimum range = -0.5 to 0.5
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <iostream>

#include "face_tracker_pkg/centroid.h"

int servomaxx, servomin,screenmaxx=640, center_offset=100, center_left, center_right;
float servo_step_distancex, current_pos_x;

class SubPub{
public:
  SubPub()
  {
    //Topic you want to publish
    //pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    pub = node_obj.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    //Topic you want to subscribe
    //sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    number_subscriber = node_obj.subscribe("/face_centroid",10,&SubPub::callback, this); 
  }

  void callback(const face_tracker_pkg::centroid::ConstPtr& msg)
  {
    //PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    
    int x= msg->x;
    int y= msg->y;

    center_left = (screenmaxx / 2) - center_offset;
    center_right = (screenmaxx / 2) + center_offset;

    geometry_msgs::Twist v_msg;

    v_msg.linear.x=0.0;

    if(x < (center_left))	{ ROS_INFO("Face is at Left");  v_msg.angular.z=0.45; }
    else if(x > center_right)	{ ROS_INFO("Face is at Right");	v_msg.angular.z=-0.45;}
    else if(x > center_left and x < center_right){ ROS_INFO("Face is at Center"); v_msg.angular.z=0.0;  }
   
    if( y > 60 ) { v_msg.linear.x = 0.0; }
    else    	 { v_msg.linear.x = 0.35; }

    ROS_INFO("x:%.2f",v_msg.linear.x);
    pub.publish(v_msg);
  }

private:
  ros::NodeHandle node_obj; 
  ros::Publisher pub;
  ros::Subscriber number_subscriber;
};

int main(int argc, char **argv)
{
  //Initiate ROS
   ros::init(argc, argv,"face_tracker_controller");
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPub SAPObject;
  ros::spin();
  return 0;
}



/*

ros::Publisher pub;

void track_face(int x,int y)
{	
	if(x < (center_left))		{ ROS_INFO("Face is at Left");    }
    	else if(x > center_right)	{ ROS_INFO("Face is at Right");   }
	else if(x > center_left and x < center_right){ ROS_INFO("Face is at Center");  }

//	geometry_msgs::Twist msg;
//	msg.linear.x=0.5;
//      msg.angular.z=0.2;
//	pub.publish(msg);
//	ROS_INFO("x:%.2f",msg.linear.x);
}
//Callback of the topic /numbers
void face_callback(const face_tracker_pkg::centroid::ConstPtr& msg)
{
	ROS_INFO("Recieved X = [%d], Y = [%d]",msg->x,msg->y);
	//Calling track face function
	track_face(msg->x,msg->y);
}

int main(int argc, char **argv)
{
	//Loading servo configurations of the dynamixel tracker
	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"face_tracker_controller");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber number_subscriber = node_obj.subscribe("/face_centroid",10,face_callback);
	//pub = node_obj.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	servomaxx = 0.5;   //max degree servo horizontal (x) can turn
	servomin = -0.5;
	screenmaxx = 640;   //max screen horizontal (x)resolution
	center_offset = 100;
	//servo_step_distancex = 0.005; //x servo rotation steps
	//current_pos_x =0 ;

	center_left = (screenmaxx / 2) - center_offset;
	center_right = (screenmaxx / 2) + center_offset;;

	//Spinning the node
	ros::spin();
	return 0;
}
*/

