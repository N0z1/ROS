#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include <stdlib.h>
#include <time.h>       /* time */
#include <math.h> /* M_PI */

#define LINEAR_SPEED 0.3 // m/s
#define ANGULAR_SPEED 0.7 // rad/s
#define OBJECT_DIST_NEAR 30 // cm
#define OBJECT_DIST_SAFE 50 // cm
#define WHEEL_R 0.07 //r, metres
#define TRACK_L 0.36 //L, metres
#define ENC_TICKS_RAD 644/(2*M_PI) // encoder ticks per wheel rotation rad

enum STATE { FORWARD, REVERSE, TURN }; // FSM states for the robot

class ros_wall_traveller
{
  public:
    ros_wall_traveller(); //constructor method
  private:
    //callback functions to attach to subscribed topics
    void leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg); // left encoder
    void rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg); // right encoder
    void midsonarCallback(const std_msgs::Int16::ConstPtr& msg); // middle sonar
    void leftsonarCallback(const std_msgs::Int16::ConstPtr& msg); // left sonar
    void rightsonarCallback(const std_msgs::Int16::ConstPtr& msg); // right sonar
    void FSM();// Finite State Machine control robots behaviour based on sensor data
	//Node Variables
	STATE state; //robot's FSM state
    ros::NodeHandle nh; //ROS node handler
    ros::Publisher vel_pub; //publisher for /cmd_vel
	geometry_msgs::Twist vel_msg; //Twist message to publish
    ros::Subscriber midsonar_sub; //subscriber for central sonar
    ros::Subscriber leftsonar_sub; // subsrciber for left sonar
    ros::Subscriber rightsonar_sub; // subscriber for right sonar
    ros::Subscriber left_encoder_sub; //subscriber for left encoder
    ros::Subscriber right_encoder_sub; //subscriber for right encoder

	//encoder count handling
    long int left_count, right_count, old_left_count, old_right_count;
	// Function to calculates rotated angle
	float Rotation_rad(); 
	float alpha; // Target angle
	float theta; // current rotation angle
	int L, M, R; // Variables to store sonar readings
};

// Main function
int main(int argc, char **argv)
{
  //initialize ROS node
  ros::init(argc, argv, "Wall_and_Avoid"); // outline node name
  //Create the object that contains node behaviour
  ros_wall_traveller Wall_and_Avoid; //this calls the constructor method
    //keep ROS updating the callback functions until node is killed
  ros::spin();
}

//Constructor. ROS initializations are done here.
ros_wall_traveller::ros_wall_traveller()
{
  //subscribe to topics
  left_encoder_sub = nh.subscribe<std_msgs::Int64>("/arduino/encoder_left_value", 10, &ros_wall_traveller::leftEncoderCallback, this);
  right_encoder_sub = nh.subscribe<std_msgs::Int64>("/arduino/encoder_right_value", 10, &ros_wall_traveller::rightEncoderCallback, this);
  midsonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10, &ros_wall_traveller::midsonarCallback, this);
  leftsonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_3", 10, &ros_wall_traveller::leftsonarCallback, this);
  rightsonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_1", 10, &ros_wall_traveller::rightsonarCallback, this); 
  //advertise topics to publish
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  //init auxiliary variables
  state = FORWARD;  //initial robot state
  left_count = 0; // initial variable value
  right_count = 0; // initial variable value
  theta = 0; // initial variable angle
  alpha = 0; // initial varriable angle
  
}

//Callback function attached to the left encoder topic
void ros_wall_traveller::leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg)
{
  left_count = msg->data; //update left encoder pulse count
}
//Callback function attached to the right encoder topic
void ros_wall_traveller::rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg)
{
  right_count = msg->data; //update right encoder pulse count
}

//The sonar callback function is used to update the Finite State Machine (FSM)
void ros_wall_traveller::midsonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
  M = msg->data; //message data for midsonarCallback
  	//evaluate FSM for using midsonarCallback
	FSM();
}
  
void ros_wall_traveller::leftsonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
	L = msg->data; // data from the leftsonarCallback
	//evaluate FSM for using leftsonarCallback	
         FSM();
}
// sonar callback added for future implmentation in desinging a maze following robot
void ros_wall_traveller::rightsonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
	R = msg->data; //message data for rightsonarCallback
	//evaluate FSM for using rightsonarCallback
	FSM();
}
/* Finite State Machine(FSM */
void ros_wall_traveller::FSM() // function responsible for managing state stransitions of the robot based on sensor inputs
{
switch(state)
{
   case FORWARD: //currently moving forward
     if((M>0)&&(M<OBJECT_DIST_NEAR)) //if an obstacle is detected in close proximity, it should reverse
     {
      ROS_INFO("REVERSE"); // log the state change to backwards
      state = REVERSE; //change state to reverse
      vel_msg.linear.x = 0.0; //negate liner movement
      vel_msg.angular.z = 0.0; //negate angular movement
      vel_pub.publish(vel_msg); //publish the stop command
     }
 //following conditions adjust robots speed and direction based on the distance to the wall
     else if(L==0) // message data is either inf or 0 so no valid data
     {
       ROS_INFO("Dist %d cm",L); // log message data of the left sonar
       vel_msg.linear.x = LINEAR_SPEED; // Function used to give a standalone linear speed//
       vel_msg.angular.z = 0.75 * ANGULAR_SPEED; // positive value multiply to add anti clockwise rotation
       vel_pub.publish(vel_msg); // publish velcity
     } 
    else if(L<100 && L>50) // message data reads close to the wall
    {
      ROS_INFO("Dist %d cm",L); // log message data of the left sonar
      vel_msg.linear.x = LINEAR_SPEED;// function set linear speed
      vel_msg.angular.z= (0.0175*L)-0.875; // adjusted angular speed using y=mx+c
       vel_pub.publish(vel_msg); // publish
    }
    else if(L<50 && L>30) // 
    {
      ROS_INFO("Dist %d cm",L);// log message data of the left sonar
      vel_msg.linear.x = (0.015*L)-0.45;
      vel_msg.angular.z= (0.0175*L)-0.875;// based on y=mx+c to find the 
      vel_pub.publish(vel_msg);
    }
  else if(L<30 && L>0) // minimum proximity to the wall
    {
      ROS_INFO("Dist %d cm",L); // log distance to the wall
      vel_msg.linear.x = 0.0; // negates linear movement
      vel_msg.angular.z= (0.0175*L)-0.875;//baseed on y=mx+c
      vel_pub.publish(vel_msg); //publish the velocity
    }
     break;
   case REVERSE: //currently reversing
     if(M == 0) break; // if no valid detection, break
     else if(M>OBJECT_DIST_SAFE) // if obstacle is far enough the robot should turn
     {
      ROS_INFO("TURN"); //log the state change to TURN
      state = TURN; //change state to turn
      old_left_count = left_count; // updated count reading for the left encoder
      old_right_count = right_count; // updated count reading for the right encoder
	
      theta = Rotation_rad(); // get robots current rotation
      alpha = theta  - M_PI/2; // set the turn target based on the robotscurrent rotation
     }
     else // Otherwise, keep moving backward until the robot reaches a safe distance from the obstacler
     {
      ROS_INFO("Backward"); // log the state change to backwards
	vel_msg.linear.x = -LINEAR_SPEED; // set predefined linear speed to negative to move robot backwards
	vel_msg.angular.z = 0.0; // set angular velocity to 0.0 to negate any rotation.
	vel_pub.publish(vel_msg); // publish the velocity command
     }
     break;
   case TURN: // robot is currently turning
      theta = Rotation_rad(); // get the robots current rotation
      if((M>0)&&(M<OBJECT_DIST_NEAR)) // if wall is detected close to the robot, the robot needs to reverse
       {
        ROS_INFO("REVERSE");// Log the state change to Reverse
        state = REVERSE; //change state to reverse
        vel_msg.linear.x = 0.0;// negate linear movement
        vel_msg.angular.z = 0.0; // negate angular movement
        vel_pub.publish(vel_msg); // stop movement
     }
     else if(theta < alpha) // If the angle is decreasing, the turn is complete
     {
      state = FORWARD; // Change the state to FORWARD
      ROS_INFO("FORWARD"); // Log the state change to FORWARD
      vel_msg.linear.x = 0.0; //set to zero so that stops movement
      vel_msg.angular.z = 0.0; // set to zero for angular velocity i.e turn
      vel_pub.publish(vel_msg); //publish
     }
     else //keep rotating
     {
      ROS_INFO("Rotated %.1f, target %.1f",theta,alpha);
      vel_msg.linear.x = 0.0; // set 0.0 to negate linear motion(forwards & Backwards)
      vel_msg.angular.z = -ANGULAR_SPEED; // Set the angular speed to turn the robot right
      vel_pub.publish(vel_msg); // Publish the velocity command
     }
	 break;
  }//end switch
}//end sonarCallback


// encoder readings calculates the rotated angle
//using the differential drive model
float ros_wall_traveller::Rotation_rad()
{
	float angle;
	int increment_l = left_count - old_left_count; // Calculate the change in the left encoder count
	int increment_r = right_count - old_right_count; // Calculate the change in the right encoder count
	angle = (WHEEL_R*(increment_r - increment_l)/(TRACK_L * ENC_TICKS_RAD))*1.25; // Calculate the rotated angle and apply a scalar to minimize the robot turning more than the target
	return angle;
}
