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
#define ENC_TICKS_RAD 644/(2*M_PI) //encoder ticks per wheel rotation rad

enum STATE { FORWARD, REVERSE, TURN }; //possible FSM states

class ros_wall_traveller
{
  public:
    ros_wall_traveller(); //constructor method
  private:
    //callback functions to attach to subscribed topics
    void leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void midsonarCallback(const std_msgs::Int16::ConstPtr& msg); // middle sonar
    void leftsonarCallback(const std_msgs::Int16::ConstPtr& msg); // left sonar
    void rightsonarCallback(const std_msgs::Int16::ConstPtr& msg); // right sonar
    void FSM();// Finite state Machine
	//node variables
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
	float Rotation_rad(); //calculates rotated angle
	float alpha; // angel that is targetted
	float theta; // current rotation angle
	int L, M, R;
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
  left_count = 0; 
  right_count = 0;
  theta = 0;
  alpha = 0;
  
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
  M = msg->data; //update sonar distance with the value from subscription
  FSM();
}
  

void ros_wall_traveller::leftsonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
	L = msg->data;
  FSM();//evaluate FSM	
}

void ros_wall_traveller::rightsonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
	R = msg->data; /*message data for rightsonarCallback*/
//evaluate FSM
	FSM();
}
/* Finite State Machine */
void ros_wall_traveller::FSM()
{
switch(state)
{
   case FORWARD: //currently moving forward
     if((M>0)&&(M<OBJECT_DIST_NEAR)) //obstacle close, go reverse!
     {
      ROS_INFO("REVERSE");
      state = REVERSE; //change state to reverse
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg); //stop
     }
     else if(L==0)
     {
	     ROS_INFO("Dist %d cm",L);
       vel_msg.linear.x = LINEAR_SPEED; // Function used for give a standalone linear speed//
       vel_msg.angular.z = 0.75 * ANGULAR_SPEED; // positive value multiply to add anti clockwise rotation
       vel_pub.publish(vel_msg); // publish cord;
     } 
    else if(L<100 && L>50) /*change when i can*/
    {
      ROS_INFO("Dist %d cm",L);
      vel_msg.linear.x = LINEAR_SPEED;
      vel_msg.angular.z= (0.0175*L)-0.875;
       vel_pub.publish(vel_msg);
    }
    else if(L<50 && L>30) // 
    {
      ROS_INFO("Dist %d cm",L);
      vel_msg.linear.x = (0.015*L)-0.45;
      vel_msg.angular.z= (0.0175*L)-0.875;
      vel_pub.publish(vel_msg);
    }
  else if(L<30 && L>0)
    {
      ROS_INFO("Dist %d cm",L);
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z= (0.0175*L)-0.875;
      vel_pub.publish(vel_msg);
    }
     break;
   	case REVERSE: //currently reversing
     if(M == 0) break; //no valid detection
     else if(M>OBJECT_DIST_SAFE) //obstacle away, now turn
     {
      ROS_INFO("TURN");
      state = TURN;
      old_left_count = left_count;
      old_right_count = right_count;
	
      theta = Rotation_rad(); // get robots current rotation
      alpha = theta  - M_PI/2; // sets the turn target based on current rotation in line
     }
     else // otherwise keep moving backward until reaches target distamce read by the middle sensor
     {
      ROS_INFO("Backward");
	vel_msg.linear.x = -LINEAR_SPEED; // Setting Linear Speed will Move the Robot Backwards
	vel_msg.angular.z = 0.0; // set angular velocity to 0.0 to negate any rotation.
	vel_pub.publish(vel_msg);
     }
     break;
   case TURN:
      theta = Rotation_rad();
      if((M>0)&&(M<OBJECT_DIST_NEAR)) //obstacle is close, so reverse
       {
        ROS_INFO("REVERSE");
        state = REVERSE; //change state to reverse
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        vel_pub.publish(vel_msg); //stop
     }
     else if(theta < alpha) //the angle is decreasing so turn is complete
     {
      state = FORWARD;
      ROS_INFO("FORWARD");
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg); //stop
     }
     else //keep rotating
     {
      ROS_INFO("Rotated %.1f, target %.1f",theta,alpha);
      vel_msg.linear.x = 0.0; // set 0.0 to negate linear motion(foraward & Backwards)
      vel_msg.angular.z = -ANGULAR_SPEED; // force robot to turn RIGHT
      vel_pub.publish(vel_msg); // 
     }
	 break;
  }//end switch
}//end sonarCallback


//calculates rotated angle from encoder readings
//using the differential drive model
float ros_wall_traveller::Rotation_rad()
{
	float angle;
	int increment_l = left_count - old_left_count;
	int increment_r = right_count - old_right_count;
	angle = (WHEEL_R*(increment_r - increment_l)/(TRACK_L * ENC_TICKS_RAD))*1.25; //multiplied by a scalar to minimize the robot turning more than target//
	return angle;
}
