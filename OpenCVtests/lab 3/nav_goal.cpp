#include <math.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include<geometry_msgs/Vector3.h>

#define DONT_CARE_ORIENT 777 //this value means any orientation valid
#define POS_TOLERANCE 0.1 //m
#define ORIENT_TOLERANCE 0.1 //rad
#define MAX_LIN_SPEED 0.5
#define MAX_ANG_SPEED 1.5

class MoveToGoal
{
  public:
    MoveToGoal(); //constructor
  private:
    void loop(); //Infinite loop until ROS is shut down
    void move(); //Moves towards goal
    bool navigating; //indicates if we're moving to a target
    bool target_reached; //indicates if we're reached position
    ros::NodeHandle nh;
    //callbacks for topics
    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    //publishers
    ros::Publisher vel_pub;
    geometry_msgs::Twist vel_msg; //vel message
    //to use tf odom<->base_link
    tf::TransformListener tl_o_b;
    //to store goal pose in odom frame
    tf::Vector3 goal_position_odom;
    float goal_theta_odom;
};
//---------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_goal");
  //ros::NodeHandle nh;
  //Creates object. This calls its constructor that starts it all
  MoveToGoal moveGoal;
}
//------- constructor -----------------------
MoveToGoal::MoveToGoal()
{
  //Attach subscription callback functions
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Pose2D>("/nav_goal", 10, &MoveToGoal::goalCallback, this);
  //prepare publishers
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  navigating = false;
  target_reached = false;
  //Infinite loop running until ROS is shut down
  loop();
}
//---- Infinite timed loop doing the node task ----------
void MoveToGoal::loop()
{
  ros::Rate r(10.0); //loop at 10Hz (every 100ms)
  while(nh.ok()){
    ros::spinOnce();
    move(); //move towards goal pose (if any)
    r.sleep();
  }
}

//Reads new local goal pose (relative to robot frame) and converts it to odom frame
//Cancels navigation if a new goal is received while still moving to a previous one
//Getting DONT_CARE_ORIENT in theta means "any final orientation"
void MoveToGoal::goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  if (navigating) { //cancel current navigation
    ROS_INFO("NavigationAborted");
    navigating = false;
    target_reached = false;
  }
  else{ //Pose2D has x,y,theta
    ROS_INFO("NavigationStart");
    navigating = true; //start navigation to goal pose
    tf::StampedTransform T_Odom_Robot; //T_Odom_Robot is ROS version of homogeneous transf. matrix
    //Get a Transform object with last /base_link pose (child) in /odom frame (parent):
    tl_o_b.lookupTransform("odom","base_link",ros::Time(0),T_Odom_Robot);
    //Convert goal pose to my_odom (global) frame 
    tf::Vector3 P_robot(msg->x,msg->y,0.0); //create 3D point
    goal_position_odom = T_Odom_Robot * P_robot; //Transform position from robot to odom
   //handle destination orientation now
   goal_theta_odom = DONT_CARE_ORIENT; //any orientation is ok
    if(msg->theta != DONT_CARE_ORIENT) //required a given final orientation instead
	{ //current orientation plus new one relative 
          tf::Quaternion q = T_Odom_Robot.getRotation();
	  float theta = 2*atan2(q.z(),q.w()); //current robot's theta -pi,pi range
	  goal_theta_odom = theta + msg->theta;
	    //Keep angle in -pi,pi range
            if (goal_theta_odom > M_PI) goal_theta_odom -=(2.0*M_PI);
            else if (goal_theta_odom < -M_PI) goal_theta_odom += (2.0*M_PI);
	}
  }
}

//--- Make robot move towards goal pose (stored in odom frame)
//Getting DONT_CARE_ORIENT in theta means "any final orientation"
void MoveToGoal::move()
{
  if (!navigating) return; //no goal given yet, just return

  //Get a tf object with last /base_link pose (child) in /odom frame (parent)
  //T_Odom_Robot is the ROS representation of a homogeneous transformation matrix
  tf::StampedTransform T_Odom_Robot;
  tl_o_b.lookupTransform("odom","base_link",ros::Time(0),T_Odom_Robot);
  // 1) EXPRESS ODOM GOAL POSE goal_position_odom IN CURRENT ROBOT'S FRAME
  tf::Vector3 P_robot; //Transformed position from odom to current robot
	P_robot = T_Odom_Robot.inverse() * goal_position_odom;//COORDINATE CONVERSION FROM goal_position_odom TO P_robot
    //USING T_Odom_Robot
  //target orientation in robot frame; DONT_CARE_ORIENT means any
  float theta, goal_theta_error;
  if(goal_theta_odom != DONT_CARE_ORIENT){ //required orientation instead
          tf::Quaternion q = T_Odom_Robot.getRotation();
	  float theta = 2*atan2(q.z(),q.w()); //current robot's theta -pi,pi range
	  goal_theta_error = goal_theta_odom - theta;
	    //map goal orientation to -pi/+pi range 
         if (goal_theta_error > M_PI) goal_theta_error -=(2.0*M_PI);
         else if (goal_theta_error < -M_PI) goal_theta_error += (2.0*M_PI);
  }
  // 2) LOCAL PLANNER
  // USE RELATIVE POSE TO CALCULATE ROBOT'S SPEEDS
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  double error_dist = sqrt(P_robot.x() * P_robot.x() + P_robot.y() * P_robot.y()); //distance to goal (m)
  double error_beta = atan2(P_robot.y(), P_robot.x()); //angle towards goal, -pi to pi
  if(target_reached == false)//REACH POSITION FIRST
  {
    if(fabs(error_beta) > ORIENT_TOLERANCE){//firstly rotate to point to the target
       vel_msg.angular.z = MAX_ANG_SPEED * error_beta; //P control
       ROS_INFO("Pointing to target, angle error %0.1f", error_beta);
    }	  
    else if(error_dist > POS_TOLERANCE){ //not in target position
	double k_p_lin = 0.5; // Linear proportional gain
	double max_lin_speed = MAX_LIN_SPEED;//sets the maximum allowed linear speed to the defined constant MAX_LIN_SPEED.
	vel_msg.linear.x = k_p_lin * error_dist;//calculates the linear velocity using proportional control

    	vel_msg.linear.x = std::min(vel_msg.linear.x, max_lin_speed);//ensures that the linear velocity does not exceed the maximum allowed valu 

    	// Calculate angular velocity using propotional control

    	double k_p_ang = 1.0; // Angular proportional gain

    	double max_ang_speed = MAX_ANG_SPEED;//sets the maximum allowed angular speed to the defined constant MAX_ANG_SPEED

    	double ang_vel = k_p_ang * error_beta;//calculates the angular velocity using proportional control

    	vel_msg.angular.z = std::min(std::max(ang_vel, -max_ang_speed), max_ang_speed);//nsures that the angular velocity is within the allowed range

    	ROS_INFO("Travelling to target, dist error %0.1f, angle error %0.1f", error_dist, error_beta);   
         }
         
	 else target_reached = true; //reached goal position
  }
  else //already in target position, now check final orientation
  { 
      if((goal_theta_odom == DONT_CARE_ORIENT)||(fabs(goal_theta_error) < ORIENT_TOLERANCE))
      { //ALL DONE!
        navigating = false;
        target_reached = false;
        ROS_INFO("NavigationCompleted!");		 
      }
      else //rotate to meet orientation
      { 
        vel_msg.angular.z = (MAX_ANG_SPEED/2.0) * goal_theta_error; //P control
	    ROS_INFO("Rotating. Error %.1f", goal_theta_error);		 
      }
  }
  //publish calculated velocity command
  vel_pub.publish(vel_msg);
}



