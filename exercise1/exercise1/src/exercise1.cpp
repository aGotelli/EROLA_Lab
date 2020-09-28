/**
 * \file exercise1.cpp
 * \brief Executable proving function to move the "robot"
 * \author Andrea Gotelli
 * \version 0.1
 * \date 27/09/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° /
 *
 * Publishes to: <BR>
 *    ° /
 *
 * Description
          This node provides the function to make it possible to move a "robot" thowards the
        target.
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <exercise1_service/rand_pose.h>


#define thr 0.1 ///< Threshold for position control


ros::Publisher pub; ///< Publisher for sending velocity commands
float xt ,yt;        /*!< initial position for the target */
ros::ServiceClient new_target_provider;

/*!
 * \brief dist computes the distance between two points in space
 * \param x x-coordinate of point 1
 * \param y y-coordinate of point 1
 * \param xt x-coordinate of point 2
 * \param yt y-coordinate of point 2
 * \return
 */
double dist(float x, float y, float xt, float yt){
	//Function to retrieve the distance between two points in space
	return (sqrt(pow(x-xt,2)+pow(y-yt,2)));
	}

/*!
 * \brief odomCallback callback function of the odometry message
 *
 * It compares the robot position to the target position. Using a proportional control law,
 * it computes the velocity command to send to the robot. In the case that the robot is very close
 * to the target, aks for another target position (on demand)
 *
 * \param msg the recived robot pose
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Callback thread on the received position
  double x, y;
	geometry_msgs::Twist vel;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	//If the robot is close enough to the target, ask for a new target
	if (dist(x,y,xt,yt)<thr){

    exercise1_service::rand_pose target;

    if(new_target_provider.call(target)) {
      xt = target.response.x;
      yt = target.response.y;
    }

    ROS_INFO_STREAM("The new target is at x = " << xt << "; y = " << yt );
	}
	//Simple proportional position control
	else{
		
    if(fabs(x-xt)>thr/2){
      vel.linear.x=-(x-xt);
		}
		if(fabs(y-yt)>thr/2){
      vel.linear.y=-(y-yt);
		}
	}

  pub.publish(vel);

}


int main (int argc, char **argv){
	//Main function for the robot contro
	ros::init(argc, argv, "exercise1");
	ros::NodeHandle nh;
	// Retrieving the initial position target.
	nh.getParam("/exercise1/xt", xt);
	nh.getParam("/exercise1/yt", yt);

  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);

  new_target_provider = nh.serviceClient<exercise1_service::rand_pose>("/RandomTarget");

  ROS_INFO_STREAM("Tracking target at x = " << xt << "; y = " << yt );
	
	ros::spin();
	return 0;
}
