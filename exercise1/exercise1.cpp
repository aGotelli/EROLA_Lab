#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
/*If needed, add some headers

*/


#define thr 0.1 //Threshold for position control


ros::Publisher pub; //Publisher for sending velocity commands
float xt,yt;

float dist(float x, float y, float xt, float yt){
	//Function to retrieve the distance between two points in space
	return (sqrt(pow(x-xt,2)+pow(y-yt,2)));
	}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Callback thread on the received position
	float x, y;
	geometry_msgs::Twist vel;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	//If the robot is close enough to the target, ask for a new target
	if (dist(x,y,xt,yt)<thr){
		/*Ask for a new target

		*/
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
	/*publish the velocity

	*/

}


int main (int argc, char **argv){
	//Main function for the robot contro
	ros::init(argc, argv, "exercise1");
	ros::NodeHandle nh;
	// Retrieving the initial position target.
	nh.getParam("/exercise1/xt", xt);
	nh.getParam("/exercise1/yt", yt);
	/* Define a client for asking a new target, a publisher for publishing the velocity, and a subscriber for reading the position

	*/
	
	ros::spin();
	return 0;
}
