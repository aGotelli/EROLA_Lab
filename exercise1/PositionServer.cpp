#include "ros/ros.h"
/*If needed, add some headers

*/


#define min_x -3.0
#define max_x 7.0
#define min_y -4.0
#define max_y 6.0



float RandomFloat(float min, float max)
{
    //Function to calcualate a random float between a min and a max
    assert(max > min); 
    float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;  
    return (random*range) + min;
}



/*Add the callback thread of the service. The service returns two random floats between min and max. 
You can use the RandomFloat function



*/




int main (int argc, char **argv){
	//Main function for the position server

	ros::init(argc, argv, "position_server");
	ros::NodeHandle nh;
	
	/* Define a ROS Server

	*/
	
	ros::spin();
	return 0;
}
