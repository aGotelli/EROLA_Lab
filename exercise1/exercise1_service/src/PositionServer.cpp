/**
 * \file PositionServer
 * \brief Executable proving the service for getting a 2D random pose
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
          This node has the only goal of providing a 2D random pose on demand.
         It uses a basic function, called twice, which returns a random number.
         The two random numbers fill up the service response.
 *
 */
#include "ros/ros.h"

#include "exercise1_service/rand_pose.h"

#define min_x -3.0  /*!< /brief Defines the minimum value for x */
#define max_x 7.0   /*!< /brief Defines the maximum value for x */
#define min_y -4.0  /*!< /brief Defines the minimum value for y */
#define max_y 6.0   /*!< /brief Defines the minimum value for y */


/*!
 * \brief RandomFloat gives a random number in a min-max range
 * \param min: lower bound for the random number
 * \param max: upper bound for the random number
 * \return returns the generated random number
 */
float RandomFloat(float min, float max)
{
    //Function to calcualate a random float between a min and a max
    assert(max > min); 
    float random = ( static_cast<float>(rand()) ) / static_cast<float>(RAND_MAX) ;
    float range = max - min;  
    return (random*range) + min;
}

/*!
 * \brief giveRandPose Service callback which fill the response with a random 2D position
 * \param res:  I/O parameter containing the response with the random 2D posetion
 * \return returns true always. this method cannot fail.
 */
bool giveRandPose(exercise1_service::rand_pose::Request&, exercise1_service::rand_pose::Response& res)
{
  res.x = RandomFloat(min_x, max_y);
  res.y = RandomFloat(min_y, max_y);

  return true;
}





int main (int argc, char **argv){
	//Main function for the position server

	ros::init(argc, argv, "position_server");
	ros::NodeHandle nh;

  ros::ServiceServer service_server = nh.advertiseService("/RandomTarget", giveRandPose);
	
	ros::spin();
	return 0;
}
