/*****************************************************
*
*  Homography decomposition function
*  Implemented by Benjamin Lewis, May 2015
*  Based on the paper "Deeper understanding of the homography decomposition for vision-based control"
*  Paper available at https://hal.inria.fr/inria-00174036v3/document
*   - Function: homogDecompose:
*		- Decompose a homography matrix H given a intrinsic parameter matrix K
*		- Return a geometeric rotation, translation, and planar normal, up to a scale factor. 
*	*** Helper Functions ***
*	- Function: rotation_angles:
*		- Helper function that decomposes a rotation matrix into Euler angles
*	- Function: normalize:
*		- Normalize a 3x1 vector
*	- Function: submatrix:
*		- Return a submatrix of a 3x3 matrix. Inputs are the row and column to be excluded (one-indexed).
*		- Returns a 2x2 submatrix.
*
*****************************************************/

#include <vector>
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include "../include/decompose.h"
#include <homography_calc/decomposition.h>

class homogDecompose
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	homogDecompose()	{
		//Topic you want to subscribe to
			
		std::string homographyTopic;
		if (ros::param::has("homogTopic") ){
			ros::param::get("homogTopic", homographyTopic);
		}else{
			homographyTopic = "homestHomog";
		}
		
		homogSub = n.subscribe(homographyTopic, 1, &homogDecompose::decompose, this);
		
		std::vector<double> K_vals;
		if (ros::param::has("cameraCalibrationVals") ){
			n.getParam("cameraCalibrationVals", K_vals);
		}else{
			ROS_ERROR("No K matrix has been set. Exiting.");
			exit(1);
		}
		
		std::cout << K_vals.size() << std::endl;
		if (K_vals.size() != 9){
			ROS_ERROR("K matrix is wrong size. Exiting.");
			exit(1);
		}
		
		K << K_vals[0], K_vals[1], K_vals[2],
			 K_vals[3], K_vals[4], K_vals[5], 
			 K_vals[6], K_vals[7], K_vals[8];
			 
		std::cout << K << std::endl;
			
		// TODO: Publish a message of translation and rotation.
		
		std::string decompositionPubTopic;
		if (ros::param::has("decompositionPubTopic") ){
			ros::param::get("decompositionPubTopic", decompositionPubTopic);
		}else{
			decompositionPubTopic = "homog_decomposition";
		}
				
		decompositionPub = n.advertise<homography_calc::decomposition>(decompositionPubTopic, 1);

	}
	
/******************************************************************
*************  Keypoint Calculation Callback  *********************
******************************************************************/

	void decompose(const std_msgs::Float64MultiArray& msg)
	{
    	Eigen::Matrix3d H; 
    	H << msg.data[0], msg.data[1], msg.data[2], 
    		 msg.data[3], msg.data[4], msg.data[5], 
    		 msg.data[6], msg.data[7], msg.data[8]; 
    	
    	decompose_return values;
    	values = computeHomographyDecomposition(H, K);
    	
    	angle_triple angles_a;
    	angle_triple angles_b;
    	
    	angles_a = rotation_angles(values.Ra);
    	angles_b = rotation_angles(values.Rb);
    	
    	homography_calc::decomposition controlMsg;
    	geometry_msgs::Vector3 Ta;
    	geometry_msgs::Vector3 Tb;
    	
    	Ta.x = values.ta(0,0);
    	Ta.y = values.ta(0,1);
    	Ta.z = values.ta(0,2);
    	
    	Tb.x = values.tb(0,0);
    	Tb.y = values.tb(0,1);
    	Tb.z = values.tb(0,2);
    	
    	controlMsg.Ta = Ta;
    	controlMsg.Tb = Tb;
    	
    	controlMsg.phi_a.data    = angles_a.phi_radians;
    	controlMsg.theta_a.data  = angles_a.theta_radians;
    	controlMsg.psi_a.data    = angles_a.psi_radians;
    	
       	controlMsg.phi_b.data    = angles_b.phi_radians;
    	controlMsg.theta_b.data  = angles_b.theta_radians;
    	controlMsg.psi_b.data    = angles_b.psi_radians;
    	
    	decompositionPub.publish(controlMsg);
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Subscriber homogSub;	
	
	Eigen::Matrix3d K;
	ros::Publisher decompositionPub;

};//End of class

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "homographyDecompose");

  //Create an object of class that will take care of everything
  homogDecompose homogDecomposeObject;

  ros::spin();

  return 0;
}

