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
#include <opencv2/opencv.hpp>

#include <std_msgs/Float64MultiArray.h>
#include "../include/homogControlLaw.h"
#include <homography_calc/controlLawInputs.h>
#include <geometry_msgs/Vector3.h>
#include <homography_calc/controlLawOutputs.h>
#include <std_msgs/Bool.h>

#define IN true
#define OUT false

class controlLaw
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	controlLaw()	{
		//Topic you want to subscribe to
		
		navigationDirection = OUT;
			
		std::string homographyControlSubTopic;
		if (ros::param::has("homogControlInputTopic") ){
			ros::param::get("homogControlInputTopic", homographyControlSubTopic);
		}else{
			homographyControlSubTopic = "homestHomogControlLawInputs";
		}
		
		homogSub = n.subscribe(homographyControlSubTopic, 1, &controlLaw::law, this);
		
		std::string directionChangeTopic;
		if (ros::param::has("directionChangeTopic") ){
			ros::param::get("directionChangeTopic", directionChangeTopic);
		}else{
			directionChangeTopic = "navChange";
		}
		
		directionSub = n.subscribe(directionChangeTopic, 1, &controlLaw::switchNavDirection, this);
			
		std::vector<double> K_vals;
		if (ros::param::has("cameraCalibrationVals") ){
			n.getParam("cameraCalibrationVals", K_vals);
		}else{
			ROS_ERROR("No K matrix has been set. Exiting.");
			exit(1);
		}
		
		if (K_vals.size() != 9){
			ROS_ERROR("K matrix is wrong size. Exiting.");
			exit(1);
		}
		
		K << K_vals[0], K_vals[1], K_vals[2],
			 K_vals[3], K_vals[4], K_vals[5], 
			 K_vals[6], K_vals[7], K_vals[8];
		
		std::string controlLawOutTopic;
		if (ros::param::has("controlLawOutTopic") ){
			ros::param::get("controlLawOutTopic", controlLawOutTopic);
		}else{
			controlLawOutTopic = "controlLawOut";
		}
				
		controlLawPub = n.advertise<homography_calc::controlLawOutputs>(controlLawOutTopic, 1);
	}
	
	void switchNavDirection(const std_msgs::Bool msg){
		if (msg.data == true){
			navigationDirection = !navigationDirection;
			ROS_INFO("Switched navigation direction in matchRawFeatures");
		}
	}
	
/******************************************************************
*************  Keypoint Calculation Callback  *********************
******************************************************************/

	void law(const homography_calc::controlLawInputs& msg)
	{
    	Eigen::Matrix3d H; 
    	H << msg.homography.data[0], msg.homography.data[1], msg.homography.data[2], 
    		 msg.homography.data[3], msg.homography.data[4], msg.homography.data[5], 
    		 msg.homography.data[6], msg.homography.data[7], msg.homography.data[8]; 
    		 
    	Eigen::MatrixXd currentCenterOfMass(3,1);
    	Eigen::MatrixXd targetCenterOfMass(3,1);
    	
    	currentCenterOfMass << msg.currentCenterOfMass.x, msg.currentCenterOfMass.y, msg.currentCenterOfMass.z;
    	targetCenterOfMass << msg.targetCenterOfMass.x, msg.targetCenterOfMass.y, msg.targetCenterOfMass.z;
    	
    	//Eigen::MatrixXd controlCompute(Eigen::Matrix3d H, Eigen::Matrix3d K, Eigen::MatrixXd currentCOM, Eigen::MatrixXd targetCOM, double weightT, double weightOmega);
    	// weightT and weightOmega currently aren't being used. 
    	command_return result;
    	result = controlCompute(H, K, currentCenterOfMass, targetCenterOfMass, 1, 1);
    	Eigen::MatrixXd translation = result.e_v;
    	Eigen::MatrixXd rotation_cross = result.e_wx;
    	
    	homography_calc::controlLawOutputs outMsg;
    	outMsg.translation.x = translation(0,0);
    	outMsg.translation.y = translation(0,1);
    	outMsg.translation.z = translation(0,2);
    	
    	outMsg.rotation_cross.x = rotation_cross(0,0);
    	outMsg.rotation_cross.y = rotation_cross(0,1);
    	outMsg.rotation_cross.z = rotation_cross(0,2);
    	
    	controlLawPub.publish(outMsg);
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Subscriber homogSub;	
	ros::Subscriber directionSub;
	ros::Subscriber homogControlTopicSub;
	ros::Publisher controlLawPub;
	
	int navigationDirection;
	Eigen::Matrix3d K;

};//End of class

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "homogControlLaw");

  //Create an object of class that will take care of everything
  controlLaw controlLawObject;

  ros::spin();

  return 0;
}

