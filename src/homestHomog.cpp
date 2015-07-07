// Find homography using OpenCV packages
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include "ros/ros.h"
#include <homography_calc/matchedPoints.h>
#include <std_msgs/Float64MultiArray.h>
#include "../include/homest_interface.h"
#include <std_msgs/Bool.h>
#include <homography_calc/controlLawInputs.h>
#include <std_msgs/Bool.h>

#define IN true
#define OUT false

Eigen::MatrixXd centerOfMass(std::vector<cv::Point2d> points);

Eigen::MatrixXd centerOfMass(std::vector<cv::Point2d> points){
	double x_center = 0.0;
	double y_center = 0.0;
	int num_points = points.size();
	for (int i = 0; i < num_points; i++){
		x_center += points[i].x;
		y_center += points[i].y;
	}
	
	Eigen::MatrixXd ret(3,1);
	ret << x_center/num_points, y_center/num_points, 1.0;
	
	return ret;
}

class homestHomogCalc
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	homestHomogCalc()	{
		//Topic you want to subscribe to
		
		navigationDirection = OUT;
			
		std::string matchedPointsTopic;
		if (ros::param::has("matchedPointsTopic") ){
			ros::param::get("matchedPointsTopic", matchedPointsTopic);
		}else{
			matchedPointsTopic = "matchedPoints";
		}
		
		matchedSub = n.subscribe(matchedPointsTopic, 5, &homestHomogCalc::calculate, this);
			
		std::string homogTopic;
		if (ros::param::has("homogTopic") ){
			ros::param::get("homogTopic", homogTopic);
		}else{
			homogTopic = "homestHomog";
		}
				
		homogPub = n.advertise<std_msgs::Float64MultiArray>(homogTopic, 1);

		std::string resetTopic;
		if (ros::param::has("resetTopic") ){
			ros::param::get("resetTopic", resetTopic);
		}else{
			homogTopic = "keyframeReset";
		}
		
		resetPub = n.advertise<std_msgs::Bool>(resetTopic, 1);
		
		std::string directionChangeTopic;
		if (ros::param::has("directionChangeTopic") ){
			ros::param::get("directionChangeTopic", directionChangeTopic);
		}else{
			directionChangeTopic = "navChange";
		}
		
		directionSub = n.subscribe(directionChangeTopic, 1, &homestHomogCalc::switchNavDirection, this);
		
		std::string homogControlTopic;
		if (ros::param::has("homogControlInputTopic") ){
			ros::param::get("homogControlInputTopic", homogControlTopic);
		}else{
			homogControlTopic = "homestHomogControlLawInputs";
		}
				
		homogControlPub = n.advertise<homography_calc::controlLawInputs>(homogControlTopic, 1);
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

	void calculate(const homography_calc::matchedPoints& msg)
	{
    	ROS_INFO("Received matched points");
		
		std::vector<cv::Point2d> matched_kps_keyframe;
		for (int i = 0; i < msg.keyframe_pts.size(); i++){
			cv::Point2d pt(msg.keyframe_pts[i].x, msg.keyframe_pts[i].y);
			matched_kps_keyframe.push_back(pt);
		}
		std::vector<cv::Point2d> matched_kps_moving;
		for (int i = 0; i < msg.motion_pts.size(); i++){
			cv::Point2d pt(msg.motion_pts[i].x, msg.motion_pts[i].y);
			matched_kps_moving.push_back(pt);
		}
		
		cv::Mat moved_mat(matched_kps_moving);  // TODO Look here (??)
		cv::Mat keyframe_mat(matched_kps_keyframe);
				
		if (matched_kps_moving.size() >= 4){
		// Despite appearances, this isn't unnecessary. It's possible to remove points 
		// via RANSAC and then not have enough to compute the homography (also >=4 points required)
		//	std::cout << "here " << std::endl;
			double H_array[9];
			findHomographyHomest( matched_kps_moving, matched_kps_keyframe, H_array ); 
			cv::Mat H = cv::Mat(3,3,CV_64F, &H_array);
			
		//	printf("H is %f %f %f ", H_array[0], H_array[1], H_array[2]);
		//	std::cout << H << std::endl;
		
		/*****************************
		Calculate reprojection error and define new keyframe if necessary.
		*****************************/
			std::vector<cv::Point2d> keyframe_reproj;
			cv::perspectiveTransform( matched_kps_moving, keyframe_reproj, H);
			// keyframe_reproj = H * matched_keypoints_moved;
	
			double err = 0;
			for (int i = 0; i < matched_kps_moving.size(); i++){
				err += cv::norm(cv::Mat(matched_kps_keyframe[i] ), cv::Mat(keyframe_reproj[i] ) );
			}
			err = err / matched_kps_moving.size();
			std::cout << "Reprojection error is " << err << std::endl;	
			
			double kfResetThresh;
			if (ros::param::has("keyframeResetThresh") ){
				ros::param::get("keyframeResetThresh", kfResetThresh);
			}else{
				kfResetThresh = 30;
			}
			if (err > kfResetThresh){
				ROS_INFO("Resetting the keyframe NOW");
				std_msgs::Bool resetMsg;
				resetMsg.data = true;
				resetPub.publish(resetMsg);
			}
		
			//Publish the homography
		
			std_msgs::Float64MultiArray homography;
			for(int i = 0 ; i < 3; i++){
				for (int j = 0; j < 3; j++){
					homography.data.push_back(H.at<float>(i, j) );
				}
			}
			
			
			homogPub.publish(homography);
			
			Eigen::MatrixXd currentCenter(3,1);
			Eigen::MatrixXd targetCenter(3,1);
			
			currentCenter = centerOfMass(matched_kps_moving);
			targetCenter = centerOfMass(matched_kps_keyframe);
			
			homography_calc::controlLawInputs controlMsg;
			
			controlMsg.homography = homography;
			
			controlMsg.currentCenterOfMass.x = currentCenter(0,0);
			controlMsg.currentCenterOfMass.y = currentCenter(0,1);
			controlMsg.currentCenterOfMass.z = currentCenter(0,2);
			
			controlMsg.targetCenterOfMass.x = targetCenter(0,0);
			controlMsg.targetCenterOfMass.y = targetCenter(0,1);
			controlMsg.targetCenterOfMass.z = targetCenter(0,2);
			
			homogControlPub.publish(controlMsg);
		}
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Subscriber matchedSub;	
	ros::Subscriber directionSub;
	ros::Publisher homogPub;
	ros::Publisher resetPub;
	ros::Publisher homogControlPub;
	
	int navigationDirection;

};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "homographyHomest");

  //Create an object of class SubscribeAndPublish that will take care of everything
  homestHomogCalc homestHomogCalcObject;

  ros::spin();

  return 0;
}

