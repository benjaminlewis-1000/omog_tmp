// Find homography using OpenCV packages
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <Eigen/Dense>

#include <vector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <homography_calc/matchedPoints.h>
#include <homography_calc/features.h>

#define SURF 1
#define IN true
#define OUT false

class matchPoints
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	matchPoints()	{
		
		navigationDirection = OUT;
	
		std::string rawKeypointsTopic;
		if (ros::param::has("rawKeypointsTopic") ){
			ros::param::get("rawKeypointsTopic", rawKeypointsTopic);
		}else{
			rawKeypointsTopic = "rawKeypoints";
		}
		
		std::string matchedPointsTopic;
		if (ros::param::has("matchedPointsTopic") ){
			ros::param::get("matchedPointsTopic", matchedPointsTopic);
		}else{
			matchedPointsTopic = "matchedPoints";
		}
		
		std::string directionChangeTopic;
		if (ros::param::has("directionChangeTopic") ){
			ros::param::get("directionChangeTopic", directionChangeTopic);
		}else{
			directionChangeTopic = "navChange";
		}
		
		directionSub = n.subscribe(directionChangeTopic, 1, &matchPoints::switchNavDirection, this);
		
		rawFeaturesSub = n.subscribe(rawKeypointsTopic, 1, &matchPoints::matchKeypoints, this);
		
		matchedPointsPub = n.advertise<homography_calc::matchedPoints>(matchedPointsTopic, 1);
	}
	
	void switchNavDirection(const std_msgs::Bool msg){
		if (msg.data == true){
			navigationDirection = !navigationDirection;
			ROS_INFO("Switched navigation direction in matchRawFeatures");
		}
	}
	
	//Subscribe to raw keypoints and match them.
	
	void matchKeypoints(const homography_calc::features& msg){
    	ROS_INFO("Received an internal message");
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg.keyframe_descriptors, "bgr8");
		cv::Mat descriptors_keyframe(cv_ptr->image);
		cv_ptr = cv_bridge::toCvCopy(msg.motion_descriptors, "bgr8");
		cv::Mat descriptors_moving(cv_ptr->image);
		
/*		descriptors_moving.convertTo(descriptors_moving, CV_32FC1);
		descriptors_keyframe.convertTo(descriptors_keyframe, CV_32FC1);*/
		
		/*std::cout << "Size of mat is " << descriptors_moving.rows << "  " << descriptors_moving.cols << std::endl;
		std::cout << "Size of kf mat is " << descriptors_keyframe.rows << "  " << descriptors_keyframe.cols << std::endl;
		std::cout << "Type: " << descriptors_moving.type() << "  " << descriptors_keyframe.type() << std::endl;*/
		int kf_size = msg.kf_descriptors.size() / 64;
		int mv_size = msg.mv_descriptors.size() / 64;
		
		cv::Mat kf_Mat = cv::Mat::zeros(kf_size, 64, CV_32FC1);
		cv::Mat mv_Mat = cv::Mat::zeros(mv_size, 64, CV_32FC1);
	//	std::cout << "Size of the mat array is " << mv_size << " elements (over 64) " << std::endl;
		
	// Get the keypoints - this is OK. 	
		std::vector<cv::Point2d> raw_kps_keyframe;
		for (int i = 0; i < msg.keyframe_pts.size(); i++){
			cv::Point2d pt(msg.keyframe_pts[i].x, msg.keyframe_pts[i].y);
			raw_kps_keyframe.push_back(pt);
		}
		std::vector<cv::Point2d> raw_kps_moving;
		for (int i = 0; i < msg.motion_pts.size(); i++){
			cv::Point2d pt(msg.motion_pts[i].x, msg.motion_pts[i].y);
			raw_kps_moving.push_back(pt);
		}
				
		for (int i = 0; i < kf_size; i++){
			for (int j = 0; j < 64; j++){
				float kf_val = msg.kf_descriptors[i * 64 + j].data;
				kf_Mat.at<float>(i,j) = kf_val;
			}
		}
		
		for (int i = 0; i < mv_size; i++){
			for (int j = 0; j < 64; j++){
				float mv_val = msg.mv_descriptors[i * 64 + j].data;
				mv_Mat.at<float>(i,j) = mv_val;
			}
		}
		
		/********************************************
		Match the keypoints between the two images so a homography can be made. 
		********************************************/
		cv::BFMatcher matcher;
		std::vector< std::vector< cv::DMatch > > doubleMatches;
		std::vector< cv::DMatch > matches;
		
		/** Option 1: Straight feature matcher. ***/ 
				//matcher.match( descriptors_moving, descriptors_keyframe, matches ); 

		/** Option 2: knn matcher, which matches the best two points and then
		 finds the best of the two. It may be overkill.    **/
		 
//		matcher.knnMatch( descriptors_moving, descriptors_keyframe, doubleMatches, 2 ); 
		matcher.knnMatch( mv_Mat, kf_Mat, doubleMatches, 2 ); 
		
	//	#if 0
		// If the matches are within a certain distance (descriptor space),
		// then call them a good match.
	
		double feature_distance;
		matchPoints::findParamDouble(&feature_distance, "featureDistance", 2.0);
		
		for (int i = 0; i < doubleMatches.size(); i++) {
			if (doubleMatches[i][0].distance < feature_distance * doubleMatches[i][1].distance){
				matches.push_back(doubleMatches[i][0]);
			}
		}	
		/** End of Option 2  **/
		
		double max_dist = 0; double min_dist = 100;

		// Regardless of option picked, calculate max and min distances (descriptor space) between keypoints
		for( int i = 0; i < descriptors_moving.rows; i++ )
		{
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}

		// Store only "good" matches (i.e. whose distance is less than minDistMult*min_dist )
		std::vector< cv::DMatch > good_matches;
		
		double minDistMult;
		matchPoints::findParamDouble(&minDistMult, "minDistMult", 2.0);
		
		for( int i = 0; i < descriptors_moving.rows; i++ ){ 
			if( matches[i].distance < minDistMult * min_dist ) { 
				good_matches.push_back( matches[i] );
			}
		}
		/********************************************
		End of keypoint matching
		********************************************/
	
		/********************************************
		Copy the keypoints from the matches into their own vectors. 
		This is necessary because I don't want to take out any detected
		keypoints from the keyframe match, since I use that repeatedly.
		*********************************************/
		// Vectors that contain the matched keypoints. 
		std::vector<cv::Point2d> matched_kps_moved;
		std::vector<cv::Point2d> matched_kps_keyframe;
	
		double max = 0.85;  
		double min = 0.15;
	
		for( int i = 0; i < good_matches.size(); i++ )
		{
		  matched_kps_moved.push_back( raw_kps_moving[i] );  // Left frame
		  matched_kps_keyframe.push_back( raw_kps_keyframe[i] );
		}		
	
		// OK, so now we have two vectors of matched keypoints. 
		if (! (matched_kps_moved.size() < 4 || matched_kps_keyframe.size() < 4) ){
			std::vector<uchar> status; 
			
			double fMatP1, fMatP2;
			matchPoints::findParamDouble(&fMatP1, "fundMatP1", 2.0);
			matchPoints::findParamDouble(&fMatP2, "fundMatP2", 0.99);
			
			findFundamentalMat(matched_kps_moved, matched_kps_keyframe,
				CV_FM_RANSAC, fMatP1, fMatP2, status);
		
			// Erase any points from the matched vectors that don't fit the fundamental matrix
			// with RANSAC.
			for (int i = matched_kps_moved.size() - 1; i >= 0; i--){
				if (status[i] == 0){
					matched_kps_moved.erase(matched_kps_moved.begin() + i);
					matched_kps_keyframe.erase(matched_kps_keyframe.begin() + i);
				}
			}
		/******************************************
		Draw the matches.  Not necessary here... 
		******************************************/
		/*	std::vector<cv:: DMatch > drawn_matches;
			std::vector<cv::KeyPoint> k2_moving, k2_keyframe;
			for (int i = 0; i < matched_kps_moved.size() ; i++){
				k2_moving.push_back(cv::KeyPoint(matched_kps_moved[i], 1.f) );
				k2_keyframe.push_back(cv::KeyPoint(matched_kps_keyframe[i], 1.f) );
				drawn_matches.push_back( cv::DMatch(i, i, 0) );
			}

			cv::Mat img_matches;
			cv::drawMatches(image, k2_moving, keyframe, k2_keyframe,
			   drawn_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
			   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
			cv::waitKey(1);
		*/
			cv::Mat moved_mat(matched_kps_moved);  // TODO Look here (??)
			cv::Mat keyframe_mat(matched_kps_keyframe);
			
			if (matched_kps_moved.size() >= 4){
			// Despite appearances, this isn't unnecessary. It's possible to remove points 
			// via RANSAC and then not have enough to compute the homography (also >=4 points required)
				homography_calc::matchedPoints kpMsg;
				kpMsg.keyframe_pts.clear();
				for (int i = 0; i < matched_kps_moved.size(); i++){
					geometry_msgs::Point keyframePoint;
					keyframePoint.z = 0;
					keyframePoint.x = matched_kps_keyframe[i].x;
					keyframePoint.y = matched_kps_keyframe[i].y;
					kpMsg.keyframe_pts.push_back(keyframePoint);
				
					geometry_msgs::Point motionPoint;
					motionPoint.z = 0;
					motionPoint.x = matched_kps_moved[i].x;
					motionPoint.y = matched_kps_moved[i].y;
					kpMsg.motion_pts.push_back(motionPoint);
				}
				matchedPointsPub.publish(kpMsg);
			}
			

	//  geometry_msgs/Point
		/**************************************************
		Compute the homography using OpenCV
		**************************************************/
		/*	if (matched_kps_moved.size() >= 4){
			// Despite appearances, this isn't unnecessary. It's possible to remove points 
			// via RANSAC and then not have enough to compute the homography (also >=4 points required)
				cv::Mat H = cv::findHomography( moved_mat, keyframe_mat, CV_RANSAC);
			}*/
		}
		
	//		#endif
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Publisher matchedPointsPub;
	ros::Subscriber rawFeaturesSub;
	ros::Subscriber directionSub;
	cv::Mat keyframe; 
	std::vector<cv::KeyPoint> raw_kps_keyframe;
	cv::Mat descriptors_keyframe;
	
	int minHessian;
	int navigationDirection;
	
	void rotate(cv::Mat& src, double angle)
	{
		int len = std::max(src.cols, src.rows);
		cv::Point2f pt(src.cols/2., src.rows/2.);
		cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

		cv::warpAffine(src, src, r, cv::Size(src.cols, src.rows));
	}
	
	void findParamDouble(double *paramVar, std::string paramName, double defaultVal){
		if (ros::param::has("paramName") ){
			ros::param::get("paramName", *paramVar);
		}else{
			*paramVar = defaultVal;
		}
	}

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "matchPoints");

  //Create an object of class SubscribeAndPublish that will take care of everything
  matchPoints matcher;

  ros::spin();

  return 0;
}

