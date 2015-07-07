// Find homography using OpenCV packages
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <Eigen/Dense>

#include <vector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <homography_calc/matchedPoints.h>
#include <homography_calc/features.h>

#define SURF 1

class keypointProcessor
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	keypointProcessor()	{
		// Set up feature detectors for the keyframe 	
		
		
	//	 For some reason, making a new detector/extractor every time is *MAGNITUDES* faster!
		#if(SURF)
			if (ros::param::has("SURF_hessian") ){
				ros::param::get("SURF_hessian", minHessian);
			}else{
				ROS_WARN("SURF_hessian is not set.");
				minHessian = 50;
			}
		#endif
		
	//	minHessian = 50;
		
		#if 0
		#if(SURF)
			if (ros::param::has("SURF_hessian") ){
				ros::param::get("SURF_hessian", minHessian);
			}else{
				ROS_WARN("SURF_hessian is not set.");
				minHessian = 50;
			}
			cv::FastFeatureDetector detector( minHessian ); 
			 // Sift seems much less crowded than Surf.
			cv::SurfDescriptorExtractor extractor;  
			// SURF descriptor calculation is much faster. 
			// FAST gets the points very quickly. 
		#else
			if (ros::param::has("SIFT_hessian") ){
				ros::param::get("SIFT_hessian", minHessian);
			}else{
				ROS_WARN("SIFT_hessian is not set.");
				minHessian = 400;
			}
			cv::SiftFeatureDetector detector( minHessian );  
			cv::SiftDescriptorExtractor extractor;
		#endif
		
		#endif
		//Topic you want to publish
		rawFeaturesPub = n.advertise<homography_calc::features>("keypoints", 1);

		//Topic you want to subscribe
		imageSub = n.subscribe("recorded", 5, &keypointProcessor::keypointFind, this);
		
		rawFeaturesInternalSub = n.subscribe("keypoints", 5, &keypointProcessor::matchKeypoints, this);
		
		matchedPointsPub = n.advertise<homography_calc::matchedPoints>("matchedPoints", 1);
	}
	
	void matchKeypoints(const homography_calc::features& msg){
    	ROS_INFO("Received an internal message");
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg.keyframe_descriptors, "bgr8");
		cv::Mat descriptors_keyframe(cv_ptr->image);
		cv_ptr = cv_bridge::toCvCopy(msg.motion_descriptors, "bgr8");
		cv::Mat descriptors_moving(cv_ptr->image);
		
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
		
			/********************************************
		Match the keypoints between the two images so a homography can be made. 
		********************************************/
		cv::BFMatcher matcher;
		std::vector< std::vector< cv::DMatch > > doubleMatches;
		std::vector< cv::DMatch > matches;
		
    	//#if 0
		/** Option 1: Straight feature matcher. ***/ 
				//matcher.match( descriptors_moving, descriptors_keyframe, matches ); 

		/** Option 2: knn matcher, which matches the best two points and then
		 finds the best of the two. It may be overkill.    **/
		 
		matcher.knnMatch( descriptors_moving, descriptors_keyframe, doubleMatches, 2 ); 
		
		// If the matches are within a certain distance (descriptor space),
		// then call them a good match.
	
		double feature_distance;
		keypointProcessor::findParamDouble(&feature_distance, "featureDistance", 2.0);
		
		for (int i = 0; i < doubleMatches.size(); i++) {
			if (doubleMatches[i][0].distance < feature_distance * 
					doubleMatches[i][1].distance){
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
		keypointProcessor::findParamDouble(&minDistMult, "minDistMult", 2.0);
		
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
			keypointProcessor::findParamDouble(&fMatP1, "fundMatP1", 2.0);
			keypointProcessor::findParamDouble(&fMatP2, "fundMatP2", 0.99);
			
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
		
	}

/******************************************************************
*************  Keypoint Calculation Callback  *********************
******************************************************************/

	void keypointFind(const sensor_msgs::Image::ConstPtr& msg)
	{
    	ROS_INFO("Received an image1236");
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat image(cv_ptr->image);
		
		//if (ros::param::has("SURF_hessian") ){
		//	ros::param::get("SURF_hessian", minHessian);
	//	}else{
		//	ROS_WARN("SURF_hessian is not set.");
			//minHessian = 50;
		//}
		
		#if(SURF)
			cv::FastFeatureDetector detector( minHessian ); 
			 // Sift seems much less crowded than Surf.
			cv::SurfDescriptorExtractor extractor;  
			// SURF descriptor calculation is much faster. 
			// FAST gets the points very quickly. 
		#else
		/*	if (ros::param::has("SIFT_hessian") ){
				ros::param::get("SIFT_hessian", minHessian);
			}else{
				ROS_WARN("SIFT_hessian is not set.");
				minHessian = 400;
			}
			cv::SiftFeatureDetector detector( minHessian );  
			cv::SiftDescriptorExtractor extractor;*/
		#endif
		
	// If the keyframe is empty, define it, empty the keyframe points buffer, and detect the key points. 
		if (keyframe.empty()){
			keyframe = image;
			raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() );
			detector.detect(keyframe, raw_kps_keyframe);
			extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);
			exit; // If we have a keyframe, we know the homography is identity (should be)
		}
		
		
	// If we don't have a brand new keyframe, compute the keypoints on the moving image,
	// then extract the feature descriptors to use in the matcher. 
		std::vector<cv::KeyPoint> raw_kps_moving;
		// Optional rotation
		double rotation_angle;
		keypointProcessor::findParamDouble(&rotation_angle, "image_rotation_angle", 0.0);
		
		keypointProcessor::rotate(image, rotation_angle);
		detector.detect(image, raw_kps_moving);
		cv::Mat descriptors_moving;
		extractor.compute(image, raw_kps_moving, descriptors_moving);
	//	ROS_INFO("Number of descriptors is %d", raw_kps_moving.size() );
		std::cout << "Rows " << descriptors_moving.rows << " Cols " << descriptors_moving.cols << std::endl;
		
		homography_calc::features kpMsg;
		kpMsg.keyframe_pts.clear();
		for (int i = 0; i < raw_kps_keyframe.size(); i++){
			geometry_msgs::Point keyframePoint;
			keyframePoint.z = 0;
			keyframePoint.x = raw_kps_keyframe[i].pt.x;;
			keyframePoint.y = raw_kps_keyframe[i].pt.y;;
			kpMsg.keyframe_pts.push_back(keyframePoint);
		}
		// Not necessarily the same size, yet.
		for (int i = 0; i < raw_kps_moving.size(); i++){
			geometry_msgs::Point motionPoint;
			motionPoint.z = 0;
			motionPoint.x = raw_kps_moving[i].pt.x;
			motionPoint.y = raw_kps_moving[i].pt.y;
			kpMsg.motion_pts.push_back(motionPoint);
		}
		
        cv_bridge::CvImage cvi_keyframe;
        cvi_keyframe.header = std_msgs::Header();
        cvi_keyframe.encoding = "bgr8";
        cvi_keyframe.image = descriptors_keyframe;
		sensor_msgs::Image im_keyframe;
		cvi_keyframe.toImageMsg(im_keyframe);
		kpMsg.keyframe_descriptors = im_keyframe;
		
        cv_bridge::CvImage cvi_moving;
        cvi_moving.header = std_msgs::Header();
        cvi_moving.encoding = "bgr8";
        cvi_moving.image = descriptors_moving;
		sensor_msgs::Image im_moving;
		cvi_moving.toImageMsg(im_moving);
		kpMsg.motion_descriptors = im_moving;
		
		kpMsg.timestamp = msg->header;  // Pass through the timestamp of the image
		rawFeaturesPub.publish(kpMsg);
	
	#if 0
		/********************************************
		Match the keypoints between the two images so a homography can be made. 
		********************************************/
		cv::BFMatcher matcher;
		std::vector< std::vector< cv::DMatch > > doubleMatches;
		std::vector< cv::DMatch > matches;
		
    	//#if 0
		/** Option 1: Straight feature matcher. ***/ 
				//matcher.match( descriptors_moving, descriptors_keyframe, matches ); 

		/** Option 2: knn matcher, which matches the best two points and then
		 finds the best of the two. It may be overkill.    **/
		 
		matcher.knnMatch( descriptors_moving, descriptors_keyframe, doubleMatches, 2 ); 
		
		// If the matches are within a certain distance (descriptor space),
		// then call them a good match.
	
		double feature_distance;
		keypointProcessor::findParamDouble(&feature_distance, "featureDistance", 2.0);
		
		for (int i = 0; i < doubleMatches.size(); i++) {
			if (doubleMatches[i][0].distance < feature_distance * 
					doubleMatches[i][1].distance){
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
		keypointProcessor::findParamDouble(&minDistMult, "minDistMult", 2.0);
		
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
		  matched_kps_moved.push_back( raw_kps_moving[ good_matches[i].queryIdx ].pt );  // Left frame
		  matched_kps_keyframe.push_back( raw_kps_keyframe[ good_matches[i].trainIdx ].pt );
		}		
	
		// OK, so now we have two vectors of matched keypoints. 
		if (! (matched_kps_moved.size() < 4 || matched_kps_keyframe.size() < 4) ){
			std::vector<uchar> status; 
			
			double fMatP1, fMatP2;
			keypointProcessor::findParamDouble(&fMatP1, "fundMatP1", 2.0);
			keypointProcessor::findParamDouble(&fMatP2, "fundMatP2", 0.99);
			
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
				homography_calc::featurePoints kpMsg;
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
				keypointsPub.publish(kpMsg);
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
		#endif
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Publisher rawFeaturesPub;
	ros::Publisher matchedPointsPub;
	ros::Subscriber imageSub;
	ros::Subscriber rawFeaturesInternalSub;
	cv::Mat keyframe; 
	std::vector<cv::KeyPoint> raw_kps_keyframe;
	cv::Mat descriptors_keyframe;
	
	int minHessian;
	
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
  ros::init(argc, argv, "keypoint_calc");

  //Create an object of class SubscribeAndPublish that will take care of everything
  keypointProcessor SAPObject;

  ros::spin();

  return 0;
}

