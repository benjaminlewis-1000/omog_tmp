// Find homography using OpenCV packages

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <Eigen/Dense>
//#include <Eigen/SVD>
//#include <opencv2/core/eigen.hpp> // Should be included after decompose.h
//#include <libconfig.h++>

#include <vector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
//#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <homography_calc/featurePoints.h>
//#include <sensor_msgs/image_encodings.h>

#define SURF 1  // Use FAST/SURF (1) or SIFT (1) feature detectors and descriptors. 

void rotate(cv::Mat& src, double angle);

class keypointProcessor{

/*#if(SURF)
void homography(const sensor_msgs::Image::ConstPtr& msg, cv::Mat &keyframe,
	std::vector<cv::KeyPoint> &kp_keyframe, cv::FastFeatureDetector detector,
	cv::SurfDescriptorExtractor extractor);
#else
void homography(const sensor_msgs::Image::ConstPtr& msg, cv::Mat &keyframe,
	std::vector<cv::KeyPoint> &kp_keyframe, cv::SiftFeatureDetector detector,
	cv::SiftDescriptorExtractor extractor);
#endif*/
	
/****************************************************************
************   Rotate an image   ********************************
****************************************************************/

private:

	void rotate(cv::Mat& src, double angle)
	{
		int len = std::max(src.cols, src.rows);
		cv::Point2f pt(src.cols/2., src.rows/2.);
		cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

		cv::warpAffine(src, src, r, cv::Size(src.cols, src.rows));
	}

	ros::NodeHandle n;
	ros::Publisher pointPub;
	ros::Subscriber imageSub;
	cv::Mat keyframe; 
	std::vector<cv::KeyPoint> raw_kps_keyframe;
	
	#if(SURF)
		cv::FastFeatureDetector detector; 
		cv::SurfDescriptorExtractor extractor;  
	#else
		cv::SiftFeatureDetector detector;  
		cv::SiftDescriptorExtractor extractor;
	#endif

/****************************************************************
************   OpenCV homography callback   *********************
****************************************************************/

public:

	keypointProcessor(){ // init function
		
		//	cv::Mat keyframe; 

//	std::vector<cv::KeyPoint> raw_keypoints_keyframe;
/*	ros::Subscriber imSub;
	ros::Publisher pointsPub;*/
	
	/*int minHessian;
	// Set up feature detectors for the keyframe 	
 	#if(SURF)
 		if (ros::param::has("SURF_hessian") ){
			ros::param::get("SURF_hessian", minHessian);
		}else{
			ROS_WARN("SURF_hessian is not set.");
			minHessian = 50;
		}
		detector = cv::FastFeatureDetector ( minHessian ); 
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
	#endif*/
	
		pointPub = n.advertise<homography_calc::featurePoints>("topic", 1);
		imageSub = n.subscribe<sensor_msgs::Image>("recorded", 100, &keypointProcessor::homography);
		
		//boost::bind(homography, _1, 1) );// keyframe, raw_keypoints_keyframe, detector, extractor) );
	}

	//#if(SURF)
	void homography(const sensor_msgs::Image::ConstPtr& msg){
/*	#else
	void homography(const sensor_msgs::Image::ConstPtr& msg, cv::Mat &keyframe,
		std::vector<cv::KeyPoint> &raw_kps_keyframe, cv::FastFeatureDetector detector,
		cv::SurfDescriptorExtractor extractor){
	//#else
	void homography(const sensor_msgs::Image::ConstPtr& msg, cv::Mat &keyframe,
		std::vector<cv::KeyPoint> &raw_kps_keyframe, cv::SiftFeatureDetector detector,
		cv::SiftDescriptorExtractor extractor){
	#endif*/

	// Convert from the sensor image message to an OpenCV Mat.
		ROS_INFO("Received an image");
/*		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat image(cv_ptr->image);

	// If the keyframe is empty, define it, empty the keyframe points buffer, and detect the key points. 
		if (keyframe.empty()){
			keyframe = image;
			raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() );
			detector.detect(keyframe, raw_kps_keyframe);
			exit; // If we have a keyframe, we know the homography is identity (should be)
		}
	
	// If we don't have a brand new keyframe, compute the keypoints on the moving image,
	// then extract the feature descriptors to use in the matcher. 
		std::vector<cv::KeyPoint> raw_kps_moving;
		// Optional rotation
		double rotation_angle;
		if (ros::param::has("image_rotation_angle") ){
			ros::param::get("image_rotation_angle", rotation_angle);
		}else{
			rotation_angle = 0.0;
		}
		rotate(image, rotation_angle);
		detector.detect(image, raw_kps_moving);
		cv::Mat descriptors_keyframe, descriptors_moving;
		extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);
		extractor.compute(image, raw_kps_moving, descriptors_moving);
	
		/***********************
		Match the keypoints between the two images so a homography can be made. 
		***********************/
	/*	cv::BFMatcher matcher;
		std::vector< std::vector< cv::DMatch > > doubleMatches;
		std::vector< cv::DMatch > matches;
		// knnMatch matches the best two points, then can be used to find the best. 
		// I think it may be superfluous. 
		
		/** Option 1: Straight feature matcher. ***/ 
				//matcher.match( descriptors_moving, descriptors_keyframe, matches ); 
				// query, train, matches
		
		/** End Option 1  **/

		/** Option 2: knn matcher, which matches the best two points and then
		 finds the best of the two. It may be overkill.    **/
	/*	matcher.knnMatch( descriptors_moving, descriptors_keyframe, doubleMatches, 2 ); 
		// query, train, matches
		
		// If the matches are within a certain distance (descriptor space),
		// then call them a good match.
	
		double feature_distance;
		if (ros::param::has("featureDistance") ){
			ros::param::get("featureDistance", feature_distance);
		}else{
			feature_distance = 2.0;
		}
		for (int i = 0; i < doubleMatches.size(); i++) {
			if (doubleMatches[i][0].distance < feature_distance * 
					doubleMatches[i][1].distance){
				matches.push_back(doubleMatches[i][0]);
			}
		}	
	
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
		if (ros::param::has("minDistMult") ){
			ros::param::get("minDistMult", minDistMult);
		}else{
			minDistMult = 2.0;
		}
		for( int i = 0; i < descriptors_moving.rows; i++ ){ 
			if( matches[i].distance < minDistMult * min_dist ) { 
				good_matches.push_back( matches[i] );
			}
		}
		/***********************
		End of keypoint matching
		***********************/
	
		/***********************
		Copy the keypoints from the matches into their own vectors. 
		This is necessary because I don't want to take out any detected
		keypoints from the keyframe match, since I use that repeatedly.
		************************/
		// Vectors that contain the matched keypoints. 
	/*	std::vector<cv::Point2d> matched_kps_moved;
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
			if (ros::param::has("fundMatP1") ){
				ros::param::get("fundMatP1", fMatP1);
			}else{
				feature_distance = 2.0;
			}
			if (ros::param::has("fundMatP2") ){
				ros::param::get("fundMatP2", fMatP2);
			}else{
				feature_distance = 0.99;
			}
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
		/*********************
		Draw the matches.
		*********************/
	/*		std::vector<cv:: DMatch > drawn_matches;
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
		
			cv::Mat moved_mat(matched_kps_moved);  // TODO Look here (??)
			cv::Mat keyframe_mat(matched_kps_keyframe);


	//  geometry_msgs/Point
		/*****************************
		Compute the homography using OpenCV
		*****************************/
	/*		if (matched_kps_moved.size() >= 4){
			// Despite appearances, this isn't unnecessary. It's possible to remove points 
			// via RANSAC and then not have enough to compute the homography (also >=4 points required)
				cv::Mat H = cv::findHomography( moved_mat, keyframe_mat, CV_RANSAC);
			}
		}*/
	}

}; // End of class


/****************************************************************
******************  Main Function  ******************************
****************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "OpenCV_homog");

//	ros::NodeHandle n;

/*	cv::Mat keyframe; 

	std::vector<cv::KeyPoint> raw_keypoints_keyframe;
/*	ros::Subscriber imSub;
	ros::Publisher pointsPub;*/
	
/*	int minHessian;
	// Set up feature detectors for the keyframe 	
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
	
	imSub = n.subscribe<sensor_msgs::Image>("recorded", 100, boost::bind(homography, _1, keyframe, raw_keypoints_keyframe, detector, extractor) );
	
	pointsPub = n.advertise<homography_calc::featurePoints>("topic", 1);
*/
	ros::spin();

	return 0;
}

