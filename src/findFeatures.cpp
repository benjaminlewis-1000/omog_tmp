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
#include <std_msgs/Bool.h>

#include "../include/database.h"

#define SURF 1
#define IN true
#define OUT false

class keypointProcessor
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	keypointProcessor()	{
		// Set up feature detectors for the keyframe 	
		
	//	 For some reason, making a new detector/extractor every time is *MAGNITUDES* faster!
	// SURF feature detector. Get the min_hessian parameter. 
		#if(SURF)
			if (ros::param::has("SURF_hessian") ){
				ros::param::get("SURF_hessian", minHessian);
			}else{
				ROS_WARN("SURF_hessian is not set.");
				minHessian = 50;
			}
		#endif
		
		// Topic for the start navigation signal. 
		std::string startTopic;
		if (ros::param::has("startTopic") ){
			ros::param::get("startTopic", startTopic);
		}else{
			ROS_ERROR("No start topic defined");
			exit(1);
		}
		
		// Database for find features only. Default is FIND_ONLY_KEYFRAMES postgresql database. 
		std::string findDB;
		if (ros::param::has("findDB") ){
			ros::param::get("findDB", findDB);
		}else{
			ROS_WARN("findDB is not set.");
			findDB = "FIND_ONLY_KEYFRAMES";
		}
		
		db = new keyframeDatabase(findDB);
		
		// Required parameter: keyframe save directory. 
		if (ros::param::has("imgSaveDir") ){
			ros::param::get("imgSaveDir", imgSaveDir);
		}else{
			ROS_ERROR("imgSaveDir is not set.");
			exit(1);
		}
		
		// Reset topic to get new keyframe signals from. The homography calculation
		// will determine when the homography becomes ill-conditioned by calculating the 
		// reprojection error. 	
		std::string resetTopic;
		if (ros::param::has("resetTopic") ){
			ros::param::get("resetTopic", resetTopic);
		}else{
			resetTopic = "keyframeReset";
		}
		
		// Topic of the video feed from the camera or a recorded video stream.
		std::string videoTopic;
		if (ros::param::has("videoFeed") ){
			ros::param::get("videoFeed", videoTopic);
		}else{
			videoTopic = "recorded";
		}
		
		// Topic to publish raw, unmatched keypoints. 
		std::string rawKeypointsTopic;
		if (ros::param::has("rawKeypointsTopic") ){
			ros::param::get("rawKeypointsTopic", rawKeypointsTopic);
		}else{
			rawKeypointsTopic = "rawKeypoints";
		}
		
		// System navigation topic to change the direction of navigation to OUT or IN. 
		std::string directionChangeTopic;
		if (ros::param::has("directionChangeTopic") ){
			ros::param::get("directionChangeTopic", directionChangeTopic);
		}else{
			directionChangeTopic = "navChange";
		}
		
		//Topic to which you want to subscribe or advertise.
		rawFeaturesPub = n.advertise<homography_calc::features>(rawKeypointsTopic, 1);
		
		keyframeResetSub = n.subscribe(resetTopic, 1, &keypointProcessor::resetKeyframe, this);
		 
		directionSub = n.subscribe(directionChangeTopic, 1, &keypointProcessor::switchNavDirection, this);
		
		startSub = n.subscribe(startTopic, 1, &keypointProcessor::start, this);

		imageSub = n.subscribe(videoTopic, 5, &keypointProcessor::keypointFind, this);
		
		startFlag = false;
			
		keyframeNum = 0;
		
		navigationDirection = OUT;
	}
	
	void switchNavDirection(const std_msgs::Bool msg){
		if (msg.data == true){
			navigationDirection = !navigationDirection;
			ROS_INFO("Switched navigation direction in matchRawFeatures");
		}
	}
	
/*	void reset(const std_msgs::Bool msg){
		if (msg.data == true){
			startFlag = true;
		}
	}*/
	
	void resetKeyframe(const std_msgs::Bool msg){
		if (msg.data == true){
			ROS_INFO("Keyframe Reset");
			
			if (navigationDirection == OUT){
			
				#if(SURF)
				// Create a new Fast feature detector and SURF descriptor extractor.
				// This is a speedup, for some reason, over keeping one detector/
				// extractor and using it every time. (Probably memory management.)
					cv::FastFeatureDetector detector( minHessian ); 
					 // Sift seems much less crowded than Surf.
					cv::SurfDescriptorExtractor extractor;  
				#endif
				
				// Lock the keyframe with a mutex while it is being written to
				imageAccessMutex.lock();
					keyframe = image; // Should be pulling in the latest received keyframe. 
				imageAccessMutex.unlock();
				
				// Clear the raw keypoints vector from beginning to end with a new keyframe. 
				raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() ); //
				// Detect the FAST points and compute their SURF descriptors.. 
				detector.detect(keyframe, raw_kps_keyframe);  //  
				extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);  //
		
				char buf[80];  
				// Save the keyframe to the image directory, then write all the data
				// to the database. Then exit this particular call to the function. 
		 		keyframeStack.push_back(keyframeNum);
		 		sprintf(buf, "%s/findMatchImages/IM%05d.jpg", imgSaveDir.c_str(), keyframeNum++);
		 		std::string fileLocation = buf;
		 		imwrite(fileLocation, image);
		 		db->addKeyframe(raw_kps_keyframe, fileLocation, descriptors_keyframe);
			
				exit; // If we have a keyframe, we know the homography is identity (should be)
			}else{ // Navigating out. 
				// Vector is erased and Mat is cleared automatically in getKeyframe function. 
				int kid;
				std::string fileLocation;
				// Pop out the keypoints, image location, and descriptors from the database
				// of the previous keyframe. 
				bool success = db->getKeyframe(kid, raw_kps_keyframe, 
						fileLocation, descriptors_keyframe);
				keyframe = imread(fileLocation);
				if ( ! (success && kid >= 0) ){
					// This should never happen. 
					ROS_ERROR("Failed to retreive keyframe from the database.");
					exit(1);
				}
				// For this, we shouldn't have to read the image because 
				// we have the key points and descriptors already computed.
			}			
		}
	}
	
	void start(const std_msgs::Bool msg){
		if (msg.data == true){
			startFlag = true;
		}
	}
/******************************************************************
*************  Keypoint Calculation Callback  *********************
******************************************************************/

	void keypointFind(const sensor_msgs::Image::ConstPtr& msg)
	{
		if (startFlag){
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		
		// Mutex the image while we're reading it. 
		imageAccessMutex.lock(); // Don't want the image changing halfway through. 
			image = cv::Mat(cv_ptr->image);
		imageAccessMutex.unlock();

		#if(SURF)
			cv::FastFeatureDetector detector( minHessian ); 
			cv::SurfDescriptorExtractor extractor;  
			// SURF descriptor calculation is much faster and works with
			// FAST feature points.  
			// FAST gets the points very quickly. 
		#endif
		
		// If the keyframe is empty, define it, empty the keyframe points buffer, and detect the key points. 
		// Should only run once. 
			if (keyframe.rows == 0 && keyframe.cols == 0){
			
				ROS_INFO("Getting the first keyframe");
				
				imageAccessMutex.lock(); // Don't want the image changing halfway through. 
					keyframe = image;
				imageAccessMutex.unlock();
				raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() ); //
				detector.detect(keyframe, raw_kps_keyframe);  //  
				extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);
		
				// If going out... 
				char buf[80];  
		 		keyframeStack.push_back(keyframeNum);
		 		sprintf(buf, "%s/findMatchImages/IM%05d.jpg", imgSaveDir.c_str(), keyframeNum++);
		 		std::string fileLocation = buf;
		 		imwrite(fileLocation, image);
		 		db->addKeyframe(raw_kps_keyframe, fileLocation, descriptors_keyframe);
		
				return; // If we have a keyframe, we know the homography is identity (should be)
				
			}
		
	// If the keyframe is empty, define it, empty the keyframe points buffer, and detect the key points. 
	//	if (startFlag){
	//ROS_INFO("Resetting the keyframe in findFeatures");
	/*	imageAccessMutex.lock(); // Don't want the image changing halfway through. 
			keyframe = image;
		imageAccessMutex.unlock(); // Don't want the image changing halfway through. 
			raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() );
			detector.detect(keyframe, raw_kps_keyframe);
			extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);
			startFlag = false;
			
	 		char buf[80];
	 		keyframeStack.push_back(keyframeNum);
	 		sprintf(buf, "%s/findOnlyImages/IM%05d.jpg", imgSaveDir.c_str(), keyframeNum++);
	 		std::string fileLocation = buf;
	 		imwrite(fileLocation, image);
	 		db->addKeyframe(raw_kps_keyframe, fileLocation, descriptors_keyframe);
			
			exit; // If we have a keyframe, we know the homography is identity (should be)
		}*/
		
		
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
		
	//std::cout << "Type: " << descriptors_moving.type() << std::endl;
		
		
	//	ROS_INFO("Number of descriptors is %d", raw_kps_moving.size() );
	//	std::cout << "Rows " << descriptors_moving.rows << " Cols " << descriptors_moving.cols << std::endl;
		
		homography_calc::features kpMsg;
		kpMsg.keyframe_pts.clear();
		for (int i = 0; i < raw_kps_keyframe.size(); i++){
			geometry_msgs::Point keyframePoint;
			keyframePoint.z = 0;
			keyframePoint.x = raw_kps_keyframe[i].pt.x;
			keyframePoint.y = raw_kps_keyframe[i].pt.y;
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

	/********************************************************************\
	**********************************************************************
	
	TODO: Publish the descriptors as an array instead of a bloody image... 
	
	**********************************************************************
	\********************************************************************/

		for (int i = 0; i < descriptors_keyframe.rows; i++){
			for (int j = 0; j < descriptors_keyframe.cols; j++){
				std_msgs::Float32 val;
				val.data = descriptors_keyframe.at<float>(i, j);
				kpMsg.kf_descriptors.push_back(val);
			}
		}
		
		for (int i = 0; i < descriptors_moving.rows; i++){
			for (int j = 0; j < descriptors_moving.cols; j++){
				std_msgs::Float32 val;
				val.data = descriptors_moving.at<float>(i,j);
				kpMsg.mv_descriptors.push_back(val);
			}
		}
	
		sensor_msgs::ImagePtr keymsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", descriptors_keyframe).toImageMsg();
		sensor_msgs::ImagePtr motionmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", descriptors_moving).toImageMsg();
		
		kpMsg.keyframe_descriptors = *keymsg;
		kpMsg.motion_descriptors = *motionmsg;
		
		kpMsg.timestamp = msg->header;  // Pass through the timestamp of the image
		rawFeaturesPub.publish(kpMsg);

		}
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Publisher rawFeaturesPub;
	ros::Publisher matchedPointsPub;
	ros::Subscriber imageSub;
	ros::Subscriber keyframeResetSub;
	ros::Subscriber startSub;
	ros::Subscriber directionSub;
	ros::Subscriber rawFeaturesInternalSub;
	cv::Mat keyframe; 
	std::vector<cv::KeyPoint> raw_kps_keyframe;
	cv::Mat descriptors_keyframe;
	
	cv::Mat image;
	
	Mutex imageAccessMutex;
	
	keyframeDatabase* db;
	int keyframeNum;
	int navigationDirection;
	std::vector<int> keyframeStack; // Keep a record of the keyframes we push on and take off.
	
	int minHessian;
	bool startFlag;
	std::string imgSaveDir;
	
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

};//End of class 

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "keypoint_calc");

  //Create an object of class SubscribeAndPublish that will take care of everything
  keypointProcessor SAPObject;

  ros::spin();

  return 0;
}

