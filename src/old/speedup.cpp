// Find homography using OpenCV packages
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <Eigen/Dense>

#include <vector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <homography_calc/features.h>
#include <std_msgs/String.h>


using namespace cv;
using namespace cv_bridge;

class keypointProcessor
{
public:

/******************************************************************
***********  Initialization function for class  *******************
******************************************************************/

	keypointProcessor()	{
		int minHessian;
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
		#endif
		//Topic you want to publish
		keypointsPub = n.advertise<std_msgs::String>("to", 1);

		//Topic you want to subscribe
		imageSub = n.subscribe("recorded", 2, &keypointProcessor::keypointFind, this);
	}

/******************************************************************
*************  Keypoint Calculation Callback  *********************
******************************************************************/

	void keypointFind(const sensor_msgs::Image::ConstPtr& msg)
	{
    	ROS_INFO("IMG received33");
		CvImagePtr cv_ptr;
		cv_ptr = toCvCopy(msg, "bgr8");
		Mat image(cv_ptr->image);
		namedWindow("Video", WINDOW_NORMAL);
		imshow("Video", image);
		waitKey(1); // Wait for 3 ms.
		
		/*	cv::FastFeatureDetector detector( 50 ); 
			cv::SurfDescriptorExtractor extractor;  */
			 // Sift seems much less crowded than Surf.
			
	/*	std::vector<cv::KeyPoint> raw_kps_keyframe;
		detector.detect(image, raw_kps_keyframe);
		std::cout << "Detecting features\n";
		cv::Mat descriptors_keyframe;
		extractor.compute(image, raw_kps_keyframe, descriptors_keyframe);*/
		
		
		if (keyframe.empty()){
			keyframe = image;
			raw_kps_keyframe.erase(raw_kps_keyframe.begin(), raw_kps_keyframe.end() );
			detector.detect(keyframe, raw_kps_keyframe);
			exit; // If we have a keyframe, we know the homography is identity (should be)
		}
		
		std::vector<cv::KeyPoint> raw_kps_moving;
		// Optional rotation
		double rotation_angle;
		keypointProcessor::findParamDouble(&rotation_angle, "image_rotation_angle", 0.0);
		
			cv::FastFeatureDetector d2( 50 ); 
		keypointProcessor::rotate(image, rotation_angle);
		d2.detect(image, raw_kps_moving);
		cv::Mat descriptors_keyframe, descriptors_moving;
		//extractor.compute(keyframe, raw_kps_keyframe, descriptors_keyframe);
		//extractor.compute(image, raw_kps_moving, descriptors_moving);
		
		/********************************************
		Match the keypoints between the two images so a homography can be made. 
		********************************************/
		/*cv::BFMatcher matcher;
		std::vector< std::vector< cv::DMatch > > doubleMatches;
		std::vector< cv::DMatch > matches;
		
    	//#if 0
		/** Option 1: Straight feature matcher. ***/ 
				//matcher.match( descriptors_moving, descriptors_keyframe, matches ); 

		/** Option 2: knn matcher, which matches the best two points and then
		 finds the best of the two. It may be overkill.    **/
		 
	//	matcher.knnMatch( descriptors_moving, descriptors_keyframe, doubleMatches, 2 ); 
		
		std_msgs::String mmm;
		mmm.data = "Hello!";
		keypointsPub.publish(mmm);
  
	}

/******************************************************************
**************  Private Functions  ********************************
******************************************************************/
private:
	ros::NodeHandle n; 
	ros::Publisher keypointsPub;
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

