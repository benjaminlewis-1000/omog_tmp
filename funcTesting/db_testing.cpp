#include <string>
#include <vector>
#include <sys/stat.h>

#include <database_interface/db_class.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

#include <geometry_msgs/Point.h>
#include "../include/database.h"

#include <cstdlib>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Database_client");
  
  bool fail = 0;
  
  // These should all connect, I'm not going to bother putting try-catch around them. 
	keyframeDatabase* db1;  
	db1 = new keyframeDatabase("FIND_ONLY_KEYFRAMES");
	keyframeDatabase* db2;  
	db2 = new keyframeDatabase("FIND_MATCH_KEYFRAMES");
	keyframeDatabase* db3;  
	db3 = new keyframeDatabase("ESM_KEYFRAMES");
	// Shouldn't connect.
	keyframeDatabase* dummy;  
	try{
		dummy = new keyframeDatabase("DUMMY_DB");
		// Shouldn't hit this next line if it fails to connect.
		ROS_ERROR("Database did *not* fail to connect when it should have.");
		fail = true;
	}catch(...){
		ROS_INFO("Database failed, as it should.");
	}
	
	Mat img = imread("/home/benjamin/Research/Code/ESM_lib/res/im000.pgm");

	FastFeatureDetector detector( 50 ); 
	SurfDescriptorExtractor extractor;  
	vector<cv::KeyPoint> raw_kps_keyframe;
	Mat descriptors_keyframe;
	detector.detect(img, raw_kps_keyframe);
	extractor.compute(img, raw_kps_keyframe, descriptors_keyframe);
	
	imwrite("test.jpg", img);
	if (db1->addKeyframe(raw_kps_keyframe, "test.jpg", descriptors_keyframe) ){
		ROS_INFO("Success db1");
	}else{
		ROS_INFO("Something went wrong in db1");
		fail = true;
	}
	if (db2->addKeyframe(raw_kps_keyframe, "test.jpg", descriptors_keyframe) ){
		ROS_INFO("Success db2");
	}else{
		ROS_INFO("Something went wrong in db2");
		fail = true;
	}
	if (db3->addKeyframe(raw_kps_keyframe, "test.jpg", descriptors_keyframe) ){
		ROS_INFO("Success db3");
	}else{
		ROS_INFO("Something went wrong in db3");
		fail = true;
	}
	if (db3->addKeyframe(raw_kps_keyframe, "test.jpg", descriptors_keyframe) ){
		ROS_INFO("Success db3");
	}else{
		ROS_INFO("Something went wrong in db3");
		fail = true;
	}
	if (db3->addKeyframe(raw_kps_keyframe, "test.jpg", descriptors_keyframe) ){
		ROS_INFO("Success db3");
	}else{
		ROS_INFO("Something went wrong in db3");
		fail = true;
	}
	
	int kid;
	std::vector<cv::KeyPoint> retVal;
	std::string location;
	cv::Mat desc;
			
	if (db1->getKeyframe(kid, retVal,location,desc) ) {
		for (int i = 0; i < 10; i++){ // Test out a few values
			if ( (retVal[0].pt.x != raw_kps_keyframe[0].pt.x) || (retVal[0].pt.y != raw_kps_keyframe[0].pt.y)  ){
				ROS_ERROR("Not matching when it comes out, keypoints!");
			fail = true;
				break;
			}
			if (desc.at<double>(0, i) != descriptors_keyframe.at<double>(0,i) ){
				ROS_ERROR("Not matching when it comes out, descriptors!");
			fail = true;
				break;
			}
			if (i == 9){
				// Got to the end, no errors.
				ROS_INFO("Looks good in db1 if there were no messages about not matching");
			}
		}
	
	}
	
	if ( db1->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Tried to take too many off, and it didn't fail!");
	}
	
	
	if (db3->getKeyframe(kid, retVal,location,desc) ){
		for (int i = 0; i < 10; i++){ // Test out a few values
			if ( (retVal[0].pt.x != raw_kps_keyframe[0].pt.x) || (retVal[0].pt.y != raw_kps_keyframe[0].pt.y)  ){
				ROS_ERROR("Not matching when it comes out, keypoints!");
			fail = true;
				break;
			}
			if (desc.at<double>(0, i) != descriptors_keyframe.at<double>(0,i) ){
				ROS_ERROR("Not matching when it comes out, descriptors!");
			fail = true;
				break;
			}
			if (i == 9){
				// Got to the end, no errors.
				ROS_INFO("Looks good in db3 if there were no messages about not matching");
			}
		}
	}
	if (! db3->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Should have more items left!");
		fail = true;
	}
	if (! db3->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Should have more items left!");
		fail = true;
	}
	if ( db3->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Should *not* have more items left!");
		fail = true;
	}
	if ( db3->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Should *not* have more items left!");
		fail = true;
	}
	if ( db3->getKeyframe(kid, retVal,location,desc) ){ // Should be empty
		ROS_ERROR("Should *not* have more items left!");
		fail = true;
	}
	
  
  
  	if (!fail){
  		std::cout << "\n\n";
  		ROS_INFO("All tests passed!");
  	}else{
  		std::cout << "\n\n";
  		ROS_INFO("Some tests failed, perhaps you forgot to clear the databases?");
  	}
  /*srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);*/
  /*ros::Time begin = ros::Time::now();
	  if (client.call(srv))
	  {
		ROS_INFO("Success: %d", srv.response.success.data);
	  }
	  else
	  {
		ROS_ERROR("Failed to call service");
		return 1;
	  }
	  
	  ros::Time end = ros::Time::now();
	  
	  ros::Duration passed = end - begin;
	  cout << passed.toSec() << endl;
	  
	  */
	  /*
	ros::ServiceClient getC = n.serviceClient<homography_calc::getKeyframe>("getKeyframe");
  homography_calc::getKeyframe srv2;
  if(getC.call(srv2) ){
  	ROS_INFO("Success: %d", srv2.response.success.data);
  }else{
  	ROS_ERROR("Database issue.");
  	return 1;
  }*/
	

  return 0;
}
