#include "ros/ros.h"
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;
using namespace cv_bridge;
using namespace std;

//Options for speed-up: multithreading each image

Mat keyframe;
Mat videoFrame;
ros::Publisher homog_pub; // Publisher of type Float32MultiArray

int frameCount;

void calcHomography(){
  // The smaller the hessian value, the more points will be found at the cost of performance.

  int minHessian = 1000;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_keyframe, keypoints_video;
 // Detect features
  detector.detect( keyframe, keypoints_keyframe );
  detector.detect( videoFrame, keypoints_video );
 // Extract descriptors
  SurfDescriptorExtractor extractor;
  
  Mat descriptors_keyframe, descriptors_video;
  extractor.compute(keyframe, keypoints_keyframe, descriptors_keyframe);
  extractor.compute(videoFrame, keypoints_video, descriptors_video);
 // Match descriptor vectors
 //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_keyframe, descriptors_video, matches );

//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;  
  
  Mat img_matches;
  
  double max_dist = 0; double min_dist = 100;


  
  drawMatches( keyframe, keypoints_keyframe, videoFrame,  keypoints_video,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
               
  //cout << "Good matches: " << good_matches.size() << endl;
  
  for( int i = 0; i < descriptors_keyframe.rows; i++ )
  { if( matches[i].distance < 2*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_keyframe[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_video[ good_matches[i].trainIdx ].pt );
  }

  cv::Mat obj_mat(obj);
  cv::Mat scene_mat(scene);

  cv::Mat H = cv::findHomography( obj_mat, scene_mat, CV_RANSAC ); 
  
   std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( keyframe.cols, 0 );
  obj_corners[2] = cvPoint( keyframe.cols, keyframe.rows ); obj_corners[3] = cvPoint( 0, keyframe.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( keyframe.cols, 0), scene_corners[1] + Point2f( keyframe.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( keyframe.cols, 0), scene_corners[2] + Point2f( keyframe.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( keyframe.cols, 0), scene_corners[3] + Point2f( keyframe.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( keyframe.cols, 0), scene_corners[0] + Point2f( keyframe.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );
  waitKey(1);

  std::cout << H << std::endl;

  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
	msg.data.push_back(H.at<double>(i, j));
    }
  }
  
  homog_pub.publish(msg);
  //ROS_INFO("Published Matrix");
  ros::spinOnce();
}

void keyframeCallback(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("Received a keyframe");
  cout << frameCount++ << endl;
// Gets the data from the message, copies it into keyframe Mat, and then calculates the homography if both images are populated. 
  CvImagePtr data;
  data = toCvCopy(msg, "bgr8");
  keyframe = data->image;
  if (!videoFrame.empty() ){
    calcHomography();
  }
}

void videoStreamCallback(const sensor_msgs::Image::ConstPtr& msg){
  //ROS_INFO("Received a video frame");
  CvImagePtr data2;
  data2 = toCvCopy(msg, "bgr8");
  videoFrame = data2->image;
  if (!keyframe.empty() ){
    calcHomography();
  }
}


int main(int argc, char ** argv){

  frameCount = 0;
  ros::init(argc, argv, "homog_listener");

  ros::NodeHandle n;

  string keyframe;
  string video;
  string homog;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("keyframe_topic", keyframe, string("keyframe"));
  private_node_handle_.param("video_topic", video, string("video"));
  private_node_handle_.param("homography_topic", homog, string("homography_mat"));

  ros::Subscriber keyframeSub = n.subscribe(keyframe.c_str(), 1, keyframeCallback);
  ros::Subscriber videoFeedSub = n.subscribe(video.c_str(), 1, videoStreamCallback);
  homog_pub = n.advertise<std_msgs::Float32MultiArray>(homog.c_str(), 2);
 
  ros::spin();

  return 0;
}
