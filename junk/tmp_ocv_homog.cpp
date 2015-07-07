/**********************************************************************
*
*   homography.cpp
*   Homography servoing test code
*	By Benjamin Lewis, May 2015
*
*   This is research code based on a variety of homography methods. It 
*   uses OpenCV to read in a video taken from an airborne camera that 
*   is gimballed down to point at the ground. Ideally, it should 
*   calculate the homography between the current frame and a keyframe. 
*   When the reprojection error of the homography grows too large, it 
*   defines a new keyframe. If a homography is not able to be computed, 
*   the frame is skipped. The homography is then decomposed into a 
*   translation and a rotation vector. A few methods for doing this,
*   including my own implementation in ../headers/decompose2.h and a 
*   copy of OpenCV's decomposition, located in ../headers/hd_ocv.h, is 
*   used. I also have, for comparison, a direct servoing method in 
*   ../headers/homography_servoing.h, which provides control laws 
*   (which I would otherwise derive from the translation and rotation in
*   the decomposition. All of these header files cite the paper on which 
*   they are based individually; therefore, I refrain from citing 
*   them here. 
*
*   For the homography as computed, I can use a variety of feature 
*   descriptors and extractors. I have found that the FAST feature detector 
*   coupled with the SURF feature descriptor provides good, fast 
*   performance and is overall reliable when presented with good features. 
*
*   Configurations for many settings are provided in the file 'config.txt',
*   located in the ../ directory from this file. (That directory is also 
*   the location of the compiled binary currently). 
*
*	This code is very much a testing code and is subject to drastic change. 
*
***************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp> // Should be included after decompose.h
#include <libconfig.h++>

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <fstream> // Write to file
#include <list>

#include "../headers/fps_calc.h"
#include "../headers/logging.h"
#include "../headers/decompose2.h"
#include "../headers/hd_ocv.h"
#include "../headers/homography_servoing.h"

#define CALC_FPS 1  // Whether or not to calculate the frames per second.
#define M_PI  3.14159265358979323846
#define RADIAN_TO_DEGREE (180/M_PI)
#define DB 0  // Turn on (1) or off (0) certain debug parameters at
// compile time.
#define SURF 1  // Use FAST/SURF (1) or SIFT (1) feature detectors and descriptors. 
#define CV_HOMOG 0  // Use OpenCV's findHomography (1) or homest (0)

#if !CV_HOMOG
	#include "../headers/homest_interface.h"
#endif

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace libconfig;

volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum) {
 if (quit_signal!=0) exit(0); // just exit already
 quit_signal=1;
}
#endif

// Stack overflow: SIFT is not designed to deal with significant 
// perspective distortions that are present. "
// http://stackoverflow.com/questions/19019726/how-to-improve-the-homography-accuracy

void analyzeHomography(cv::Mat H, Eigen::Matrix3d K );
void rotate(cv::Mat& src, double angle, cv::Mat& dst);
std::string print33Matrix(Eigen::Matrix3d M);
std::string print33Mat(cv::Mat M, std::string name);

/**********************************************************
	BEGINNING OF HOMOGRAPHY TEST CODE
***********************************************************/
int main(int argc, char** argv){

// Code and variable setup
	#if(DB)
		std::ofstream out_file;
		out_file.open(cfg.lookup("homography.debugFile"), std::ofstream::out | std::ofstream::app);
	#endif
	
/**************************************************************************
	Preamble - play the video and wait for a spacebar or the given number 
	of frames to pass (whichever is first) to define the first keyframe.
**************************************************************************/
	int count = 0;
	for(;;){
		Mat frame;;
		count++;
		if (count == delay){
			center = frame;
			break;
		}
		if (quit_signal) exit(0);
		
		imshow("Screen", frame);
		
		char key = waitKey(1);
		if (key == ' '){
			center = frame;
			break;
		}else if(key == 'p'){
			while(key != 'c'){
		cap >> frame
				key = waitKey(30);
			}
		}
	}
	
// TODO: Try to get more corner matches than center matches, as these will have a better effect on homography. Maybe I can throw out points in the center region of one of the frames, then try to match...
    int frameno=0; 
    double r_err = 0; // Reprojection error
/**************************************************************************
	Beginning of main processing loop. This loop takes in each frame, finds
	feature points, computes a homography, decomposes the homography, 
	computes the control law, and so on. If the reprojection error grows 
	too large on the homography, a new keyframe is defined. 
**************************************************************************/
	count = 0;
	for (;;){



		
// This if statement encompasses the rest of the function. If we don't have 
// enough common points to perform a fundamental matrix calculation,
// then we also can't do a homography and we should move on to the next frame.
		if (! (matched_keypoints_moved.size() < 4 || matched_keypoints_keyframe.size() < 4) ){
		
		
/*********************
	Draw the matches.
*********************/
/***********************
	End of drawing matches.
***********************/

		
			if (matched_keypoints_moved.size() >= 4){
			// Despite appearances, this isn't unnecessary. It's possible to remove points 
			// via RANSAC and then not have enough to compute the homography (also >=4 points required)
				#else
					double H_array[9];
					findHomographyHomest( matched_keypoints_moved, matched_keypoints_keyframe, H_array ); 
					Mat H = Mat(3,3,CV_64FC1, &H_array);
					
					cv::Mat H2 = cv::findHomography( moved_mat, keyframe_mat, CV_RANSAC ); 
				#endif
			
// Analyze the homography 

				analyzeHomography(H, K);
				
				Eigen::Matrix3d hh;
				cv2eigen(H,hh);
				
				MatrixXd result = homogControlLaw(hh, K, matched_keypoints_keyframe, matched_keypoints_moved, 1.0, 0.1);
				log(LOG_CRITICAL) << "Control law is " << result.transpose() << endl;
				log(LOG_CRITICAL) << "That angle is " << atan2(result(1,0) , result(0,0) ) * RADIAN_TO_DEGREE << "\u00B0" << endl;
				
				#if(!CV_HOMOG)
				
					string sep = "--------------------------------------------------------------------------";
					log(LOG_CRITICAL) << sep << print33Mat(H, "Homest H") ;
					log(LOG_CRITICAL) << sep << print33Mat(H2, "OpenCV H") << sep << endl;
							
					Eigen::Matrix3d h2;
					cv2eigen(H2,h2);
					
					result = homogControlLaw(h2, K, matched_keypoints_keyframe, matched_keypoints_moved, 1.0, 0.1);
					log(LOG_CRITICAL) << "Control law 2 is " << result.transpose() << endl;
					log(LOG_CRITICAL) << "That angle is " << atan2(result(1,0) , result(0,0) ) * RADIAN_TO_DEGREE << "\u00B0" << endl;
					
				#endif
/*****************************
	End of homography computation
*****************************/		
			
				log(LOG_INFO) << "Done, in main. H is: " << endl << H << endl;
			
				// Calculate reprojection error
				// TODO: Refine the reprojection error code and fix it. (Done I think)
			
/*****************************
	Calculate reprojection error and define new keyframe if necessary.
*****************************/
				vector<Point2d> keyframe_reproj;
				perspectiveTransform( matched_keypoints_moved, keyframe_reproj, H);
				// keyframe_reproj = H * matched_keypoints_moved;
			
				double err = 0;
				for (int i = 0; i < matched_keypoints_moved.size(); i++){
					err += norm(Mat(matched_keypoints_keyframe[i] ), Mat(keyframe_reproj[i] ) );
				}
				err = err / matched_keypoints_moved.size();
				log(LOG_IMPORTANT_INFO) << "Reprojection error is " << err << endl;
				r_err += err;
			
				if (err > (double) cfg.lookup("homography.reprojectionErrorMax") ){
					log(LOG_CRITICAL) << endl << "Error has grown large. New keyframe at " << frameno << " frames with total error of " << r_err << endl;
					// Get a new keyframe	
					center = frame;
					r_err = 0;
					waitKey(0);
					detector.detect( center, raw_keypoints_keyframe );
					//exit(1);
				}
				
				#if(DB)
					out_file << endl << "T values are: " <<  test.ta << "  ||  " << test.tb << endl;
				#endif					 
	
/**********************************************
		Draw the box showing the edges of one picture in the other.
**********************************************/

				//-- Get the corners from the image_1 ( the moving to be "detected" )
				std::vector<Point2f> moved_corners(4);
				moved_corners[0] = cvPoint(0,0);
				moved_corners[1] = cvPoint( frame.cols, 0 );
				moved_corners[2] = cvPoint( frame.cols, frame.rows ); 
				moved_corners[3] = cvPoint( 0, frame.rows );
				std::vector<Point2f> keyframe_corners(4);

				cv::perspectiveTransform( cv::Mat(moved_corners), cv::Mat(keyframe_corners), H);
				// H * moved_mat = keyframe_mat

				//-- Draw lines between the corners (the mapped moving in the keyframe - image_2 )
				line( img_matches, keyframe_corners[0] + Point2f( frame.cols, 0), keyframe_corners[1] + Point2f( frame.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, keyframe_corners[1] + Point2f( frame.cols, 0), keyframe_corners[2] + Point2f( frame.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, keyframe_corners[2] + Point2f( frame.cols, 0), keyframe_corners[3] + Point2f( frame.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, keyframe_corners[3] + Point2f( frame.cols, 0), keyframe_corners[0] + Point2f( frame.cols, 0), Scalar( 0, 255, 0), 4 );
			
				//-- Show detected matches
				imshow( "Good Matches & moving detection", img_matches );
			}
			
			#if CALC_FPS
				double dur = CLOCK()-start; 
				log(LOG_INFO) << "avg time per frame " << avgdur(dur) << " ms. fps " << avgfps() << ". frameno = " << frameno++ << endl;//\n",avgdur(dur),avgfps(),frameno++ );
			#endif
			
			ostringstream fname;
			char sname[15];
			sprintf(sname, "%03d",count++);
			fname << "images/IM" << sname << ".pgm";
			string filename = fname.str();
			
			imshow("Screen", frame);
			
			cvtColor(frame, frame, CV_BGR2GRAY);
			imwrite(filename, frame);
			
			char key = waitKey(10);
			if (key == 0x1B){
				quit_signal=1;
				center = frame;
			}
			else if(key == 'p'){
				while(key != 'c'){
					key = waitKey(30);
				}
			}	
		
		}else{
			continue;
		}
	}
}

void analyzeHomography(cv::Mat H, Eigen::Matrix3d K ){

/**********************************************
		Decompose the homography into R and T
**********************************************/
	Eigen::Matrix3d b;
	cv2eigen(H,b);

	decompose_return test;
	test = homogDecompose(b, K);
	string sep = "--------------------------------------------------------------------------";
	log(LOG_IMPORTANT_INFO) << " Parameter\t|\tValue\n" << sep << "\n";
/**********************************************
		Analyze R and T
**********************************************/
	
	char t1[100];
	char t2[100];
	sprintf(&t1[0], " T1\t\t| %.04g,\t%.04g,\t%.04g", test.ta(0,0), test.ta(0,1), test.ta(0,2) );
	sprintf(&t2[0], " T2\t\t| %.04g,\t%.04g,\t%.04g", test.tb(0,0), test.tb(0,1), test.tb(0,2) );
	
	log(LOG_IMPORTANT_INFO) << t1 << endl << t2<< endl;
	
	sprintf(&t1[0], " T1 control \u2220\t| %.4g\u00B0 ", atan2(test.ta(1), test.ta(0) ) * RADIAN_TO_DEGREE);
	sprintf(&t2[0], " T2 control \u2220\t| %.4g\u00B0 ", atan2(test.tb(1), test.tb(0) ) * RADIAN_TO_DEGREE);
	log(LOG_IMPORTANT_INFO) << t1 << endl << t2 << endl;

	sprintf(&t1[0], " Normal 1\t| %.04g,\t%.04g,\t%.04g", test.na(0,0), test.na(0,1), test.na(0,2) );
	sprintf(&t2[0], " Normal 2\t| %.04g,\t%.04g,\t%.04g", test.nb(0,0), test.nb(0,1), test.nb(0,2) );
	log(LOG_INFO) << t1 << endl << t2 << endl;
	
	angle_triple rAnglesA, rAnglesB;
	rAnglesA = rotation_angles(test.Ra);
	rAnglesB = rotation_angles(test.Rb);
	
	double phi_a = rAnglesA.phi_radians * RADIAN_TO_DEGREE;
	double phi_b = rAnglesB.phi_radians * RADIAN_TO_DEGREE;

	double theta_a = rAnglesA.theta_radians * RADIAN_TO_DEGREE;
	double theta_b = rAnglesB.theta_radians * RADIAN_TO_DEGREE;

	double psi_a = rAnglesA.psi_radians * RADIAN_TO_DEGREE;
	double psi_b = rAnglesB.psi_radians * RADIAN_TO_DEGREE;

	sprintf(&t1[0], " Euler angles 1\t| %.04g\u00B0,\t%.04g\u00B0,\t%.04g\u00B0", phi_a, theta_b, psi_a );
	sprintf(&t2[0], " Euler angles 2\t| %.04g\u00B0,\t%.04g\u00B0,\t%.04g\u00B0", phi_b, theta_b, psi_b );
	log(LOG_INFO) <<  t1 << endl << t2 << endl;
	
	log(LOG_DEBUG) << sep << " R1" << print33Matrix(test.Ra) << sep << " R2" << print33Matrix(test.Rb) << sep << endl;

}

std::string print33Matrix(Eigen::Matrix3d M){
	char t1[200];
	sprintf(&t1[0], "\t\t| %.04g %.04g %.04g\n\t\t| %.04g %.04g %.04g\n\t\t| %.04g %.04g %.04g \n", \
		 M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2) );
		 
	std::string ret(t1);
	return ret;
}

std::string print33Mat(cv::Mat M, std::string name){
	char t1[200];
	sprintf(&t1[0], "\n %s\t| %.04g %.04g %.04g\n\t\t| %.04g %.04g %.04g\n\t\t| %.04g %.04g %.04g \n", name.c_str(), \
		 M.at<double>(0), M.at<double>(1), M.at<double>(2), M.at<double>(3), M.at<double>(4), M.at<double>(5), M.at<double>(6), M.at<double>(7), M.at<double>(8) );
		 
	std::string ret(t1);
	return ret;
}

// TODO : My implementation of getting R and n need a looking at in order to match what the opencv implementation is.
#if(0)				 
  /* Proof of concept: I'm getting the same translation vector up to a scale factor. However, this means that I've implemented the algorithm incorrectly. */
			vector<Mat> rotations, translations, normals;
			Mat K_mat = (Mat_<double>(3,3) << 1200.375, 0, 640, 0, 1200.954, 360, 0, 0, 1);
			decomposeHomographyMat(H, K_mat, rotations, translations, normals); 
		
			cout << "Translations scaled to my implementation: " << endl;
			for (int i = 0; i < translations.size(); i++){
				cout << translations[i].at<double>(0) / test.ta(0) << "; "
					<< translations[i].at<double>(1) / test.ta(1) << "; " 
					<< translations[i].at<double>(2) / test.ta(2) << endl;  
			}
			cout << "Rotations, ocv:" << endl;
			for (int i = 0; i < rotations.size(); i++){
				angle_triple rAng;
				Eigen::Matrix3d b;
				cv2eigen(rotations[i], b);
				rAng = rotation_angles(b);
				//cout << rAng.phi_radians << "  |  " << rAng.theta_radians << "  |  " << rAng.psi_radians << endl;
				cout << "Euler angles:    \u0444: " << rAng.phi_radians * RADIAN_TO_DEGREE 
					<< "\u00B0 \u0398: " << rAng.theta_radians * RADIAN_TO_DEGREE  
					<< "\u00B0 \u03C8: " << rAng.psi_radians * RADIAN_TO_DEGREE << endl;
			}
			cout << "Normals: " << endl;
			for (int i = 0; i < normals.size(); i++){
				cout << normals[i] << endl;
			}
#endif

