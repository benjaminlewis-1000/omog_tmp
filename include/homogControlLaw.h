// Header file implementing the homography-only control from the paper "Homography-based 2D visual tracking and servoing," by S. Benhimane and E. Malis

#ifndef HOMOG_CONTROL_LAW_H
#define HOMOG_CONTROL_LAW_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <vector>

// Dependent on an isomorphism between the task function and the camera pose means that we are able to define motion commands based on the homography.

// Tasks: 
// - Find center of mass of both clouds of points by 1/n * sum(point)_1^n.  Points are of the form [x, y, 1]. 
// NOTE: These are not pixel locations - they are K^-1 * pixel location. Thanks to the wonders of distributive multiplication, I can find the centroid in pixels, then multiply it by K^-1 rather than multiplying each point individually.

struct command_return{
	Eigen::MatrixXd e_v;
	Eigen::MatrixXd e_wx;
};

Eigen::MatrixXd centerOfMass(std::vector<cv::Point2d> points);
Eigen::MatrixXd controlCompute(Eigen::Matrix3d H, Eigen::Matrix3d K, std::vector<cv::Point2d> currentPoints, std::vector<cv::Point2d> targetPoints, double weightT, double weightOmega);
command_return controlCompute(Eigen::Matrix3d H, Eigen::Matrix3d K, Eigen::MatrixXd currentCOM, Eigen::MatrixXd targetCOM, double weightT, double weightOmega);

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

// Inputs: H, K, weightT, weightOmega, image points of target image, image points of current image.
Eigen::MatrixXd controlCompute(Eigen::Matrix3d H, Eigen::Matrix3d K, std::vector<cv::Point2d> currentPoints, std::vector<cv::Point2d> targetPoints, double weightT, double weightOmega){
	Eigen::MatrixXd CMC = centerOfMass(currentPoints);  // Current Mass Center 
	Eigen::MatrixXd TMC = centerOfMass(targetPoints);  // Target Mass Center
	
	CMC = K.inverse() * CMC;  // m = K^{-1} * p, where p is image coordinates and m are projections coordinates. The '1' in the third index is preserved.
	TMC = K.inverse() * TMC;
	// Notation: in the paper, the * is the reference or target frame, while non-* variables are the current frame.
	// m = Hm*
	// Current = H * Reference (/target)
	// Therefore need a homography generated by findHomography(target, current)
	// Or, using homest, 
	
	Eigen::MatrixXd num = (CMC.transpose() * H * TMC);
	Eigen::MatrixXd denom = (CMC.transpose() * CMC);
	double mult = num(0,0) / denom(0,0);
	Eigen::MatrixXd e_v = ( mult * CMC ) - TMC;
	Eigen::Matrix3d e_wx = H - H.transpose();  // Skew-symmetric matrix, defined per the paper as [a]_x = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0]
	// Therefore, a 3x1 vector of e_w would be [e_wx(3,2), e_wx(1,3), e_wx(2,1)].
	
	
	return e_v;
}

command_return controlCompute(Eigen::Matrix3d H, Eigen::Matrix3d K, Eigen::MatrixXd currentCOM, Eigen::MatrixXd targetCOM, double weightT, double weightOmega){
	//Eigen::MatrixXd CMC = centerOfMass(currentPoints);  // Current Mass Center 
	//Eigen::MatrixXd TMC = centerOfMass(targetPoints);  // Target Mass Center
	
	currentCOM = K.inverse() * currentCOM;  // m = K^{-1} * p, where p is image coordinates and m are projections coordinates. The '1' in the third index is preserved.
	targetCOM = K.inverse() * targetCOM;
	// Notation: in the paper, the * is the reference or target frame, while non-* variables are the current frame.
	// m = Hm*
	// Current = H * Reference (/target)
	// Therefore need a homography generated by findHomography(target, current)
	// Or, using homest, 
	
	Eigen::MatrixXd num = (currentCOM.transpose() * H * targetCOM);
	Eigen::MatrixXd denom = (currentCOM.transpose() * currentCOM);
	double mult = num(0,0) / denom(0,0);
	Eigen::MatrixXd e_v = ( mult * currentCOM ) - targetCOM;
	Eigen::Matrix3d e_wx = H - H.transpose();  // Skew-symmetric matrix, defined per the paper as [a]_x = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0]
	// Therefore, a 3x1 vector of e_w would be [e_wx(3,2), e_wx(1,3), e_wx(2,1)].
	
	command_return retVal;
	retVal.e_v = e_v;
	retVal.e_wx = e_wx;
	return retVal;
}

#endif
