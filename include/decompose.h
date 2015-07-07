/*****************************************************
*
*  Homography decomposition function
*  Implemented by Benjamin Lewis, May 2015
*  Based on the paper "Deeper understanding of the homography decomposition for vision-based control"
*  Paper available at https://hal.inria.fr/inria-00174036v3/document
*   - Function: homogDecompose:
*		- Decompose a homography matrix H given a intrinsic parameter matrix K
*		- Return a geometeric rotation, translation, and planar normal, up to a scale factor. 
*	*** Helper Functions ***
*	- Function: rotation_angles:
*		- Helper function that decomposes a rotation matrix into Euler angles
*	- Function: normalize:
*		- Normalize a 3x1 vector
*	- Function: submatrix:
*		- Return a submatrix of a 3x3 matrix. Inputs are the row and column to be excluded (one-indexed).
*		- Returns a 2x2 submatrix.
*
*****************************************************/

// TODO: Put in the opposites as well, i.e. (Ra,-ta,-na) and (Rb, -tb, -nb)
// Header file for homography decomposition, which includes the decomposition and 
// all associated helper functions. 

#ifndef HOMOG_DECOMPOSE_H
#define HOMOG_DECOMPOSE_H

#include <Eigen/Dense>
#include <Eigen/SVD>

// All constructs must use doubles, not floats, in the interest of numerical accuracy.
struct decompose_return{
	Eigen::Matrix3d Ra;
	Eigen::Matrix3d Rb;
	Eigen::MatrixXd ta;
	Eigen::MatrixXd tb;
	Eigen::MatrixXd na;
	Eigen::MatrixXd nb;
};

struct angle_triple{
	double phi_radians;
	double theta_radians;
	double psi_radians;
};

angle_triple  rotation_angles(Eigen::Matrix3d R);
Eigen::MatrixXd submatrix(Eigen::MatrixXd matrix, int indexRow, int indexColumn);
int sign(double number);
Eigen::MatrixXd normalize(Eigen::MatrixXd mat);
decompose_return computeHomographyDecomposition(Eigen::Matrix3d H, Eigen::Matrix3d K);

Eigen::MatrixXd submatrix(Eigen::MatrixXd matrix, int indexRow, int indexColumn){
	// Returns the original matrix without the row of indexRow and column
	// of indexColumn. submatrix([1,2,3;4,5,6;7,8,9], 1, 1) will return
	// [1, 3; 7, 9].
	// Make it zero-indexed for consistency with C++ and Eigen convention.
	
	int rows = matrix.rows() - 1;
	int cols = matrix.cols() - 1;
	
	if (indexRow > rows || indexColumn > cols){ // Then it's a 
	 		// problem since we're trying to take out an index tat doesn't 
	 		// exist in the input matrix.
		exit(-1);
	}

	Eigen::MatrixXd ret(rows, cols);

	// If we're past the row or column that we want to exclude from the submatrix, then
	// we still want to pick the appropriate value from matrix (input) but subtract one from
	// the row/column value that we're going to be assigning that value to in ret. 
	for (int i = 0; i < matrix.rows(); i++){
		for (int j = 0; j < matrix.rows(); j++){
			if (i != indexRow && j != indexColumn){
				int jIndex = (j < indexColumn) ? j : j - 1;
				int iIndex = (i < indexRow) ? i : i - 1;
				//cout << "i " << iIndex << " j " << jIndex << endl;
				ret(iIndex, jIndex) = matrix(i, j);
			}
		}
	}
	
	//std::cout << "submatrix is " << ret << std::endl;
	
	//std::cout << "Determinant is " << ret.determinant() << std::endl;
	
	return ret;
}

int sign(double number){
// sgn=@(a)2*(a>=0)-1 -- MATLAB
// Gets the sign of the number, 0 being positive (+1). 
	int num = 2 * (number >= 0) - 1;
	return num;
}

Eigen::MatrixXd normalize(Eigen::MatrixXd mat){
// Vector divided by the norm of the vector. 
	return mat / mat.norm();
}

// Decompose a 3x3 homography with a 3x3 intrinsic camera calibration matrix, returning
// two valid solutions, R/n/t a and b. Method is from paper "A deeper understanding of the 
// homography decomposition for vision-based control." 

// Method is that of computing the translation vector first. 
// Of note from the paper, the most well-conditioned decomposition will be determined by looking 
// at the s_ii elements of S. The one with the largest absolute value corresponds to the most well-
// conditioned decomposition. 
decompose_return computeHomographyDecomposition(Eigen::Matrix3d H, Eigen::Matrix3d K){
	H = K.inverse() * H * K;  // This becomes equal to R + t * n'
	// Transform the projective homography H, obtained from findHomography or similar, into the Euclidean homography which can be decomposed into R,T,n.
	
	// Take the SVD
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(H);
	
	Eigen::MatrixXd svs(3,1); // Array of singular values
	svs = svd.singularValues();
	
	// Find the median singular value, just by deciding which value is in between the other
	// two values. It's probably the middle value from the .singularValues function, but 
	// it doesn't hurt to check. 
	double medianSVD;
	if ( (svs(1) > svs(0) && svs(1) < svs(2)) || (svs(1) < svs(0) && svs(1) > svs(2) ) )
		medianSVD = svs(1);
	if ( (svs(0) > svs(2) && svs(0) < svs(1)) || (svs(0) < svs(2) && svs(0) > svs(1) ) )
		medianSVD = svs(0);
	if ( (svs(2) > svs(0) && svs(2) < svs(1)) || (svs(2) < svs(0) && svs(2) > svs(1) ) )
		medianSVD = svs(2);
		
	//std::cout << "SVDs: " << svs(0) << "  " << svs(1) << "  " << svs(2) << "   " << medianSVD << std::endl; // Good to here, 
	// except minor numerical discrepencies. 
		
	// Divide H by the median singular value so that it has... uh... wait... H isn't in SL(3) group any more.
	H = H / medianSVD;
	Eigen::MatrixXd eye3;
	eye3 = Eigen::MatrixXd::Identity(3, 3);
	Eigen::MatrixXd Sr(3,3);
	Sr = H * H.transpose() - eye3;
	//std::cout << "H_mark is " << std::endl << Sr << std::endl << std::endl;
	
	double sr11 = Sr(0, 0);
	double sr12 = Sr(0, 1);
	double sr13 = Sr(0, 2);
	double sr21 = Sr(1, 0);
	double sr22 = Sr(1, 1);
	double sr23 = Sr(1, 2);
	//double sr31 = Sr(2, 0);
	//double sr32 = Sr(2, 1);
	double sr33 = Sr(2, 2);

	double M_sr11 = (-1 * submatrix(Sr,0,0).determinant() ); // This needs to be fixed.
	double M_sr22 = (-1 * submatrix(Sr,1,1).determinant() ); // Somehow the abs function makes the two functions identical.
	double M_sr33 = (-1 * submatrix(Sr,2,2).determinant() );

	double M_sr13 = -1 * submatrix(Sr,0,2).determinant();
	double M_sr23 = -1 * submatrix(Sr,1,2).determinant();
	double M_sr12 = -1 * submatrix(Sr,0,1).determinant();

	int er23 = sign(M_sr23);
	int er13 = sign(M_sr13);
	int er12 = sign(M_sr12);

// Find the largest value:

	char well_condition;

	if (abs(sr11) >= abs(sr22) && abs(sr11) >= abs(sr33) ){
		well_condition = 1;
	}else if (abs(sr22) >= abs(sr11) && abs(sr22) >= abs(sr33) ){
		well_condition = 2;
	}else{ well_condition = 3; }

	double e_srii; 
	double srii; 
	Eigen::MatrixXd ta_sr(3,1), tb_sr(3,1);

// The largest diagonal value determines which method should be used for finding the ta and tb values. 
	switch(well_condition){
		case 1:  // sr11 is the largest value on the diagonal and thus we should use ta(sr11) and tb(sr11).
			ta_sr << sr11, sr12 + sqrt(M_sr33), sr13 + er23 * sqrt( (M_sr22) );
			tb_sr << sr11, sr12 - sqrt(M_sr33), sr13 - er23 * sqrt( (M_sr22) );
			e_srii = sign(M_sr11);
			srii = sr11;
			break;

		case 2:
			ta_sr << sr12 + sqrt(M_sr33), sr22, sr23 - er13 * sqrt( (M_sr11) );
			tb_sr << sr12 - sqrt(M_sr33), sr22, sr23 + er13 * sqrt( (M_sr11) );
			e_srii = sign(M_sr22);
			srii = sr22;
			break;

		case 3:
			ta_sr << sr13 + er12 * sqrt(M_sr22), sr23 + sqrt( (M_sr11) ), sr33;
			tb_sr << sr13 - er12 * sqrt(M_sr22), sr23 - sqrt( (M_sr11) ), sr33;
			e_srii = sign(M_sr33);
			srii = sr33;
			break;

		default:
			break;
	}

	double nu_r = 2.0 * sqrt(abs(1+Sr.trace() - M_sr11 -  M_sr22 - M_sr33) );
	double norm_te_r_squared = 2.0 + Sr.trace() - nu_r;
	double norm_te_r = sqrt(norm_te_r_squared);
	double rho_r = sqrt(2.0 + Sr.trace() + nu_r);// These four are good
	
	//std::cout << "Check: " << nu_r << "  " << norm_te_r_squared << "  " << norm_te_r << "  " << rho_r << std::endl;
	
	Eigen::MatrixXd tar(3,1), tbr(3,1);
	tar = normalize(ta_sr) * norm_te_r;
	tbr = normalize(tb_sr) * norm_te_r;
   // std::cout << tar_11 << std::endl;
	
	Eigen::MatrixXd nar(3,1), nbr(3,1); 
	nar = 0.5 * (sign(srii) * rho_r / norm_te_r * tbr - tar );
    nbr = 0.5 * (sign(srii) * rho_r / norm_te_r * tar - tbr ); /// Shouldn't have to normalize this, I guess. 
	//std::cout << nar_11 << " ta  tb" << nbr_11 << " sr22  " << M_sr22<< std::endl;
    
    // Answers to the decomposition
    decompose_return rVal;
    
    rVal.Ra = (eye3 - (2/ nu_r) * tar * nar.transpose() ) * H;
    rVal.Rb = (eye3 - (2/ nu_r) * tbr * nbr.transpose() ) * H;
    
    
    rVal.ta = tar.transpose();
    rVal.tb = tbr.transpose();
    rVal.na = (rVal.Ra.transpose() * nar).transpose();
    rVal.nb = (rVal.Rb.transpose() * nbr).transpose();
        
    return rVal;
}

angle_triple  rotation_angles(Eigen::Matrix3d R){
	double phi = atan2(R(2,1), R(2,0) );
	double theta = atan2(-R(2,0), sqrt( pow(R(2,1),2) + pow(R(2,2),2) ) );
	double psi = atan2( R(1,0) , R(0,0) );
	
	angle_triple rVal;
	rVal.phi_radians = phi;
	rVal.theta_radians = theta;
	rVal.psi_radians = psi;
	
	return rVal;
}

#endif
