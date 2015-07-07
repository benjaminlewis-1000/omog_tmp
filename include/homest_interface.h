// homest helper / access function. The goal of this file is to get a working interface where I can simply pass a couple of matched vectors in to get a homography

#ifndef HOMEST_INTERFACE_H
#define HOMEST_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <list>
#include <vector>
#include "homest.h"

#include "logging.h" // Logging function

#define MAXSTRLEN	256
#define INL_PCENT 0.7

static int readMatchingPoint2d(std::vector<cv::Point2d> in1, std::vector<cv::Point2d> in2, double (**pts0)[2], double (**pts1)[2]);
void filePointsToVec(char *fname, std::vector<cv::Point2d> *in1, std::vector<cv::Point2d> *in2);
void findHomographyHomest(std::vector<cv::Point2d> in1, std::vector<cv::Point2d> in2, double* Hret);

static int readMatchingPoint2d(std::vector<cv::Point2d> in1, std::vector<cv::Point2d> in2, double (**pts0)[2], double (**pts1)[2]){
  *pts0=(double (*)[2])malloc(in1.size()*sizeof(double[2]));
  *pts1=(double (*)[2])malloc(in1.size()*sizeof(double[2]));
// There should definitely be error checking and bounding. 

	if (in1.size() != in2.size() ){	
      fprintf(stderr, "The two point vectors are not the same size\n");
      exit(1);
	}
	if(in1.size() == 0 || in2.size() == 0){
		fprintf(stderr, "One or both of the input vectors has no elements.\n");
	}
	for (int i = 0; i < in1.size(); i++){
		*((*pts0)[i]  )  = in1[i].x;
		*((*pts0)[i]+1) = in1[i].y;
		*((*pts1)[i]  )  = in2[i].x;  // Hooray for the magic of pointers. 
		*((*pts1)[i]+1) = in2[i].y;
	}
}

void filePointsToVec(char *fname, std::vector<cv::Point2d> *in1, std::vector<cv::Point2d> *in2){
// A method to read points from a file into a vector<Point2d>. Mostly used to check accuracy of my readMatchingPoint2d method. 
	register int i;
	int ncoords, nmatches;
	double coords[4];
	FILE *fp;
	char buf[MAXSTRLEN];
	if((fp=fopen(fname, "r"))==NULL){
		fprintf(stderr, "cannot open file %s\n", fname);
		exit(1);
	}

	fgets(buf, MAXSTRLEN, fp);
	if(ferror(fp)){
 		fprintf(stderr, "File %s: error reading first line\n", fname);
		exit(1);
	}
	log(LOG_DEBUG) << buf << std::endl;
	
	ncoords=sscanf(buf, "%lf%lf%lf%lf", coords, coords+1, coords+2, coords+3);
	log(LOG_DEBUG) << coords[2] << std::endl;
	
	for(i=0; !feof(fp); i++){
		ncoords=fscanf(fp, "%lf%lf%lf%lf\n", coords, coords+1, coords+2, coords+3);
		//std::cout << coords[2] << std::endl;
		//cv::Point2d pt1(coords[0], coords[1]);
		in1->insert(in1->end(), cv::Point2d(coords[0], coords[1]) );
		in2->insert(in2->end(), cv::Point2d(coords[2], coords[3]) );
		if(ncoords==EOF) break;

		if(ncoords!=4){
		  fprintf(stderr, "File %s: line %d contains only %d coordinates\n", fname, i + 1, ncoords);
		  exit(1);
  		}

		if(ferror(fp)){
		  fprintf(stderr, "File %s: error reading 2D point coordinates, line %d\n", fname, i + 1);
		  exit(1);
		}

  }
}

void findHomographyHomest(std::vector<cv::Point2d> in1, std::vector<cv::Point2d> in2, double* Hret){  // Must be renamed so as not to conflict with OpenCV
	double (*pts0)[2], (*pts1)[2];
	register int i;
	int npts, donorm, noutl, *outidx=NULL;
	char *matchesfile;
	double H[NUM_HPARAMS], rms, rmeds;
	int estAffine=0;
	
	log(LOG_DEBUG) << in1.size() << "  " << in2.size() << std::endl;
	readMatchingPoint2d(in1, in2, &pts0, &pts1);
	
	clock_t start_time, end_time;

	npts=in1.size(); // & in2.size() ; // This should give the same number if they are both same size, but it's hacky, obscure, and risky to say the least. 
	
	#ifdef NEED_OUTLIERS
		if((outidx=(int *)malloc(npts*sizeof(int)))==NULL){
			fprintf(stderr, "Memory allocation request failed in main()\n");
			exit(1);
		}
	#endif /* NEED_OUTLIERS */

	donorm=1;

	log(LOG_DEBUG) << (estAffine? "Affine h" : "H") << "omography estimation using " << npts << " image matches\n";
	//fprintf(stdout, "%somography estimation using %d image matches\n", estAffine? "Affine h" : "H", npts);

	start_time=clock();
	if(!estAffine){
    	int cstfunc;

		cstfunc=HOMEST_SYM_XFER_ERROR; // use the symmetric transfer error
		//cstfunc=HOMEST_XFER_ERROR; // use the transfer error in 2nd image
		//cstfunc=HOMEST_SAMPSON_ERROR; //use the Sampson error
		//cstfunc=HOMEST_REPR_ERROR; // use the reprojection error
		//cstfunc=HOMEST_NO_NLN_REFINE; // no refinement
		homest(pts0, pts1, npts, INL_PCENT, Hret, donorm, cstfunc, outidx, &noutl, 1);
  	}
	else{
		homestaff(pts0, pts1, npts, INL_PCENT, Hret, donorm, outidx, &noutl, 1);
	}
	end_time=clock();

	log(LOG_DEBUG) << "Estimated " << (estAffine? "affine " : "") << "homography [" << noutl << " outliers, " << (double)(100.0*noutl)/npts << "]\n";
	//fprintf(stdout, "\nEstimated %shomography [%d outliers, %.2lf%%]\n", estAffine? "affine " : "", noutl, (double)(100.0*noutl)/npts);
	/*for(i=0; i<NUM_HPARAMS; ++i){
		if(!(i%3)) fprintf(stdout, "\n");
			fprintf(stdout, "%.7g ", H[i]);
		}*/
		
	cv::Mat A = cv::Mat(3,3,CV_64FC1, &Hret);
	log(LOG_DEBUG) << std::endl <<  A << std::endl;
	
	log(LOG_DEBUG) << "\nElapsed time: " << ((double) (end_time - start_time)) / CLOCKS_PER_SEC << " seconds, " 
		<< ((double) (end_time - start_time)) / (CLOCKS_PER_SEC/1000.0) << " msecs\n";
	/*fprintf(stdout, "\nElapsed time: %.2lf seconds, %.2lf msecs\n", ((double) (end_time - start_time)) / CLOCKS_PER_SEC,
 		                ((double) (end_time - start_time)) / (CLOCKS_PER_SEC/1000.0));*/

	homest_RMS_RMedS(pts0, pts1, npts, H, &rms, &rmeds);
	log(LOG_DEBUG) << "\nHomography RMS and RMedS errors for input points: " << rms << "  " << rmeds << std::endl;
	//fprintf(stdout, "\nHomography RMS and RMedS errors for input points: %g %g\n", rms, rmeds);
	//fflush(stdout);

	{ double q1, q2, q3;

	homest_quartiles(pts0, pts1, npts, H, &q1, &q2, &q3);
	log(LOG_DEBUG) << "\t25%%, 50%% and 75%% quartiles: " << q1 << "  " << q2 << "  " << q3 << std::endl;
//	fprintf(stdout, "\t25%%, 50%% and 75%% quartiles: %g %g %g\n", q1, q2, q3);
//	fflush(stdout);
	}

#ifdef NEED_OUTLIERS
	fprintf(stdout, "Indices of the %d outlier pairs:\n", noutl);
	for(i=0; i<noutl; ++i)
		fprintf(stdout, "%d ", outidx[i]);
	fputc('\n', stdout);

	if(outidx) free(outidx);
#endif /* NEED_OUTLIERS */

	free(pts0);
	free(pts1);
	
	//return A;
	
}

#endif
