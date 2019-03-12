#include <iostream>
#include <time.h>
#include "SIFT_Matcher.h"
#include "Homography.h"
#include "APAP_Processor.h"
#include "RANSAC_Processor.h"
#include "MathUtils.h"
#include "MBS.h"
#include "stdafx.h"

using namespace cv;
using namespace std;
using namespace Eigen;

// #define DEBUG_YL

int main(int argc, char* argv[]){

	// read image
	clock_t time1, time2;
	cout << "load image..." << endl;
	// time1 = clock();
	vector<char*> imagename;
	imagename.resize(2);
	vector<Mat> img;
	img.resize(2);
	for (int i = 0;i < 2;i++) {
		imagename[i] = argv[i + 1];
		img[i] = imread(imagename[i]);
	}
	readFile("config.txt");
	cout << gamma << " " << sigma << " " << C1 << " " << C2 << endl;
	if (img[0].empty() || img[1].empty()) {
		cout << "the images can not be loaded" << endl;
		return 1;
	}

	vector<DMatch> matches;
	// vector<KeyPoint> keypoints_1, keypoints_2;
	vector<vector<KeyPoint>> keypoints;
	vector<KeyPoint> keyPoints_obj1, keyPoints_scene2;
	vector<vector<KeyPoint>> keyPoints_obj;
	vector<Point2d> obj1, scene2;//特征点

	// compute SIFT points and match them
	cout << "SIFT Feature points..." << endl;
	// findFeaturePointsWithSIFT(img_1, img_2, keypoints_1, keypoints_2, matches);
	findFeaturePointsWithSIFT(img, keypoints, matches);
	cout << "Using RANSAC filtering matches..." << endl;
	// img_1<->keyPoints_obj（图像1的特征点）,  img_2<->keyPoints_scene（图像2的特征点）
	// RANSAC_filter(img_1, img_2, keypoints_1, keypoints_2, keyPoints_obj1, keyPoints_scene2, matches);
	RANSAC_filter(img[0], img[1], keypoints[0], keypoints[1], keyPoints_obj1, keyPoints_scene2, matches);

	MatrixXd A;
	Matrix3d H;
	vector<MatrixXd> Wi_Vector;
	vector<Matrix3d> H_vectors;
	
	//  compute homography(img_2->img_1)
	cout << "Calculating homography..." << endl;
	// getHomography(keyPoints_scene, keyPoints_obj, scene, obj, H, A);
	// compute homography(img_1->img_2)
	getHomography(keyPoints_scene2, keyPoints_obj1, scene2, obj1, H, A);


	//**************************************************
	// Calculate the weight matrix for each superpixel
	//***************************************************

	// step1: Divide the target image into superpixels
	cout << "Divide the target image into superpixels..." << endl;
	// cv::Mat img = cv::imread(argv[2]); //iamge2 is target
	int spSize = Sp_Size;
	double alpha = 0.1;

	MBS mbs;
	mbs.SetSuperpixelSize(spSize);
	mbs.SetAlpha(alpha);
	int spCnt = mbs.SuperpixelSegmentation(img[1]);
	cout << "spCnt is:" << spCnt << endl;
	cv::Mat spVisual = mbs.Visualization(img[1]);
	cv::imwrite("MBS_out.jpg", spVisual);

	vector<vector<pair<int, int>>> superPixels;
	vector<pair<int, int>> superpixelsCenter;
	mbs.getSuperPixels(superPixels);
	mbs.computeSuperPixelCenter(superPixels, superpixelsCenter);
	drawPointAndRectangular(spVisual, scene2, superpixelsCenter);
	// int superPixelsSize = mbs.computeSuperPixelsSize(superPixels);
	// cout << "superPixelsSize:" << superPixelsSize << endl; // There are no missing superpixels.

	// step2: Calculate the weight matrix of superpixels in an image
	time1 = clock();
	cout << "Calculate the weight matrix of superpixels in an image..." << endl;
	calculate_Wi_Matrices(img[1], scene2, Wi_Vector, superpixelsCenter);

	//**************************************************
	// Calculate the homography of each superpixel
	//***************************************************

	cout << "Calculate the homography of each superpixel..." << endl;
	H_vectors = calculate_CellHomography(Wi_Vector, A);
	cout << "the size of H_vectors is:" << H_vectors.size() << endl;

	//*******************************************************************************
	// Each superpixel is transformed using the corresponding single-matrix matrix
	//*********************************************************************************

	Mat homography_target, display;
	ConvertImage(img[1], homography_target, H_vectors, superPixels);

	// time2 = clock();
	warpImage(img[0], homography_target, display);
	time2 = clock();
	cout << "time1 to time2:" << (time2 - time1) << endl;

	return 0;
}