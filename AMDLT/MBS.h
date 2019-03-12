// [10/26/2017 Yinlin]
// Implementation of "Minimum Barrier Superpixel Segmentation"
// huyinlin@gmail.com

#ifndef _MBS_H_
#define _MBS_H_

#include <vector>

#include "opencv2/opencv.hpp"

using namespace std;

// Minimum Barrier Superpixel Segmentation
class MBS
{
public:
	MBS();
	~MBS();

	/************************************************************************/
	/* parameter setting functions                                          */
	/************************************************************************/
	// control compactness, small alpha leads to more compact superpixels,
	// [0-1] is fine, the default is 0.1 which is suitable for most cases
	void SetAlpha(double alpha);

	// set the average size of superpixels
	void SetSuperpixelSize(int spSize);

	/************************************************************************/
	/* do the over-segmentation                                             */
	/************************************************************************/
	int SuperpixelSegmentation(cv::Mat& image);

	/************************************************************************/
	/* utility functions                                                    */
	/************************************************************************/
	int* GetSuperpixelLabels();
	cv::Mat GetSeeds();
	cv::Mat GetSuperpixelElements();

	cv::Mat Visualization();
	cv::Mat Visualization(cv::Mat& image);
	void showLabels();
	cv::Mat Visualization1();
	void getSuperPixels(vector<vector<pair<int, int>>>& superPixels);
	void computeSuperPixelCenter(vector<vector<pair<int, int>>>& superPixels, vector<pair<int,int>>& superpixelsCenter);
	int computeSuperPixelsSize(vector<vector<pair<int, int>>>& superPixels);
	void dilateSuperPixels(vector<vector<pair<int, int>>>& superPixels, vector<vector<pair<int, int>>>& dilateedSuperPixels, vector<vector<bool>>& dilateStruct);

public:
	void DistanceTransform_MBD(cv::Mat& image, float* seedsX, float* seedsY,
		int cnt, int* labels, float* dmap, float factor, int iter = 4);
	int FastMBD(cv::Mat& img, int* labels, int spSize, int outIter, int inIter,
		float alpha, float* seedsX, float* seedsY, int cnt);
	void MergeComponents(int* ioLabels, int w, int h);
	pair<int,int> computePointsCenter(vector<pair<int, int>>& points);
	double _alpha;
	int _spSize;

	int* _labels;

	cv::Mat _seeds;

	int _imgWidth;
	int _imgHeight;
	int _spCnt;
};

void dilateSuperPixel(int height, int weight, vector<pair<int, int>>& superPixel, vector<vector<bool>>& dilateStruct, vector<pair<int, int>>& dilatedSuperPixel);

#endif // _MBS_H_
