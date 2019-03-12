#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits.h>
#include <cmath>
#include <Eigen/Dense>
#include <vl/generic.h>
#include <vl/slic.h>
#include <vl/sift.h>
#include "stdafx.h"
#include <fstream>
#include <string>
/*
extern "C" {
#include <vl/generic.h>
#include <vl/slic.h>
#include <vl/sift.h>
}
*/
#define  _USE_MATH_DEFINES
#include <cmath>
using namespace cv;
using namespace std;
using namespace Eigen;

// double gamma = 0.0025;
// double sigma = 12.0;
// int C1 = 50, C2 = 50;//Grid count
extern double gamma;
extern double sigma;
extern int C1, C2;//Grid count
extern double SIFT_Threshold; //Threshold: used to filter pairs of features that have already been matched
extern int Sampling_Rate; //Sampling rate: sampling more matching pairs
extern int Sp_Size; //Seed value: divide superpixel
extern double RANSAC_Threshold;
extern int width, height;//warp image
extern int x_offset, y_offset;

void readFile(char* fileName);