#pragma once
#include "stdafx.h"
#include "GridBox.h"
#include <fstream>

void calculate_Wi_Matrices(Mat img, vector<Point2d>& obj, vector<MatrixXd>& vec);
vector<Matrix3d> calculate_CellHomography(vector<MatrixXd>& matrices, MatrixXd& A);
void calculate_Wi_Matrices(Mat img, vector<Point2d>& obj, vector<MatrixXd>& vec, vector<pair<int, int>>& superpixelsCenter);
void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, int C1, int C2);
void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, vector<vector<pair<int, int>>>& superPixels);
void warpImage(const Mat& img_1, const Mat& img_2, Mat& target);

//将需要线性插值的区域画出来
void drawArea(Mat& img, GridBox& gird);

// 对图像进行线性插值
void linearInterImg(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);
void linearInterImg1(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);

// 线性插值辅助函数
void interMat(int i, int j, Mat& img, vector<Point2d>& pointVec);

// （x,y）进行取整处理
void computeX_Y(double& t_x, double& t_y);

// 对图像中的一个像素点进行线性插值
void linearInterImg2(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat);

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat, int ThresholdN);

//辅助画图工具
void drawPointAndRectangular(Mat& img, vector<Point2d>& scene2, vector<pair<int, int>>& superpixelsCenter);