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

//����Ҫ���Բ�ֵ�����򻭳���
void drawArea(Mat& img, GridBox& gird);

// ��ͼ��������Բ�ֵ
void linearInterImg(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);
void linearInterImg1(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);

// ���Բ�ֵ��������
void interMat(int i, int j, Mat& img, vector<Point2d>& pointVec);

// ��x,y������ȡ������
void computeX_Y(double& t_x, double& t_y);

// ��ͼ���е�һ�����ص�������Բ�ֵ
void linearInterImg2(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat);

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat);

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat, int ThresholdN);

//������ͼ����
void drawPointAndRectangular(Mat& img, vector<Point2d>& scene2, vector<pair<int, int>>& superpixelsCenter);