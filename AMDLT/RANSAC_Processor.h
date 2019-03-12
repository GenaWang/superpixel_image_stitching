#pragma once
#include "stdafx.h"
#include "MathUtils.h"
void RANSAC_filter(cv::Mat img1, cv::Mat img2, vector<cv::KeyPoint> keypoints_1, vector<cv::KeyPoint> keypoints_2,
	vector<cv::KeyPoint>& RR_keypoint01, vector<cv::KeyPoint>& RR_keypoint02, vector<cv::DMatch> raw_matches);

void NO_RANSAC_filter(cv::Mat img1, cv::Mat img2, vector<cv::KeyPoint> keypoints_1, vector<cv::KeyPoint> keypoints_2,
	vector<cv::KeyPoint>& RR_keypoint01, vector<cv::KeyPoint>& RR_keypoint02, vector<cv::DMatch> raw_matches);