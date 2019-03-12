#include "RANSAC_Processor.h"

void findGoodMatches(vector<cv::DMatch> matches, vector<cv::DMatch>& good_matches)
{
	double min_dist;
	for (int i = 0; i < matches.size(); i++)
	{
		double match_dist = matches[i].distance;
		if (i == 0)
			min_dist = match_dist;
		min_dist = match_dist < min_dist ? match_dist : min_dist;
	}

	for (int i = 0; i < matches.size(); i++)
		if (matches[i].distance <= SIFT_Threshold * min_dist)
			good_matches.push_back(matches[i]);
}

//RANSAC剔除误匹配
void RANSAC_filter(cv::Mat img1, cv::Mat img2, vector<cv::KeyPoint> keypoints_1, vector<cv::KeyPoint> keypoints_2,
	vector<cv::KeyPoint>& RR_keypoint01, vector<cv::KeyPoint>& RR_keypoint02, vector<cv::DMatch> raw_matches)
{
	vector<cv::KeyPoint> R_keypoint01, R_keypoint02;
	vector<cv::DMatch> matches;
	vector<cv::DMatch> matchesCopy;
	findGoodMatches(raw_matches, matches);
	for (int i = 0; i < matches.size(); i++)
	{
		R_keypoint01.push_back(keypoints_1[matches[i].queryIdx]);
		R_keypoint02.push_back(keypoints_2[matches[i].trainIdx]);
		matchesCopy.push_back(matches[i]);
		matchesCopy[i].queryIdx = i;
		matchesCopy[i].trainIdx = i;
	}
	Mat second_match;
	drawMatches(img1, R_keypoint01, img2, R_keypoint02, matchesCopy, second_match);
	imwrite("SecondGoodMatch.jpg", second_match);

	//坐标转换
	vector<cv::Point2d> p01, p02;//p01 -> object; p02 -> scene
	for (int i = 0; i < matches.size(); i++)
	{
		p01.push_back(R_keypoint01[i].pt);
		p02.push_back(R_keypoint02[i].pt);
	}

	//利用基础矩阵剔除误匹配点
	vector<cv::DMatch> goodMatches;
	vector<uchar> RansacStatus;
	// cv::Mat Fundamental = cv::findHomography(p02, p01, RansacStatus, CV_FM_RANSAC); //(阈值CV_FM_RANSAC=8，通常该设置为1到8)
	cv::Mat Fundamental = cv::findHomography(p02, p01, RansacStatus, RANSAC, RANSAC_Threshold);
	int index = 0;
	for (int i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_keypoint01.push_back(R_keypoint01[i]);
			RR_keypoint02.push_back(R_keypoint02[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			goodMatches.push_back(matches[i]);
			index++;
		}
	}
	cout << "good match count = " << index << endl;
	cv::Mat display_match;
	cv::drawMatches(img1, RR_keypoint01, img2, RR_keypoint02, goodMatches, display_match);
	cv::imwrite("ThirdGoodMatch.jpg", display_match);
}

void NO_RANSAC_filter(cv::Mat img1, cv::Mat img2, vector<cv::KeyPoint> keypoints_1, vector<cv::KeyPoint> keypoints_2,
	vector<cv::KeyPoint>& RR_keypoint01, vector<cv::KeyPoint>& RR_keypoint02, vector<cv::DMatch> raw_matches)
{
	//vector<cv::KeyPoint> R_keypoint01, R_keypoint02;
	vector<cv::DMatch> matches;
	vector<cv::DMatch> matchesCopy;
	findGoodMatches(raw_matches, matches);
	for (int i = 0; i < matches.size(); i++)
	{
		RR_keypoint01.push_back(keypoints_1[matches[i].queryIdx]);
		RR_keypoint02.push_back(keypoints_2[matches[i].trainIdx]);
		matchesCopy.push_back(matches[i]);
		matchesCopy[i].queryIdx = i;
		matchesCopy[i].trainIdx = i;
	}
	Mat second_match;
	drawMatches(img1, RR_keypoint01, img2, RR_keypoint02, matchesCopy, second_match);
	//imshow("first_match ", first_match);
	imwrite("SecondGoodMatch.jpg", second_match);
}