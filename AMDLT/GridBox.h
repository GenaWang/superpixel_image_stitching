#pragma once
#include "stdafx.h"
class GridBox
{
public:
	double vertx[4], verty[4];
	GridBox(cv::Point2d tl, cv::Point2d tr, cv::Point2d bl, cv::Point2d br);
	GridBox();
	~GridBox();
	bool contains(double x, double y);
};