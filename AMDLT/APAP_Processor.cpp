#include "APAP_Processor.h"
#include "MathUtils.h"
/*
* \param points 特征点集
* \param x y x*的坐标
*/
using namespace cv;
using namespace std;
using namespace Eigen;
vector<Point2d> points;
MatrixXd calculate_Wi_forPoint(double x, double y)
{
	// const double sigma_squared = sigma * sigma;
	double sigma_squared = sigma * sigma;
	MatrixXd Wi(2 * points.size(), 2 * points.size());
	Wi.setZero();
	int countTest = 0;
#pragma omp parallel for
	for (int i = 0; i < points.size(); i++)
	{
		double u = (double)points[i].x, v = (double)points[i].y;
		double sqr_dist = getSqrDist(x, y, u, v);
		// cout << "sqr_dist" << sqr_dist << endl;
		double candidate = exp(-sqr_dist / sigma_squared);
		// cout << "candidate:" << candidate << endl;
		double omega_i = max(candidate, gamma);
		// cout << "omega_i:" << omega_i << endl;
		if (omega_i == gamma)
			countTest += 1;
		Wi(i * 2, i * 2) = omega_i;
		Wi(i * 2 + 1, i * 2 + 1) = omega_i;
	}
	// cout << "total points:" << points.size() << endl;
	// cout << "countTest:" << countTest << endl;
	return Wi;
}

//计算每一个x*对应的*i
void calculate_Wi_Matrices(Mat img, vector<Point2d>& obj, vector<MatrixXd>& vec)
{
	points = obj;
	int Width = img.size().width, Height = img.size().height;
	ArrayXd heightArray = ArrayXd::LinSpaced(C1 + 1, 0, Height - 1),
		widthArray = ArrayXd::LinSpaced(C2 + 1, 0, Width - 1);//分割
	string filename = "Wi.txt";
	ofstream fout(filename);
	int count = 0;
	for (int i = 0; i < C1; i++) {
		double y = (heightArray(i) + heightArray(i + 1)) / 2;
		for (int j = 0; j < C2; j++) {
			//	cout << "i = " << i << ", j = " << j << endl;
			double x = (widthArray(j) + widthArray(j + 1)) / 2;
			MatrixXd Wi = calculate_Wi_forPoint(x, y);
			vec.push_back(Wi);
		}
	}

}

// 计算每个超像素对应的权重矩阵Wi
void calculate_Wi_Matrices(Mat img, vector<Point2d>& obj, vector<MatrixXd>& vec, vector<pair<int, int>>& superpixelsCenter) {
	points = obj;
	int count_Wi = superpixelsCenter.size();
	vec.resize(count_Wi);
#pragma omp parallel for
	for (int i = 0; i < count_Wi; i++) {
		vec[i] = calculate_Wi_forPoint(superpixelsCenter[i].first, superpixelsCenter[i].second);
		// cout << i << " ";
	}
	cout << endl;
	return;
}


//每个分块的H*
vector<Matrix3d> calculate_CellHomography(vector<MatrixXd>& matrices, MatrixXd& A)
{
	vector<Matrix3d> H_vec;
	H_vec.resize(matrices.size());
	string fileName = "H_vec.txt";
	ofstream fout(fileName);
#pragma omp parallel for
	for (int i = 0; i < matrices.size(); i++)
	{
		MatrixXd WA = matrices[i] * A;
		Matrix3d H;
		JacobiSVD<MatrixXd> svd(WA, ComputeThinU | ComputeThinV);
		MatrixXd V = svd.matrixV();
		VectorXd h = V.col(V.cols() - 1);
		
		H << h[0], h[1], h[2],
			h[3], h[4], h[5],
			h[6], h[7], h[8];
		/*
		fout << h[0] << " " << h[1] << " " << h[2] << endl;
		fout << h[3] << " " << h[4] << " " << h[5] << endl;
		fout << h[6] << " " << h[7] << " " << h[8] << endl;
		fout << endl;
		*/
		// H_vec.push_back(H);
		H_vec[i] = H;
		// cout << H << endl;
	}
	// fout.close();
	return H_vec;
}

// 将原始的网格根据每个分块的homo进行单映变换，形成新的网格，并建立网格索引
GridBox** getIndex(const Mat& img, int C1, int C2, int offset_x, int offset_y, vector<Matrix3d> H_vec)
{
	GridBox** a = new GridBox*[C2 + 1];
	for (int i = 0; i < C2 + 1; i++)
		a[i] = new GridBox[C1 + 1];

	ArrayXf widthArray = ArrayXf::LinSpaced(C2 + 1, 0, img.size().width),
		heightArray = ArrayXf::LinSpaced(C1 + 1, 0, img.size().height); // 0 ~ C1 - 1, 0 ~ C2 - 1
	double min_x, min_y;
	for (int gy = 0; gy < C1; gy++)
		for (int gx = 0; gx < C2; gx++)
		{
			int H_index = gy * C2 + gx;
			double topleftx, toplefty,
				toprightx, toprighty,
				bottomleftx, bottomlefty,
				bottomrightx, bottomrighty;
			ConvertCoordinates(widthArray[gx], heightArray[gy], topleftx, toplefty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx + 1], heightArray[gy], toprightx, toprighty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx], heightArray[gy + 1], bottomleftx, bottomlefty, H_vec[H_index]);
			ConvertCoordinates(widthArray[gx + 1], heightArray[gy + 1], bottomrightx, bottomrighty, H_vec[H_index]);
			GridBox gbox = GridBox(Point2d(topleftx + offset_x, toplefty + offset_y),
				Point2d(toprightx + offset_x, toprighty + offset_y),
				Point2d(bottomleftx + offset_x, bottomlefty + offset_y),
				Point2d(bottomrightx + offset_x, bottomrighty + offset_y));
			a[gy][gx] = gbox;
		}
	return a;
}


// 根据图像坐标（x,y），获取该位置在grids网格上的索引坐标（gx,gy）
void findGrid(int &gx, int &gy, double x, double y, GridBox** grids)
{
	for (int grid_x = 0; grid_x < C2; grid_x++)
		for (int grid_y = 0; grid_y < C1; grid_y++)
		{
			if (grids[grid_y][grid_x].contains(x, y))
			{
				gx = grid_x;
				gy = grid_y;
				return;
			}
		}
}


//横向(x)分成C2块 纵向(y)分成C1块
//使用每一个小块图像的homo对img进行变换
void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, int C1, int C2)
{
	int Width = img.size().width, Height = img.size().height;
	// int x_offset = 0, y_offset = 100;
	int x_offset = 0, y_offset = 100;
	GridBox** grids = getIndex(img, C1, C2, x_offset, y_offset, H_vec);
	target = Mat::zeros(height, width, img.type());
	uchar b, g, r;

	for (int y = y_offset; y < height; y++) {
		for (int x = x_offset; x < width; x++)
		{
			int gx = -1, gy = -1;
			findGrid(gx, gy, x, y, grids);
			if (gx >= 0 && gy >= 0) {
				int H_index = gx + gy * C2;
				double t_nx, t_ny;
				ConvertCoordinates(x - x_offset, y - y_offset, t_nx, t_ny, H_vec[H_index].inverse());

				if (t_nx >= 0 && t_nx <= Width && t_ny >= 0 && t_ny <= Height)
				{
					ConvertPoint(img, Width, Height, t_nx, t_ny, b, g, r);
					target.at<Vec3b>(y, x) = Vec3b(b, g, r);
				}
			}
		}
	}
	Mat gridMat = Mat::zeros(height, width, img.type());
	for (int gy = 0; gy<C1; gy++)
		for (int gx = 0; gx < C2; gx++)
		{
			GridBox grid = grids[gy][gx];
			double* verty = grid.verty, *vertx = grid.vertx;
			int i, j;
			Point2d p1, p2;

			for (i = 0, j = 3; i < 4; j = i++)
			{
				p1 = Point2d(vertx[i], verty[i]);
				p2 = Point2d(vertx[j], verty[j]);
				line(gridMat, p1, p2, cv::Scalar(255, 0, 0), 1, CV_AA);
			}
		}
	imshow("warp img 2", target);
	imshow("grids", gridMat);
	waitKey(0);
}


void ConvertImage(const Mat& img, Mat& target, vector<Matrix3d> H_vec, vector<vector<pair<int, int>>>& superPixels) {
	int Width = img.size().width, Height = img.size().height;
	target = Mat::zeros(height, width, img.type());
	Mat target1 = Mat::zeros(height, width, img.type());
	Mat target2 = Mat::zeros(height, width, img.type());
	vector<vector<bool>> imgMat;
	vector<bool> vec(width, 0);
	imgMat.resize(height);
// #pragma omp parallel for
	for (int i = 0; i < height; i++)
		imgMat[i] = vec;

	// 图像四个点对应的超像素索引
	int homo_index0, homo_index1, homo_index2, homo_index3;

	int count = 0; // The total number of pixels that have not been assigned
	int superPixelsCount = 0; // The total number of pixels assigned
	// int x_offset = 0, y_offset = 100;
	// int x_offset = 0, y_offset = 200;
#pragma omp parallel for
	for (int i = 0; i < superPixels.size(); i++) {
		for (int j = 0; j < superPixels[i].size(); j++) {
			// int y = superPixels[i][j].first;
			int x = superPixels[i][j].first;
			// int x = superPixels[i][j].second;
			int y = superPixels[i][j].second;
			if (x == 0 && y == 0)
				homo_index0 = i;
			if (x == (Width - 1) && y == 0)
				homo_index1 = i;
			if (x == 0 && y == (Height - 1))
				homo_index2 = i;
			if (x == (Width - 1) && y == (Height - 1))
				homo_index3 = i;
			double t_x, t_y;
			ConvertCoordinates(x, y, t_x, t_y, H_vec[i]);
			computeX_Y(t_x, t_y);
			t_x += x_offset;
			t_y += y_offset;
			if ((t_x >= 0 && t_x < width) && (t_y >= 0 && t_y < height)) {
				target.at<Vec3b>(int(t_y), int(t_x)) = img.at<Vec3b>(y, x);
				imgMat[int(t_y)][int(t_x)] = 1;
				superPixelsCount += 1;
			}
			else
				count += 1;
		}
	}

	cout << "count:" << count << endl;
	cout << "superPixelsCount" << superPixelsCount << endl;

	// 画出需要线性插值的区域
	double topleftx, toplefty,
		toprightx, toprighty,
		bottomleftx, bottomlefty,
		bottomrightx, bottomrighty; //图像中的四个点

	
	// 左上角的点（topleftx, toplefty）
	int x0 = 0, y0 = 0;
	ConvertCoordinates(x0, y0, topleftx, toplefty, H_vec[homo_index0]);
	// 右上角的点（toprightx, toprighty）
	int x1 = Width, y1 = 0;
	ConvertCoordinates(x1, y1, toprightx, toprighty, H_vec[homo_index1]);
	// 左下角的点（bottomleftx, bottomlefty）
	int x2 = 0, y2 = Height;
	ConvertCoordinates(x2, y2, bottomleftx, bottomlefty, H_vec[homo_index2]);
	// 右下角的点（bottomrightx, bottomrighty）
	int x3 = Width, y3 = Height;
	ConvertCoordinates(x3, y3, bottomrightx, bottomrighty, H_vec[homo_index3]);
	GridBox gbox = GridBox(Point2d(topleftx + x_offset, toplefty + y_offset),
		Point2d(toprightx + x_offset, toprighty + y_offset),
		Point2d(bottomleftx + x_offset, bottomlefty + y_offset),
		Point2d(bottomrightx + x_offset, bottomrighty + y_offset));
	// drawArea(target1, gbox);

	/*
	// 将需要线性插值的区域变成红色
	int InterPointCount = 0;
	int totalPointCount = 0;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (gbox.contains(i, j)) {
				totalPointCount += 1;
				if (!imgMat[j][i]) {
					target1.at<Vec3b>(j, i) = Vec3b(0, 0, 255);
					InterPointCount += 1;
				}
				else {
					// InterPointCount += 1;
				}
			}
			// target1.at<Vec3b>(j,i)= Vec3b(0, 0, 255);
		}
	}
	cout << "total points:" << totalPointCount << endl;
	cout << "need to inter points:" << InterPointCount << endl;
	*/

	//进行线性插值
	int InterCount = 1; //需要进行线性插值的次数
	for (int i = 0; i < InterCount; i++) {
		linearInterImg2(target, gbox, imgMat);
	}

	/*
	// 将需要线性插值的区域变成红色
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (gbox.contains(i, j) && (!imgMat[j][i]))
				target2.at<Vec3b>(j, i) = Vec3b(0, 0, 255);
		}
	}
	*/

	cout << count << endl;
	// imshow("homo_image", target);
	imwrite("homo_img2.jpg", target);
	imwrite("target1.jpg", target1);
	imwrite("target2.jpg", target2);
	waitKey(0);
	return;
}


bool isBlack(const Mat& img, int x, int y, uchar& b, uchar& g, uchar& r)
{
	if (x >= img.size().width || y >= img.size().height)
		return true;
	Vec3b color = img.at<Vec3b>(y, x);
	b = color[0];
	g = color[1];
	r = color[2];
	if (b == 0 && g == 0 && r == 0)
		return true;
	return false;
}


uchar getWarpValue(uchar val1, uchar val2, int weight1, int weight2)
{
	return (val1 * weight1 + val2 * weight2) / (weight1 + weight2);
}


void warpImage(const Mat& image_1, const Mat& img_2, Mat& target)
{
	uchar b, g, r;
	uchar b1, g1, r1, b2, g2, r2;
	target = Mat::zeros(height, width, CV_8UC3);
	Mat img_1 = Mat::zeros(width, height, CV_8UC3);
	image_1.copyTo(img_1(Rect(x_offset, y_offset, image_1.size().width, image_1.size().height)));
// #pragma omp parallel for
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
		{
			int weight_left = isBlack(img_1, x, y, b1, g1, r1) ? 0 : 1,
				weight_right = isBlack(img_2, x, y, b2, g2, r2) ? 0 : 1;
			if (weight_left + weight_right > 0)
			{
				b = getWarpValue(b1, b2, weight_left, weight_right);
				g = getWarpValue(g1, g2, weight_left, weight_right);
				r = getWarpValue(r1, r2, weight_left, weight_right);
				target.at<Vec3b>(y, x) = Vec3b(b, g, r);
			}
		}
	// imshow("APAP target", target);
	imwrite("result.jpg", target);
	// waitKey(0);
}

void drawArea(Mat& img, GridBox& grid) {
	double* verty = grid.verty, *vertx = grid.vertx;
	int i, j;
	Point2d p1, p2;

	for (i = 0, j = 3; i < 4; j = i++)
	{
		p1 = Point2d(vertx[i], verty[i]);
		p2 = Point2d(vertx[j], verty[j]);
		line(img, p1, p2, cv::Scalar(255, 0, 0), 1, CV_AA);
	}
}

void linearInterImg(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat) {
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (gird.contains(i, j) && !imgMat[j][i]) {
				vector<Point2d> pointVec;
				if (imgMat[j][i - 1])
					pointVec.push_back(Point2d(i - 1, j)); //left point
				if (imgMat[j][i + 1])
					pointVec.push_back(Point2d(i + 1, j)); //right point
				if (imgMat[j - 1][i])
					pointVec.push_back(Point2d(i, j - 1)); //top point
				if (imgMat[j + 1][i])
					pointVec.push_back(Point2d(i, j + 1)); //bottom point
				if (!pointVec.empty()) {
					interMat(i, j, img, pointVec);
					imgMat[j][i] = 1;
				}
			}
			else
				continue;
		}
	}
}

void linearInterImg1(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat) {
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (gird.contains(i, j) && !imgMat[j][i]) {
				if ((i - 1) >= 0 && (i - 1) < width && (j) >= 0 && (j) < height) {
					img.at<Vec3b>(j, i) = img.at<Vec3b>(j, i - 1);
					cout << double(img.at<Vec3b>(j, i - 1)[0]) << " " << double(img.at<Vec3b>(j, i - 1)[1]) << " " << double(img.at<Vec3b>(j, i - 1)[2]) << endl;
					imgMat[j][i] = 1;
				}
			}
		}
	}
}

void linearInterImg2(Mat& img, GridBox& gird, vector<vector<bool>>& imgMat) {
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (gird.contains(i, j) && !imgMat[j][i]) {
				linearWeight(img, j, i, imgMat);
			}
		}
	}
}

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat) {
	double bSum = 0, gSum = 0, rSum = 0;
	int countNum = 0;
	// 上(j-1,i)
	if ((i) >= 0 && (i) < width && (j - 1) >= 0 && (j - 1) < height && imgMat[j - 1][i]) {
		countNum += 1;
		// cout << double(img.at<Vec3b>(j - 1, i)[0]) << " " << double(img.at<Vec3b>(j - 1, i)[1]) << " " << double(img.at<Vec3b>(j - 1, i)[2]) << endl;
		bSum += double(img.at<Vec3b>(j - 1, i)[0]);
		gSum += double(img.at<Vec3b>(j - 1, i)[1]);
		rSum += double(img.at<Vec3b>(j - 1, i)[2]);
	}
	//下(j+1,i)
	if ((i) >= 0 && (i) < width && (j + 1) >= 0 && (j + 1) < height && imgMat[j + 1][i]) {
		countNum += 1;
		// cout << double(img.at<Vec3b>(j + 1, i)[0]) << " " << double(img.at<Vec3b>(j + 1, i)[1]) << " " << double(img.at<Vec3b>(j + 1, i)[2]) << endl;
		bSum += double(img.at<Vec3b>(j + 1, i)[0]);
		gSum += double(img.at<Vec3b>(j + 1, i)[1]);
		rSum += double(img.at<Vec3b>(j + 1, i)[2]);
	}

	//左(j,i-1)
	if ((i-1) >= 0 && (i-1) < width && (j) >= 0 && (j) < height && imgMat[j][i - 1]) {
		countNum += 1;
		// cout << double(img.at<Vec3b>(j, i - 1)[0]) << " " << double(img.at<Vec3b>(j, i - 1)[1]) << " " << double(img.at<Vec3b>(j, i - 1)[2]) << endl;
		bSum += double(img.at<Vec3b>(j, i - 1)[0]);
		gSum += double(img.at<Vec3b>(j, i - 1)[1]);
		rSum += double(img.at<Vec3b>(j, i - 1)[2]);
	}

	//右(j.i+1)
	if ((i + 1) >= 0 && (i + 1) < width && (j) >= 0 && (j) < height && imgMat[j][i+1]) {
		countNum += 1;
		// cout << double(img.at<Vec3b>(j, i + 1)[0]) << " " << double(img.at<Vec3b>(j, i + 1)[1]) << " " << double(img.at<Vec3b>(j, i + 1)[2]) << endl;
		bSum += double(img.at<Vec3b>(j, i+1)[0]);
		gSum += double(img.at<Vec3b>(j, i+1)[1]);
		rSum += double(img.at<Vec3b>(j, i+1)[2]);
	}

	// cout << "countNum:" << countNum << "/bSum;" << bSum << "/gSum:" << gSum << "/rSum:" << rSum << endl;

	if (countNum != 0) {
		img.at<Vec3b>(j, i)[0] = bSum / countNum;
		img.at<Vec3b>(j, i)[1] = gSum / countNum;
		img.at<Vec3b>(j, i)[2] = rSum / countNum;
		imgMat[j][i] = 1;
	}
}

void linearWeight(Mat& img, int j, int i, vector<vector<bool>>& imgMat, int ThresholdN) {
	//double->RGBValue, double->weigh
	vector<double> linearValues;
	double bTempValue, gTempValue, rTempValue, eTempValue;
											   
	// 上(j-1,i)
	for (int k = 1; k <= ThresholdN;k++) {
		if ((i) >= 0 && (i) < width && (j - 1) >= 0 && (j - 1) < height && imgMat[j - 1][i]) {
			bTempValue = double(img.at<Vec3b>(j - 1, i)[0]);
			gTempValue = double(img.at<Vec3b>(j - 1, i)[1]);
			rTempValue = double(img.at<Vec3b>(j - 1, i)[2]);
			eTempValue = exp(-k);
			linearValues.push_back(bTempValue);
			linearValues.push_back(gTempValue);
			linearValues.push_back(rTempValue);
			linearValues.push_back(eTempValue);
			break;
		}
		j = j - k;
	}

	//下(j+1,i)
	for (int k = 1;k <= ThresholdN;k++) {
		if ((i) >= 0 && (i) < width && (j + 1) >= 0 && (j + 1) < height && imgMat[j + 1][i]) {
			bTempValue = double(img.at<Vec3b>(j + 1, i)[0]);
			gTempValue = double(img.at<Vec3b>(j + 1, i)[1]);
			rTempValue = double(img.at<Vec3b>(j + 1, i)[2]);
			eTempValue = exp(-k);
			linearValues.push_back(bTempValue);
			linearValues.push_back(gTempValue);
			linearValues.push_back(rTempValue);
			linearValues.push_back(eTempValue);
			break;
		}
		j = j + k;
	}

	//左(j,i-1)
	for (int k = 1;k <= ThresholdN;k++) {
		if ((i - 1) >= 0 && (i - 1) < width && (j) >= 0 && (j) < height && imgMat[j][i - 1]) {
			bTempValue = double(img.at<Vec3b>(j, i - 1)[0]);
			gTempValue = double(img.at<Vec3b>(j, i - 1)[1]);
			rTempValue = double(img.at<Vec3b>(j, i - 1)[2]);
			eTempValue = exp(-k);
			linearValues.push_back(bTempValue);
			linearValues.push_back(gTempValue);
			linearValues.push_back(rTempValue);
			linearValues.push_back(eTempValue);
			break;
		}
		i = i - k;
	}

	//右(j.i+1)
	for (int k = 1;k <= ThresholdN;k++) {
		if ((i + 1) >= 0 && (i + 1) < width && (j) >= 0 && (j) < height && imgMat[j][i + 1]) {
			bTempValue = double(img.at<Vec3b>(j, i + 1)[0]);
			gTempValue = double(img.at<Vec3b>(j, i + 1)[1]);
			rTempValue = double(img.at<Vec3b>(j, i + 1)[2]);
			eTempValue = exp(-k);
			linearValues.push_back(bTempValue);
			linearValues.push_back(gTempValue);
			linearValues.push_back(rTempValue);
			linearValues.push_back(eTempValue);
			break;
		}
		i = i + k;
	}
	
	int linearCount = linearValues.size() / 4;
	double eSum = 0;
	for (int k = 0;k < linearCount;k++) {
		eSum += linearValues[k * 4 + 3];
	}

	//Exponential interpolation
	if (linearCount != 0) {
		double bSum = 0, gSum = 0, rSum = 0;
		for (int k = 0;k < linearCount;k++) {
			bSum += (linearValues[k * 4 + 3] / eSum)*linearValues[k * 4 + 0];
			gSum += (linearValues[k * 4 + 3] / eSum)*linearValues[k * 4 + 1];
			rSum += (linearValues[k * 4 + 3] / eSum)*linearValues[k * 4 + 2];
		}
		img.at<Vec3b>(j, i)[0] = bSum;
		img.at<Vec3b>(j, i)[1] = gSum;
		img.at<Vec3b>(j, i)[2] = rSum;
		imgMat[j][i] = 1;
	}

}

void interMat(int i, int j, Mat& img, vector<Point2d>& pointVec) {
	uchar bSum, gSum, rSum;
	double countPoint = 1.0 / double(pointVec.size());
	for (int i = 0; i < pointVec.size(); i++) {
		bSum += img.at<Vec3b>(pointVec[i].y, pointVec[i].x)[0];
		gSum += img.at<Vec3b>(pointVec[i].y, pointVec[i].x)[1];
		rSum += img.at<Vec3b>(pointVec[i].y, pointVec[i].x)[2];
	}
	uchar b, g, r;
	b = bSum * countPoint;
	g = gSum * countPoint;
	r = rSum * countPoint;
	// cout << "b:" << int(b) << "g:" << int(g) << "r:" << int(r) << endl;
	img.at<Vec3b>(j, i) = Vec3b(b, g, r);
}

void computeX_Y(double& t_x, double& t_y) {
	if (t_x - int(t_x) > 0.5)
		t_x += 1;
	if (t_y - int(t_y) > 0.5)
		t_y += 1;
}

void drawPointAndRectangular(Mat& img, vector<Point2d>& scene2, vector<pair<int, int>>& superpixelsCenter) {
	int Scene2Number = scene2.size(), superpixelsCenterNumber = superpixelsCenter.size();

	//Draw the position of feature points in the image
	for (int i = 0;i < Scene2Number;i++) {
		// circle(img, scene2[i], 2, Scalar(0, 0, 255), 3);
		Point2d topPoint(scene2[i].x, scene2[i].y -2), bottomPoint(scene2[i].x, scene2[i].y + 2);
		Point2d leftPoint(scene2[i].x - 2, scene2[i].y), rightPoint(scene2[i].x + 2, scene2[i].y);
		line(img, topPoint, bottomPoint, Scalar(0, 0, 255), 1);
		line(img, leftPoint, rightPoint, Scalar(0, 0, 255), 1);
	}

	//Draw the position of the superpixel center in the image
	for (int i = 0;i < superpixelsCenterNumber;i++) {
		Point point1 = Point(superpixelsCenter[i].first - 2, superpixelsCenter[i].second - 2),
			point2 = Point(superpixelsCenter[i].first + 2, superpixelsCenter[i].second + 2);
		rectangle(img, point1, point2, Scalar(124, 71, 102), 1, 4);
	}

	imwrite("imgMarked.jpg", img);
}