#include "stdafx.h"

double gamma;
double sigma;
int C1, C2;
double SIFT_Threshold;
int Sampling_Rate;
int Sp_Size;
double RANSAC_Threshold;
int width, height;
int x_offset, y_offset;

void readFile(char* fileName) {
	ifstream f;
	f.open(fileName);
	string line;
	vector<string> lines;
	while (getline(f, line)) {
		if (line == "")
			break;
		else {
			lines.push_back(line);
		}
	}
	for (int i = 0; i < lines.size(); i++) {
		if (i == 0)
			gamma = stod(lines[i]);
		else if (i == 1)
			sigma = stod(lines[i]);
		else if (i == 2)
			C1 = stoi(lines[i]);
		else if (i == 3)
			C2 = stoi(lines[i]);
		else if (i == 4)
			SIFT_Threshold = stod(lines[i]);
		else if (i == 5)
			Sampling_Rate = stoi(lines[i]);
		else if (i == 6)
			Sp_Size = stoi(lines[i]);
		else if (i == 7)
			RANSAC_Threshold = stod(lines[i]);
		else if (i == 8)
			width = stoi(lines[i]);
		else if (i == 9)
			height = stoi(lines[i]);
		else if (i == 10)
			x_offset = stoi(lines[i]);
		else if (i == 11)
			y_offset = stoi(lines[i]);
	}
}