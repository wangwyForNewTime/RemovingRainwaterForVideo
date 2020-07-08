#pragma once
/**
文件描述:超分辨率算法头文件-插值法
**/


#ifndef INTERPOLATOR_H_
#define INTERPOLATOR_H_
#include <opencv2/opencv.hpp>
#include <vector>

namespace cv {
	class Mat;
}

struct Point5d {
	double x;
	double y;
	double xw;
	double yw;
	double color;
};

class Interpolator {
public:
	Interpolator(const cv::Mat &src, double scale);
	cv::Mat BicubicInterpolate();
private:
	cv::Mat src;
	double scale;
	std::vector<Point5d> neighbors;

	double GetColor(const cv::Point_<double>& p);
};


#endif /* INTERPOLATOR_H_ */