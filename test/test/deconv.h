#pragma once
#include <iostream>
#include "opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

namespace deconv
{
	// padded real input, complex frequency output

	const double PI = 3.14159265359;

	void Dft2D(const Mat& in, Mat* out)
	{
		int h = in.rows;
		int w = in.cols;
		float weight = 0.f;
		//float coef = 1.f / static_cast<float>(h * w);
		Mat out_[] = { Mat::zeros(h, w, CV_32FC1), Mat::zeros(h, w, CV_32FC1) };
		for (int k = 0; k < h; k++) {
			for (int l = 0; l < w; l++) {
				for (int m = 0; m < h; m++) {
					for (int n = 0; n < w; n++) {
						weight = 2.f * PI * ((float)k / (float)h * (float)m + (float)l / (float)w * (float)n);
						out_[0].at<float>(k, l) += in.at<float>(m, n) * cos(weight);
						out_[1].at<float>(k, l) += in.at<float>(m, n) * (-sin(weight));
					}
				}
				//out_[0].at<float>(k, l) *= coef;
				//out_[1].at<float>(k, l) *= coef;
			}
		}
		merge(out_, 2, (*out));
	}

	// complex frequency input, real output
	void InvDft2D(const Mat& in, Mat* out)
	{
		int h = in.rows;
		int w = in.cols;
		float weight = 0.f;
		float coef = 1.f / static_cast<float>(h * w);
		Mat in_[] = { Mat::zeros(h, w, CV_32FC1), Mat::zeros(h, w, CV_32FC1) };
		split(in, in_);
		Mat out_[] = { Mat::zeros(h, w, CV_32FC1), Mat::zeros(h, w, CV_32FC1) };
		for (int m = 0; m < h; m++) {
			for (int n = 0; n < w; n++) {
				for (int k = 0; k < h; k++) {
					for (int l = 0; l < w; l++) {
						weight = 2.f * PI * ((float)k / (float)h * (float)m + (float)l / (float)w * (float)n);
						out_[0].at<float>(m, n) += (in_[0].at<float>(k, l) * cos(weight) - in_[1].at<float>(k, l) * sin(weight));
						out_[1].at<float>(m, n) += (in_[0].at<float>(k, l) * sin(weight) + in_[1].at<float>(k, l) * cos(weight));
					}
				}
				out_[0].at<float>(m, n) *= coef;
				out_[1].at<float>(m, n) *= coef;
			}
		}
		out_[0].copyTo((*out));
	}

	void RearrangeAfterFilter(const Mat& in, Mat* out, int kernelsize)
	{
		int shiftval = kernelsize / 2;
		(*out) = Mat::zeros(in.rows, in.cols, CV_32FC1);
		in(Range(0, in.rows - shiftval), Range(in.cols - shiftval, in.cols)).copyTo((*out)(Range(shiftval, in.rows), Range(0, shiftval)));
		in(Range(in.rows - shiftval, in.rows), Range(0, in.cols - shiftval)).copyTo((*out)(Range(0, shiftval), Range(shiftval, in.cols)));
		in(Range(in.rows - shiftval, in.rows), Range(in.cols - shiftval, in.cols)).copyTo((*out)(Range(0, shiftval), Range(0, shiftval)));
		in(Range(0, in.rows - shiftval), Range(0, in.cols - shiftval)).copyTo((*out)(Range(shiftval, in.rows), Range(shiftval, in.cols)));
	}

}