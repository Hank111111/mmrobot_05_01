#ifndef __DIGITAL_DEAL_H__
#define __DIGITAL_DEAL_H__

#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;



class digitalProcess{
public:
	string finddigits(Mat img);
private:
	vector<Mat> get_mats(Mat mt);
	string changename(int i, int j);
};

#endif
