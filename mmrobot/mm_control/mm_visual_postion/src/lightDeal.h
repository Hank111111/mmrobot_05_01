#include <opencv2/core/core.hpp> 
#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>  
#include <string>



using namespace std;
using namespace cv;

class lightProcess
{
private:


public:
	lightProcess();
	~lightProcess();
	float* findlights(cv::Mat img);

};

lightProcess::lightProcess()
{
}

lightProcess::~lightProcess()
{
}

float* lightProcess::findlights(cv::Mat img)
{
	float *state = new float[6];
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y, dst;
	//ԭʼͼ  
	Mat src = img;
	const Size size = src.size();
	int height = size.height;
	int width = size.width;

	//�Ҷ�ͼ���˲�
	Mat myimg1;
	cvtColor(src, myimg1, COLOR_BGR2GRAY);
	GaussianBlur(myimg1, myimg1, Size(5, 5), 0, 0);

	//HSV�ռ䣬��ͨ��
	Mat hsv = Mat::zeros(height, width, CV_16SC3);
	cvtColor(src, hsv, CV_RGB2HSV);

	//����ͨ��
	vector<cv::Mat> mv;
	Mat hsv_h = Mat::zeros(height, width, CV_8UC1);
	Mat hsv_s = Mat::zeros(height, width, CV_8UC1);
	Mat hsv_v = Mat::zeros(height, width, CV_8UC1);

	split(hsv, mv);
	hsv_h = mv.at(0);
	hsv_s = mv.at(1);
	hsv_v = mv.at(2);


	//�Ҷ�ͼsobel�ݶ�
	Sobel(myimg1, grad_x, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);

	Sobel(myimg1, grad_y, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);

	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);


	//namedWindow("sobel", 0);
	//imshow("sobel", dst);
	//waitKey();

	//���Բ�任,����Ҫ�󣺾������40cm��λ�ڵ������룬��������
	vector<Vec3f> circles;
	HoughCircles(dst, circles, HOUGH_GRADIENT, 1, 30, 60, 65, 30, 70);

	//λ������,�ж�Բ�飬�Ե�һ����ɫԲΪ��׼
	vector<Vec3f> lights;
	Vec3f temp;
	int counts = 0;



	for (unsigned int i = 0; i < circles.size(); i++)
		for (unsigned int j = 0; j < circles.size() - i - 1; j++)
		{
			if (circles[j][0] > circles[j + 1][0])
			{
				temp = circles[j];
				circles[j] = circles[j + 1];
				circles[j + 1] = temp;
			}
		}

	for (unsigned int i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = circles[i][2];
		//ͳ����ɫ
		int colWidth = 1.414*radius;
		int colHeight = 1.414*radius;
		int colMinX = center.x - 0.5*colWidth;
		int colMinY = center.y - 0.5*colHeight;
		Mat matColorH = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Mat matColorS = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Mat matColorV = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Rect rectROI(colMinX, colMinY, colWidth, colHeight);
		hsv_h(rectROI).convertTo(matColorH, matColorH.type(), 1, 0);
		hsv_s(rectROI).convertTo(matColorS, matColorS.type(), 1, 0);
		hsv_v(rectROI).convertTo(matColorV, matColorV.type(), 1, 0);
		Scalar mean_h = mean(matColorH);
		//Scalar mean_s = mean(matColorS);
		//Scalar mean_v = mean(matColorV);
		double dH = mean_h[0];
		//double dS = mean_s[0];
		//double dV = mean_v[0];
		if (dH > 20 && dH <= 79)
		{
			//cout << "��λ�̵�" << endl;
			//cout << "Բ��x: " << center.x << "  Բ��y: " << center.y << "  �뾶r: " << radius << endl;
			lights.push_back(circles[i]);
			break;
		}
	}
	
	if (lights.empty())
	{
		for (int i = 0;i<6;i++)
			state[i]=-1;
		return state;
	}
	for (unsigned int i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		if (center.x <= lights[0][0])
			continue;
		if (abs(center.x - lights[0][0]) <= 10 && abs(center.y - lights[0][1]) <= 10)
			continue;
		if (abs(center.y - lights[counts][1]) < 20)
		{
			lights.push_back(circles[i]);
			counts++;
		}
		if (counts >= 4)
			break;
	}

	//�����ʶ
	if (counts != 4)
	{
		state[0] = 0;
		cout << "ʶ��ָʾ�ƴ���" << endl;
	}
	else
		state[0] = 1;


	//�ж���ɫ�����𣬻��Ƴ�Բ
	for (unsigned int i = 0; i < lights.size(); i++)
	{
		Point center(cvRound(lights[i][0]), cvRound(lights[i][1]));
		int radius = lights[i][2];
		int colWidth = 1.414*radius;
		int colHeight = 1.414*radius;
		int colMinX = center.x - 0.5*colWidth;
		int colMinY = center.y - 0.5*colHeight;
		Mat matColorH = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Mat matColorS = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Mat matColorV = Mat::zeros(colWidth, colHeight, CV_8UC1);
		Rect rectROI(colMinX, colMinY, colWidth, colHeight);
		hsv_h(rectROI).convertTo(matColorH, matColorH.type(), 1, 0);
		hsv_s(rectROI).convertTo(matColorS, matColorS.type(), 1, 0);
		hsv_v(rectROI).convertTo(matColorV, matColorV.type(), 1, 0);
		Scalar mean_h = mean(matColorH);
		Scalar mean_s = mean(matColorS);
		Scalar mean_v = mean(matColorV);
		double dH = mean_h[0];
		double dS = mean_s[0];
		double dV = mean_v[0];
		//cout << "Num: " << i+1 << "----X:" << center.x << " Y:" << center.y << endl;
		//cout << "H:" << dH << " S:" << dS << " V:" << dV << endl;
		string lightcolor, lightstate, lightresult;
		if (dH > 0 && dH < 180)
		{
			if (dH > 20 && dH <= 79)
				lightcolor = " Green ";
			if (dH > 80 && dH <= 115)
			{
				if (dV > 180 && dV < 255 && dS < 15)
					lightcolor = " Red ";
				else
					lightcolor = " Orange ";
			}

			if (dH > 115 && dH <= 150) 
				lightcolor = " Red ";

			if (dV >= 0 && dV < 180)
			{
				lightstate = "Off ";
				state[i+1] = 0;
			}

			if (dV >= 180 && dV <= 255)
			{
				lightstate = "On ";
				state[i+1] = 1;
			}
		}
		if (dH >= 80 && dH <= 150 && i == lights.size() - 1)
			lightcolor = " Orange ";
		lightresult = lightcolor + lightstate;

		//����Բ��
		circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		//����Բ����
		circle(src, center, radius, Scalar(155, 50, 255), 3, 8, 0);
		char str_i[20];
		str_i[0] = i + 1 + 48;
		for (unsigned int i = 0; i < lightresult.length(); i++)
			str_i[i + 1] = lightresult[i];
		str_i[lightresult.length() + 1] = '\0';
		Point Textposition(center.x - radius, center.y - radius);
		putText(src, str_i, Textposition, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 2, 8);
	}


	//namedWindow("result", 0);
	//imshow("result", src);
	//waitKey();

	string saveName;
	saveName= "/home/youibot/mrobot/src/Leftresult.bmp";
	imwrite(saveName, src);//�����ļ�
	return state;
}

