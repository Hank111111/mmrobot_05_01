#include "mm_visual_postion/digitalmeter/digitalDeal.h"
#include "ros/ros.h"
#include <ros/package.h>

using namespace std;
using namespace cv;
using namespace cv::ml;

string raw_path = ros::package::getPath("mm_visual_postion") + "/src/digitalmeter";

bool cmp(const Rect &a, const Rect &b) {
	if (a.y - b.y > -30.0 && a.y - b.y < 30.0)
	{
		if (a.x < b.x)
			return true;
		else
			return false;
	}
	else if (a.y - b.y <= -30.0)
		return true;
	else 
		return false;
}


string digitalProcess::finddigits(Mat img){
    Mat traindata, train_label, tmp;
    string raw_path = ros::package::getPath("mm_visual_postion") + "/src/digitalmeter";
	string knnPath = raw_path + "/knn.xml";
	//for (int j = 0; j < 10; j++) {
	//	for (int i = 0; i < 40; i++) {
	//		string name = changename(j, i);
	//		//cout << name << endl;
	//		tmp = imread(name, 0);
	//		//imshow("src", tmp);
	//		//waitKey();
	//		resize(tmp, tmp, Size(75, 125));
	//		traindata.push_back(tmp.reshape(0, 1));
	//		train_label.push_back(j);
	//	}
	//}

	//traindata.convertTo(traindata, CV_32F);
	//Ptr<TrainData> tData = TrainData::create(traindata, ROW_SAMPLE, train_label);
	//Ptr<KNearest> knn = KNearest::create();
	//knn->setDefaultK(3);
	//knn->setIsClassifier(true);  
	//knn->train(tData);
	//knn->save(knnPath);

	Ptr<KNearest> knn = StatModel::load<KNearest>(knnPath);
	Mat mymt = img;
	float change_ratio = 400.0 / mymt.rows;
	resize(mymt, mymt, Size(int(mymt.cols * change_ratio), 400), INTER_CUBIC);

	vector<Mat> dds = get_mats(mymt);    
	ostringstream oss;
	for (int n = 0; n < dds.size(); n++) {
		dds[n].copyTo(tmp);
		resize(tmp, tmp, Size(75, 125));
		tmp = tmp.reshape(0, 1);
		tmp.convertTo(tmp, CV_32F);
		int result = knn->predict(tmp);
		oss << result;
		if (n > 0 && n % 4 == 3)
			oss << "/";
		if (n % 4 == 2)
			oss << ".";
	}
	cout << oss.str().size() << ": " << oss.str() << endl;
    return oss.str();
}


string digitalProcess::changename(int i, int j) {
	ostringstream oss;
	oss << (raw_path + "/")<< i << "_out/" << j << ".jpg";
	string name = oss.str();
	return name;
}


vector<Mat> digitalProcess::get_mats(Mat mt) {
	vector<Mat> channels, result_datas;
	Mat gray, image_bin, result, ele, image_bin2, temp;
	cvtColor(mt, gray, CV_BGR2GRAY);
	threshold(gray, image_bin2, 0, 255, CV_THRESH_OTSU);
	ele = getStructuringElement(MORPH_RECT, Size(5, 5)); // 膨胀
	dilate(image_bin2, image_bin, ele);
	image_bin.copyTo(temp);

	//imshow("preprocess", temp);
	//waitKey();
	
	vector<vector<Point>> conts;
	vector<Vec4i> hire;

	//轮廓提取
	findContours(image_bin, conts, hire, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//Mat imgcontours = Mat::zeros(image_bin.size(), CV_8UC1);
	//drawContours(imgcontours, conts, -1, Scalar(255), 1, 8, hire);
	//imshow("contours", imgcontours);
	//waitKey();

	vector<Rect> rects;
	temp.copyTo(image_bin);
	for (int i = 0; i < conts.size(); i++) {
		Rect t = boundingRect(Mat(conts[i]));
		float t_ratio = t.width / t.height;
		//Mat ROI = mt(t);
		//imshow("roi", ROI);
		//waitKey();
		if (t_ratio > 0.8 || t.width < 15 || t.height < 60) continue;
		rects.push_back(t);
	}


	//sort numbers
	sort(rects.begin(), rects.end(), cmp);

	//cut pictures
	for (int i = 0; i < rects.size(); i++) {
		image_bin2(rects[i]).copyTo(result);
		//imshow("result", result);
		//waitKey();
		result_datas.push_back(result);
	}

	return result_datas;
}
        
