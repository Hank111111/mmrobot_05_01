#pragma once
#ifndef __SWITCH_FINDER_BY_HOUGH_H__
#define __SWITCH_FINDER_BY_HOUGH_H__


#define _USE_MATH_DEFINES
#include <cmath>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include "mm_visual_postion/utils/hough_v3_4_5.h"
#include "mm_visual_postion/switch/LineRefiner.h"
#include "mm_visual_postion/utils/image_utils.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using namespace std;


class SwitchFinderByHough {
private:
	bool hough_circle_init;
	bool adapdative_treshold_init;
public:
	SwitchFinderByHough() {
		hough_circle_init = true;
		adapdative_treshold_init = true;
	}
	int findSwitch(const cv::Mat& img, const std::vector<cv::Mat>& template_img_ve, vector<cv::Point2d> & switch_corners, double& match_score, bool visualization);
	int findPossibleSwitchInROI(const cv::Mat& img,const cv::Mat& gray, const cv::Mat& edge, const cv::Rect& roi_rect, 
								const std::vector<cv::Mat>& template_img_vec, vector<cv::Point2d>& global_corners, double& template_match_score, bool visualization);

	
private:
	void getEdge(cv::Mat& gray, cv::Mat& edge);
	void getEdgeAdapdativeThreshold(cv::Mat& gray, cv::Mat& edge, bool visualization);
	void computeDistance(const vector<cv::Vec3f>& lines, const vector<int>& chosen_index,vector<float>& distance, float center_x, float center_y);
	void hardConstraintByDistance(const vector<float>& distance, vector<bool>& mask, float min_dist, float max_dist);
	void hardConstraintByAccumValue(const vector<cv::Vec3f>& lines, const vector<int>& indexes, vector<bool>& mask, float prop_k);
	void getParallelScore(const vector<cv::Vec3f>& lines1, const vector<bool>& lines1_mask,
		const vector<cv::Vec3f>& lines2, const vector<bool>& lines2_mask, float weight, cv::Mat& score);
	void addCenterDistanceAndAccumValue(const vector<cv::Vec3f>& lines1, const vector<int>& lines1_chosen_indexes, const vector<float>& center_dist_1,
		const vector<cv::Vec3f>& lines2, const vector<int>& lines2_chosen_indexes, const vector<float>& center_dist_2, float weight_dist, float weight_accum, cv::Mat& score);

	void getRotateRect(cv::Mat& edge, vector<cv::RotatedRect>& rect_list, bool visualisation);
	void getROI(vector<cv::RotatedRect>& rect_list, int img_width, int img_height, vector<cv::Rect>& roi_list, bool visualization, const cv::Mat& img=cv::Mat());
	void filterROI(vector<cv::Rect>& roi_list, vector<cv::Rect>& rect_list_filtered, bool visualization, const cv::Mat& img);
	void getCircles(const cv::Mat& gray, vector<cv::Vec4f>& circles, bool visualization);
	double inline getRectArea(cv::Rect rect);
	int matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat& template_img, double& match_val, cv::Point& match_loc);
	void addAccumValue(const vector<cv::Vec3f>& lines1, const vector<cv::Vec3f>& lines2, float weight_accum, cv::Mat& score);

};




#endif // !__SWITCH_FINDER_BY_HOUGH_H__