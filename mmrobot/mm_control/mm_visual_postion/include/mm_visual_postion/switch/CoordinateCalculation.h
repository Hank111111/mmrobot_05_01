#ifndef COORDINATECALCULATION_H
#define COORDINATECALCULATION_H
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string.h>
#include <math.h>
#include <cmath>

#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <opencv2/core/eigen.hpp>

#include "mm_visual_postion/switch/SwitchFinderByHough.h"
#include "mm_visual_postion/utils/RigidTransfomSolver.h"
#include "mm_visual_postion/switch/particularFilter.h"
#include "mm_visual_postion/utils/CoordinateTransformer.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
using namespace std;
using namespace cv;

#define VISUALIZATION
//#define BYPASS_TF
//#define DISABLE_PARTICLE_FILTER


#define METHOD_HOUGH 0
#define METHOD_MORPHOLOGY 1
#define METHOD_COMBINE_HOUGH_MORPH 2


void drawPolyLines(cv::Mat draw_img, vector<Point2d> points, Scalar rgb);
bool isMoved(const geometry_msgs::TransformStamped &last_transform, const geometry_msgs::TransformStamped &transform, double translation_threshold, double rotation_threshold);
class ResultInfo{
public:
	bool success;
	vector<Point2d> left_corners_in_pixel;
	vector<Point2d> right_corners_in_pixel;
	vector<Eigen::Vector4d> corners_in_base;
	vector<double> xyz_abc_in_cam;
	vector<double> gt_xyz_abc_in_cam;
	vector<double> gt_xyz_abc_in_base;

	vector<double> xyz_abc_in_base;
	
	vector<double> xyz_abc_in_tool;

	cv::Mat show_left;
	cv::Mat show_right;

	ResultInfo(){
		success = false;
		left_corners_in_pixel.resize(4);
		right_corners_in_pixel.resize(4);
		corners_in_base.resize(4);
		xyz_abc_in_cam.resize(6);
		xyz_abc_in_tool.resize(6);
		xyz_abc_in_base.resize(6);
	}

	void releaseShowImage(){
		show_left.release();
		show_right.release();
	}
};
class CoordinateCalculation
{
  private:
	SwitchFinderByHough switchFinderByHough;
	ParticleFilter particle_filter;
	geometry_msgs::TransformStamped last_transform_endeffector_to_base, last_update_transform_endeffector_to_base;
	Particle result_particle;
	
	cv::Mat left_projection_cv_mat, right_projection_cv_mat;

  public:
	cv::Mat showLeftImg;
	cv::Mat showRightImg;
	std::shared_ptr<CoordinateTransformer> left_transformer_ptr, right_transformer_ptr;
	std::shared_ptr<StereoCameraArmModel> model_ptr;
	CoordinateCalculation();
	~CoordinateCalculation();

	void updateCameraArmParams(std::shared_ptr<StereoCameraArmModel> &ptr);

	//像素坐标转换为世界坐标
	void uv2cxyzOpencv(const std::vector<Point2d> &uvLeftPoints, const std::vector<Point2d> &uvRightPoints, std::vector<Point3d> &xyzPoints);

	//畸变矫正
	void leftCorrect(const cv::Mat &leftImg, cv::Mat &corrected_img);
	void rightCorrect(const cv::Mat &rightImg, cv::Mat &corrected_img);

	//相机坐标系转换到工具坐标系，机器人手眼关系写在这里
	Point3d cxyz2bxyz(const Point3d &cxyz);
	void resetParticleFilter();
	//开关处理
	int processSwitchImgMorphology(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
								   const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, cv::Mat* draw_img_ptr=nullptr);
	int processSwitchImgMorphologyImpl(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
								   const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, bool visualization);

	int processSwitchImgHough(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
							  const cv::Vec4f &tempMargin, vector<Point2d> &M, double& match_score, cv::Mat* draw_img_ptr=nullptr);
	
	int calculateSwitchPosSimple(const cv::Mat &left_img, const cv::Mat &right_img, const std::vector<cv::Mat> &tempImgVec,
													  const cv::Vec4f &tempMargin, const geometry_msgs::TransformStamped &transform_endeffector_to_base,
													  vector<double> &xyzabc, int use_method, double& match_score, ResultInfo* additional_info_ptr=nullptr);
	
	int calculateSwitchPosParticleFilter(const cv::Mat &left_img, const cv::Mat &right_img,const std::vector<cv::Mat> &tempImgVec,
									const cv::Vec4f &tempMargin, const geometry_msgs::TransformStamped &transform_endeffector_to_base,
									vector<double> &xyzabc, int use_method=METHOD_COMBINE_HOUGH_MORPH, ResultInfo* additional_info_ptr=nullptr);

	int processSwitchImgCombineAvg(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
								   const cv::Vec4f &tempMargin, vector<Point2d> &M,vector<Point2d>& estimate_var, cv::Mat* draw_img_ptr=nullptr);

	//开关2的开关档位识别
	int recognizeSwitchState(const cv::Mat &img, const cv::Mat &tempImg, const cv::Vec4f &tempMargin); // img should be corrected firstly

	//手车开关处理

	//运动结果沿工具坐标系平移
	void Tmove(const double &x_move, const double &y_move, const double &z_move, const double bxyzabc[6], double xyzabc[6]);

	//检验开关是否识别正确
	bool isSwitchRecognitionCorrect(double xyzabc1[6], double xyzabc2[6]);

	//匹配前筛
	int matchSelect(const cv::Mat &img, const cv::Mat &tem, const cv::Vec4f &temMargin, Point2d &ret, cv::Rect2f &rect_pos, bool visualization);

	bool comparePixelPositions(const vector<Point2d> &method1_points, const vector<Point2d> &method2_points, double acceptable_error);
	void avgPixelPositions(const vector<Point2d> &method1_points, const vector<Point2d> &method2_points, vector<Point2d> &avg_points);
	int matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat &template_img, const double &size_factor, double &match_val, cv::Point &match_loc);
	int proposeROIs(const cv::Mat &edgeImg, vector<Rect> &roiRects);
	void getRotateRectVertices(const cv::RotatedRect &rect, vector<Point2d> &vertices);
	int getRotateRectVertices(const cv::RotatedRect &rect, const cv::Size imgSize, vector<Point2f> &vertices);
	int computeSwitchPoseInCameraCoord(const vector<Point2d> &leftPoints, const vector<Point2d> &rightPoints, vector<double> &xyzabc, double &length);
	int computeSwitchPose(const std::vector<Point3d>& p, vector<double> &xyzabc, double &length);

};

class NumberWithIndex
{
  public:
	double data;
	int index;
	NumberWithIndex()
	{
		data = 0.0;
		index = 0;
	}
	NumberWithIndex(double data, int index) : data(data), index(index){};
	bool operator<(NumberWithIndex b)
	{
		return data < b.data;
	}
};

#endif
