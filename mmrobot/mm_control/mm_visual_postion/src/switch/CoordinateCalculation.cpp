#include "mm_visual_postion/switch/CoordinateCalculation.h"

using namespace std;
using namespace cv;

void debugPubStaticTransform(geometry_msgs::TransformStamped static_cam_endeffector_transformStamped){
		static_cam_endeffector_transformStamped.header.stamp = ros::Time::now();
		static_cam_endeffector_transformStamped.header.frame_id = "tool0_controller";
		static_cam_endeffector_transformStamped.child_frame_id = "base";
		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(static_cam_endeffector_transformStamped);
		ros::spinOnce();
}
void debugPubStaticTransform(std::vector<double>& xyzabc, std::string origin_frame_name, std::string target_frame_name){
		geometry_msgs::TransformStamped static_cam_endeffector_transformStamped;
		Eigen::Matrix4d trans;
		createTransformationMatrix(xyzabc, trans);

		static_cam_endeffector_transformStamped.header.stamp = ros::Time::now();
		static_cam_endeffector_transformStamped.header.frame_id = origin_frame_name;
		static_cam_endeffector_transformStamped.child_frame_id = target_frame_name;
		static_cam_endeffector_transformStamped.transform.translation.x = trans(0,3) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.y = trans(1,3) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.z = trans(2,3) / 1000.0;
		Eigen::Quaternion<double> q(trans.block<3,3>(0,0));
		static_cam_endeffector_transformStamped.transform.rotation.x = q.x();
		static_cam_endeffector_transformStamped.transform.rotation.y = q.y();
		static_cam_endeffector_transformStamped.transform.rotation.z = q.z();
		static_cam_endeffector_transformStamped.transform.rotation.w = q.w();
		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(static_cam_endeffector_transformStamped);
		ros::spinOnce();
}
void debugPubCorners(Eigen::Vector4d& corner, std::string pre_fix, int i, std::string origin_frame_name){
		geometry_msgs::TransformStamped static_cam_endeffector_transformStamped;
		stringstream target_frame;
		target_frame <<pre_fix<<i;

		static_cam_endeffector_transformStamped.header.stamp = ros::Time::now();
		static_cam_endeffector_transformStamped.header.frame_id = origin_frame_name;
		static_cam_endeffector_transformStamped.child_frame_id = target_frame.str();
		static_cam_endeffector_transformStamped.transform.translation.x = corner(0) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.y = corner(1) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.z = corner(2) / 1000.0;
		static_cam_endeffector_transformStamped.transform.rotation.w = 1.0;

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(static_cam_endeffector_transformStamped);
		ros::spinOnce();
}

void CoordinateCalculation::resetParticleFilter(){
	particle_filter.reset();
}
void CoordinateCalculation::updateCameraArmParams(std::shared_ptr<StereoCameraArmModel> &ptr)
{
	model_ptr = ptr;
	eigen2cv(model_ptr->left_camera.projection_mat, left_projection_cv_mat);
	eigen2cv(model_ptr->right_camera.projection_mat, right_projection_cv_mat);
	left_transformer_ptr = std::make_shared<CoordinateTransformer>(model_ptr->endeffector_to_cam_transform, model_ptr->left_camera.projection_mat);
	right_transformer_ptr = std::make_shared<CoordinateTransformer>(model_ptr->endeffector_to_cam_transform, model_ptr->right_camera.projection_mat);
}

CoordinateCalculation::CoordinateCalculation()
{
#ifdef BYPASS_TF
	ROS_WARN("TF information is bypassed, be sure to disable BYPASS_TF macro");
#endif

#ifdef DISABLE_PARTICLE_FILTER
	ROS_WARN("particle filter is disabled");
#endif
	/*
	//相机内外参初始化
	image_size = cv::Size(2208, 1242);
	//左相机
	double tem_leftIntrinsic[3][3] = { 1404.6805178647157, 0, 1093.7708076703914, 0, 1404.9562302837180, 650.19805634924830, 0, 0, 1 };
	memcpy(leftIntrinsic, tem_leftIntrinsic, sizeof(tem_leftIntrinsic));

	double tem_leftDistortion[1][5] = { -0.15884502830994698, -0.017019860223729141, -0.00050845821974763127, -0.00049227193638006373, 0.042091851482026002 };
	memcpy(leftDistortion, tem_leftDistortion, sizeof(tem_leftDistortion));

	//世界坐标系定在了左相机的相机坐标系
	double tem_leftRotation[3][3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	memcpy(leftRotation, tem_leftRotation, sizeof(tem_leftRotation));

	double tem_leftTranslation[1][3] = { 0, 0, 0 };
	memcpy(leftTranslation, tem_leftTranslation, sizeof(tem_leftTranslation));

	//右相机
	double tem_rightIntrinsic[3][3] = { 1395.0427725909017, 0, 1065.3453232201614, 0, 1393.9191585585920, 681.76116308075120, 0, 0, 1 };
	memcpy(rightIntrinsic, tem_rightIntrinsic, sizeof(tem_rightIntrinsic));

	double tem_rightDistortion[1][5] = { -0.15815821923496060, -0.023252410765977595, -0.00088122276582216434, -0.00048619722968107478, 0.047581138754438562 };
	memcpy(rightDistortion, tem_rightDistortion, sizeof(tem_rightDistortion));

	//两眼之间的关系
	double tem_rightRotation[3][3] = { 0.99985594561053415, 0.0022064006648113573, 0.016829136144518399, -0.0022230836705065750, 0.99999705589818755, 0.00097267360974158759,
		-0.016826940490128992, -0.0010099460695902113, 0.99985790694612109 };
	memcpy(rightRotation, tem_rightRotation, sizeof(tem_rightRotation));

	double tem_rightTranslation[1][3] = { -120.02236830747624, 0.66187592661751726, -1.5840147731881373 };
	memcpy(rightTranslation, tem_rightTranslation, sizeof(tem_rightTranslation));

	//固定填充矩阵
	double tem_KFillmatrix[1][3] = { 0, 0, 0 };
	memcpy(KFillmatrix, tem_KFillmatrix, sizeof(tem_KFillmatrix));

	double tem_MFillmatrix[1][4] = { 0, 0, 0, 1 };
	memcpy(MFillmatrix, tem_MFillmatrix, sizeof(tem_MFillmatrix));

	mRightIntrinsic = cv::Mat(3, 3, CV_64F, tem_rightIntrinsic).clone(); //see https://stackoverflow.com/questions/44511057/fail-to-assign-a-cvmat-object-in-c-class
	mRightDistortion = cv::Mat(1, 5, CV_64F, tem_rightDistortion).clone();
	mLeftIntrinsic = cv::Mat(3, 3, CV_64F, tem_leftIntrinsic).clone();
	mLeftDistortion = cv::Mat(1, 5, CV_64F, tem_leftDistortion).clone();
	initUndistortRectifyMap(mLeftIntrinsic, mLeftDistortion, cv::Mat(), mLeftIntrinsic, image_size, CV_32FC1, leftMap1, leftMap2);
	initUndistortRectifyMap(mRightIntrinsic, mRightDistortion, cv::Mat(), mRightIntrinsic, image_size, CV_32FC1, rightMap1, rightMap2);



	mLeftRotation = cv::Mat(3, 3, CV_64F, tem_leftRotation).clone();
	mLeftTranslation = cv::Mat(3, 1, CV_64F, tem_leftTranslation).clone();
	mKFillmatrix = cv::Mat(3, 1, CV_64F, tem_KFillmatrix).clone();
	mMFillmatrix = cv::Mat(1, 4, CV_64F, tem_MFillmatrix).clone();
	mLeftRT1 = cv::Mat(3, 4, CV_64F); //左相机M矩阵
	hconcat(mLeftRotation, mLeftTranslation, mLeftRT1);
	mLeftRT = cv::Mat(4, 4, CV_64F); //左相机M矩阵
	vconcat(mLeftRT1, mMFillmatrix, mLeftRT);
	mLeftK = cv::Mat(3, 4, CV_64F); //左相机K矩阵
	hconcat(mLeftIntrinsic, mKFillmatrix, mLeftK);
	mLeftM = mLeftK * mLeftRT;

	mRightRotation = cv::Mat(3, 3, CV_64F, tem_rightRotation).clone();
	mRightTranslation = cv::Mat(3, 1, CV_64F, tem_rightTranslation).clone();
	mRightRT1 = cv::Mat(3, 4, CV_64F); //右相机M矩阵
	hconcat(mRightRotation, mRightTranslation, mRightRT1);
	mRightRT = cv::Mat(4, 4, CV_64F); //右相机M矩阵
	vconcat(mRightRT1, mMFillmatrix, mRightRT);
	mRightK = cv::Mat(3, 4, CV_64F); //左相机K矩阵
	hconcat(mRightIntrinsic, mKFillmatrix, mRightK);
	mRightM = mRightK * mRightRT;



	double toolHcam[4][4] = {0.99984169237257225, -0.0026395303756734569, 0.017220805341667676, -60.218822707932652,
							 0.0018455069589268978, 0.99892779452794767, 0.046059820389033529, -92.795753707993754,
							 -0.017324782747274569, -0.046019621029755459, 0.99878541646990293, 92.137688446275092,
							 0, 0, 0, 1};
	THC = cv::Mat(4, 4, CV_64FC1, toolHcam).clone();

	*/
	//手眼标定结果

}

CoordinateCalculation::~CoordinateCalculation()
{
}

void CoordinateCalculation::uv2cxyzOpencv(const std::vector<Point2d> &uvLeftPoints, const std::vector<Point2d> &uvRightPoints, std::vector<Point3d> &xyzPoints)
{
	cv::Mat resultArray;
	cv::triangulatePoints(left_projection_cv_mat, right_projection_cv_mat, uvLeftPoints, uvRightPoints, resultArray);

	xyzPoints.resize(uvRightPoints.size());
	for (unsigned int i = 0; i < uvRightPoints.size(); i++)
	{
		xyzPoints[i].x = resultArray.at<double>(0, i) / resultArray.at<double>(3, i);
		xyzPoints[i].y = resultArray.at<double>(1, i) / resultArray.at<double>(3, i);
		xyzPoints[i].z = resultArray.at<double>(2, i) / resultArray.at<double>(3, i);
	}
}


//相机坐标系转换到工具坐标系
Point3d CoordinateCalculation::cxyz2bxyz(const Point3d &cxyz)
{
	Eigen::Vector4d point_in_cam;
	point_in_cam << cxyz.x, cxyz.y, cxyz.z, 1.0;
	Eigen::Vector4d point_in_tool = model_ptr->endeffector_to_cam_transform * point_in_cam;

	Point3d cv_point_in_tool;
	cv_point_in_tool.x = point_in_tool(0);
	cv_point_in_tool.y = point_in_tool(1);
	cv_point_in_tool.z = point_in_tool(2);
	return cv_point_in_tool;
}

int CoordinateCalculation::proposeROIs(const cv::Mat &edgeImg, vector<Rect> &roiRects)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(edgeImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //外轮廓
	if (contours.size() <= 0)
	{
		return -1;
	}
	vector<Rect> possibleRoi;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		vector<Point> hull;
		convexHull(contours[i], hull);
		double hullArea = contourArea(hull);
		if (hullArea < 10000)
			continue;
		Rect rect = boundingRect(hull);
		rect.width = fmin(rect.width + 30, edgeImg.cols - rect.x - 15);
		rect.height = fmin(rect.height + 30, edgeImg.rows - rect.y - 15);
		rect.x = fmax(rect.x - 15, 0);
		rect.y = fmax(rect.y - 15, 0); //margin
		possibleRoi.push_back(rect);
	}

	// filter the ROIs
	for (unsigned int i = 0; i < possibleRoi.size(); i++)
	{
		bool is_fused = false;
		for (unsigned int j = 0; j < roiRects.size(); j++)
		{
			cv::Rect intersect_rect = possibleRoi[i] & roiRects[j];
			double overlap_area = intersect_rect.size().width * intersect_rect.size().height;
			if (overlap_area / (possibleRoi[i].size().width * possibleRoi[i].size().height) > 0.8 || overlap_area / (roiRects[j].size().width * roiRects[j].size().height))
			{
				roiRects[j] = possibleRoi[i] | roiRects[j];
				is_fused = true;
				break;
			}
		}
		if (!is_fused)
		{
			roiRects.push_back(possibleRoi[i]);
		}
	}

	return 0;
}

int CoordinateCalculation::getRotateRectVertices(const cv::RotatedRect &rect, const cv::Size imgSize, vector<Point2f> &vertices)
{
	Point2f points[4];
	rect.points(points);

	Point2d center;
	center.x = (points[0].x + points[1].x + points[2].x + points[3].x) / 4.0;
	center.y = (points[0].y + points[1].y + points[2].y + points[3].y) / 4.0;

	bool chosen[4] = {false};
	int left_up_index, right_up_index, right_down_index, left_down_index;
	// find left up
	for (int i = 0; i < 4; i++)
	{
		if (points[i].x < 0 || points[i].y < 0 || points[i].x > imgSize.width || points[i].y > imgSize.height)
			return -1;
		if (chosen[i])
			continue;
		if (points[i].x < center.x && points[i].y < center.y)
		{
			left_up_index = i;
			chosen[i] = true;
		}
		else if (points[i].x < center.x && points[i].y > center.y)
		{
			left_down_index = i;
			chosen[i] = true;
		}
		else if (points[i].x > center.x && points[i].y < center.y)
		{
			right_up_index = i;
			chosen[i] = true;
		}
		else if (points[i].x > center.x && points[i].y > center.y)
		{
			right_down_index = i;
			chosen[i] = true;
		}
	}
	vertices.resize(4);
	vertices[0] = points[left_up_index];
	vertices[1] = points[right_up_index];
	vertices[2] = points[right_down_index];
	vertices[3] = points[left_down_index];
	return 0;
}

int CoordinateCalculation::processSwitchImgHough(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
												 const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, double& match_score, cv::Mat* draw_img_ptr)
{
	/* 
	* use Hough method to find thw switch
	* If don't want this function to draw a image with switch's edge, just assign draw_img_ptr=nullptr
	*/
	int ret = switchFinderByHough.findSwitch(img, tempImgVec, corner_points, match_score, false);
	if(ret >=0 && draw_img_ptr != nullptr){
		drawPolyLines(*draw_img_ptr, corner_points, Scalar(0, 0, 255));
	}
	return ret;
}

int CoordinateCalculation::processSwitchImgMorphology(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
												 const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, cv::Mat* draw_img_ptr)
{
	/* 
	* use Hough method to find thw switch
	* If don't want this function to draw a image with switch's edge, just assign draw_img_ptr=nullptr
	*/
	int ret = processSwitchImgMorphologyImpl(img, tempImgVec,tempMargin, corner_points, false);
	if(ret >=0 && draw_img_ptr != nullptr){
		drawPolyLines(*draw_img_ptr, corner_points, Scalar(0, 255, 0));
	}
	return ret;
}


int CoordinateCalculation::processSwitchImgCombineAvg(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
													  const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, 
													  vector<Point2d>& estimate_var,cv::Mat* draw_img_ptr)
{
	/*
    * combine processSwitchImgHough and processSwitchImgMorphology
	* compare these two methods to valid the result, and if the result is valided, 
	* the output will be the average of these two methods.
	*/
	int ret_value[2] = {0};
	vector<Point2d> method_hough_points, method_morphology_points;
	double match_score;
	if (processSwitchImgHough(img, tempImgVec, tempMargin, method_hough_points,match_score, draw_img_ptr) < 0)
	{
		ROS_INFO("hough method cannot find corresponding switch in left image");
		ret_value[0] = -1;
	}
	if (processSwitchImgMorphology(img, tempImgVec, tempMargin, method_morphology_points, draw_img_ptr) < 0)
	{
		ROS_INFO("hough method cannot find corresponding switch in right image");
		ret_value[1] = -2;
	}
	bool isFound = (ret_value[0] >= 0 && ret_value[1] >= 0);

	estimate_var.resize(4);
	if (isFound)
	{
		for(unsigned int i=0; i<4; i++){
			Point2d diff;
			diff.x = fabs(method_hough_points[i].x - method_morphology_points[i].x)/2.0;
			diff.y = fabs(method_hough_points[i].y - method_morphology_points[i].y)/2.0;
			estimate_var[i] = diff;
		}
		bool compare_result = comparePixelPositions(method_hough_points, method_morphology_points, 10.0);
		if (!compare_result)
		{
			cout << "Points calculated by two methods are not consistent. Please try again." << endl;
		}
		isFound = isFound && compare_result;
		//avgPixelPositions(method_hough_points, method_morphology_points, corner_points);
		corner_points = method_hough_points;
	}
	if(isFound)
		return 0;
	else return -1;
}
int CoordinateCalculation::calculateSwitchPosSimple(const cv::Mat &left_img, const cv::Mat &right_img, const std::vector<cv::Mat> &tempImgVec,
													  const cv::Vec4f &tempMargin, const geometry_msgs::TransformStamped &transform_endeffector_to_base,
													  vector<double> &xyzabc, int use_method, double& match_score, ResultInfo* additional_info_ptr){
	int ret_left ,ret_right = 0;
	vector<Point2d> left_points, right_points;
	cv::Mat* show_left_img_ptr = nullptr;
	cv::Mat* show_right_img_ptr = nullptr; 
	vector<Point2d> left_estimate_var(4); 	vector<Point2d> right_estimate_var(4);
	
	#ifdef VISUALIZATION	
		additional_info_ptr->show_left = left_img.clone();
		additional_info_ptr->show_right = right_img.clone();
		show_left_img_ptr = &additional_info_ptr->show_left;
		show_right_img_ptr = &additional_info_ptr->show_right;
	#endif
	switch(use_method){
		case METHOD_HOUGH:{
			double left_match_score, right_match_score;
			ret_left = processSwitchImgHough(left_img, tempImgVec, tempMargin, left_points, left_match_score, show_left_img_ptr);
			ret_right = processSwitchImgHough(right_img, tempImgVec, tempMargin, right_points, right_match_score, show_right_img_ptr);
			match_score = (left_match_score + right_match_score)/2.0;
			break;
		}
		case METHOD_MORPHOLOGY:{
			ret_left = processSwitchImgMorphology(left_img, tempImgVec, tempMargin, left_points, show_left_img_ptr);
			ret_right = processSwitchImgMorphology(right_img, tempImgVec, tempMargin, right_points, show_right_img_ptr);
			break;
		}
		case METHOD_COMBINE_HOUGH_MORPH:{
			ret_left = processSwitchImgCombineAvg(left_img, tempImgVec, tempMargin, left_points,left_estimate_var, show_left_img_ptr);
			ret_right = processSwitchImgCombineAvg(right_img, tempImgVec, tempMargin, right_points, right_estimate_var,show_right_img_ptr);
			break;
		}
	}
	#ifdef VISUALIZATION
		cv::Mat showImg;

		if(ret_left < 0 || ret_right < 0){
			cv::hconcat(*show_left_img_ptr, *show_right_img_ptr, showImg);
			cv::putText(showImg, "not found!", cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, CV_RGB(255, 0, 0), 3);
			namedWindow("img", 0);
			imshow("img", showImg);
			waitKey(10);
		}
	#endif

	if(ret_left < 0 || ret_right < 0){
		if(additional_info_ptr != nullptr){
			additional_info_ptr->success = false;
		}
		return -1;
	} 
	std::vector<double> xyzabc_in_cam(6);
	left_transformer_ptr->setTransEndEffectorToBase(transform_endeffector_to_base);
	right_transformer_ptr->setTransEndEffectorToBase(transform_endeffector_to_base);
	double length;

	computeSwitchPoseInCameraCoord(left_points, right_points, xyzabc_in_cam, length);
	//debugPubStaticTransform(transform_endeffector_to_base);
	//debugPubStaticTransform(xyzabc_in_cam, "camera", "object_from_cam");
	std::vector<double> xyzabc_in_base;
	left_transformer_ptr->getObjPoseInBaseFromObjPoseInCam(xyzabc_in_cam, xyzabc_in_base);
	//debugPubStaticTransform(xyzabc_in_base, "base", "object");
	std::cout<<"xyzabc in base: ";
	for(unsigned int xyzabc_i=0; xyzabc_i<6; xyzabc_i++){
		cout<<xyzabc_in_base[xyzabc_i]<<" ";
	}
	std::cout<<std::endl;

	#ifdef VISUALIZATION
		cv::hconcat(*show_left_img_ptr, *show_right_img_ptr, showImg);
		cv::putText(showImg, "    found!", cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, CV_RGB(0, 255, 0), 3);
		namedWindow("img", 0);
		imshow("img", showImg);
		waitKey(10);
	#endif

	// transform to tool coordinatepoint
	Point3d cxyz(xyzabc_in_cam[0], xyzabc_in_cam[1], xyzabc_in_cam[2]);
	Point3d bxyz = cxyz2bxyz(cxyz);
	xyzabc.resize(6);
	xyzabc[0] = bxyz.x;
	xyzabc[1] = bxyz.y;
	xyzabc[2] = bxyz.z;
	xyzabc[3] = xyzabc_in_cam[3];
	xyzabc[4] = xyzabc_in_cam[4];
	xyzabc[5] = xyzabc_in_cam[5];

	if(additional_info_ptr != nullptr){
		additional_info_ptr->success = true;
		additional_info_ptr->left_corners_in_pixel = left_points;
		additional_info_ptr->right_corners_in_pixel = right_points;
		std::vector<cv::Point3d> corners_in_cam;
		uv2cxyzOpencv(left_points, right_points, corners_in_cam);
		additional_info_ptr->corners_in_base.resize(4);
		for(unsigned int corner_i = 0; corner_i < corners_in_cam.size(); corner_i++){
			Eigen::Vector4d point_in_cam, point_in_base;
			point_in_cam << corners_in_cam[corner_i].x , corners_in_cam[corner_i].y , corners_in_cam[corner_i].z ,1.0;
			left_transformer_ptr->getPosInBaseFromPosInCam(point_in_cam, point_in_base);
			additional_info_ptr->corners_in_base[corner_i] = point_in_base;
			debugPubCorners(point_in_base, "corner_", corner_i, "base");

			debugPubCorners(point_in_cam, "corner_cam_", corner_i, "camera");
		}
		additional_info_ptr->xyz_abc_in_cam = xyzabc_in_cam;
		additional_info_ptr->xyz_abc_in_tool = xyzabc;
		additional_info_ptr->xyz_abc_in_base = xyzabc_in_base;

		std::vector<cv::Point3d> cv_corners_in_base;
        cv_corners_in_base.resize(4);
        for(unsigned int i=0; i<4; i++){
            cv_corners_in_base[i].x = additional_info_ptr->corners_in_base[i](0);
            cv_corners_in_base[i].y = additional_info_ptr->corners_in_base[i](1);
            cv_corners_in_base[i].z = additional_info_ptr->corners_in_base[i](2);
        }
        double length; std::vector<double> xyzabc_in_base_method;
		computeSwitchPose(cv_corners_in_base, xyzabc_in_base_method, length);
		//debugPubStaticTransform(xyzabc_in_base_method, "base", "object_new");


	}


	return 0;
}
int CoordinateCalculation::calculateSwitchPosParticleFilter(const cv::Mat &left_img, const cv::Mat &right_img, const std::vector<cv::Mat> &tempImgVec,
													  const cv::Vec4f &tempMargin, const geometry_msgs::TransformStamped &transform_endeffector_to_base,
													  vector<double> &xyzabc, int use_method, ResultInfo* additional_info_ptr)
{
	bool particle_filter_initialized = particle_filter.initialized();

	if (!particle_filter_initialized)
	{
		last_transform_endeffector_to_base = transform_endeffector_to_base;
		last_update_transform_endeffector_to_base = transform_endeffector_to_base;
	}
	bool moved = isMoved(last_transform_endeffector_to_base, transform_endeffector_to_base, 0.002, 0.005);
	bool is_moving = isMoved(last_update_transform_endeffector_to_base, transform_endeffector_to_base, 0.002, 0.005);
	last_update_transform_endeffector_to_base = transform_endeffector_to_base;
	int ret_left ,ret_right = 0;
	vector<Point2d> left_points, right_points;
	cv::Mat* show_left_img_ptr = nullptr;
	cv::Mat* show_right_img_ptr = nullptr; 
	vector<Point2d> left_estimate_var(4); 	vector<Point2d> right_estimate_var(4);
	for(unsigned int i=0; i<4; i++){
		left_estimate_var[i].x = 5.0; //in pixel, value by default, these values will be overwrited if METHOD_COMBINE_HOUGH_MORPH is used
		left_estimate_var[i].y = 5.0;
		right_estimate_var[i].x =5.0; 
		right_estimate_var[i].y = 5.0;
	}
	
	#ifdef VISUALIZATION	
		if(additional_info_ptr != nullptr){
			additional_info_ptr->show_left = left_img.clone();
			additional_info_ptr->show_right = right_img.clone();
			show_left_img_ptr = &additional_info_ptr->show_left;
			show_right_img_ptr = &additional_info_ptr->show_right;
		}
	#endif
	double match_score;
	switch(use_method){
		case METHOD_HOUGH:{
			ret_left = processSwitchImgHough(left_img, tempImgVec, tempMargin, left_points, match_score,show_left_img_ptr);
			ret_right = processSwitchImgHough(right_img, tempImgVec, tempMargin, right_points, match_score,show_right_img_ptr);
			break;
		}
		case METHOD_MORPHOLOGY:{
			ret_left = processSwitchImgMorphology(left_img, tempImgVec, tempMargin, left_points, show_left_img_ptr);
			ret_right = processSwitchImgMorphology(right_img, tempImgVec, tempMargin, right_points, show_right_img_ptr);
			break;
		}
		case METHOD_COMBINE_HOUGH_MORPH:{
			ret_left = processSwitchImgCombineAvg(left_img, tempImgVec, tempMargin, left_points,left_estimate_var, show_left_img_ptr);
			ret_right = processSwitchImgCombineAvg(right_img, tempImgVec, tempMargin, right_points, right_estimate_var,show_right_img_ptr);
			break;
		}
	}
	#ifdef VISUALIZATION
		cv::Mat showImg;

		if(additional_info_ptr != nullptr){
			if(ret_left < 0 || ret_right < 0){
				cv::hconcat(*show_left_img_ptr, *show_right_img_ptr, showImg);
				cv::putText(showImg, "not found!", cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, CV_RGB(255, 0, 0), 3);
				namedWindow("img", 0);
				imshow("img", showImg);
				waitKey(10);
			}
		}
	#endif
	if(ret_left < 0 || ret_right < 0){
		if(additional_info_ptr != nullptr){
			additional_info_ptr->success = false;
		}
		return -1;
	} 

	std::vector<double> xyzabc_in_cam(6);
	left_transformer_ptr->setTransEndEffectorToBase(transform_endeffector_to_base);
	right_transformer_ptr->setTransEndEffectorToBase(transform_endeffector_to_base);
	double length;

	// initilize/reset particles
	computeSwitchPoseInCameraCoord(left_points, right_points, xyzabc_in_cam, length);
	std::vector<double> xyzabc_in_base;
	left_transformer_ptr->getObjPoseInBaseFromObjPoseInCam(xyzabc_in_cam, xyzabc_in_base);
	Particle possible_particle;
	possible_particle.setTranslation(xyzabc_in_base[0], xyzabc_in_base[1], xyzabc_in_base[2]);
	possible_particle.fromEulerAngles(xyzabc_in_base[3], xyzabc_in_base[4], xyzabc_in_base[5]);
	possible_particle.l = length;
	if (!particle_filter_initialized)
	{
		StdPos std_pos(5, 5, 5, 0.02, 5); // mm, mm, mm, quaternion value, mm
		//StdPos std_pos(0.0001, 0.0001, 0.0001, 0.0001, 0.0001); // mm, mm, mm, quaternion value, mm
		
		particle_filter.init(1000, possible_particle, std_pos, left_transformer_ptr, right_transformer_ptr);
		particle_filter.getResult(result_particle);
	}
	else if (moved && ! is_moving)
	{
		StdPos predict_std_pos(0.3, 0.3, 0.3, 0.01, 0.1);
		particle_filter.prediction(predict_std_pos);
		double error_x = 0;
		double error_y = 0;
		for (int p_i = 0; p_i < 4; p_i++)
		{
			error_x += right_estimate_var[p_i].x + left_estimate_var[p_i].x;
			error_y += right_estimate_var[p_i].y + left_estimate_var[p_i].y;
		}
		error_x = error_x / 8.0;
		error_y = error_y / 8.0;

		std::vector<Point2d> pixel_points;
		for (int p_i = 0; p_i < 4; p_i++)
		{
			pixel_points.push_back(left_points[p_i]);
			pixel_points.push_back(right_points[p_i]);
		}
		particle_filter.updateWeights(StdPixel(error_x, error_y), pixel_points);
		particle_filter.getResult(result_particle);
		StdPos reset_std_pos(5, 5, 5, 0.1,5); // mm, mm, mm, quaternion value, mm
		particle_filter.resample(possible_particle, reset_std_pos);
		xyzabc_in_base[0] = result_particle.x;
		xyzabc_in_base[1] = result_particle.y;
		xyzabc_in_base[2] = result_particle.z;
		double rot_x, rot_y, rot_z;
		result_particle.toEulerAngles(rot_x, rot_y, rot_z); // extrinsic XYZ (or intrinsic zyx) rotation
		xyzabc_in_base[3] = rot_x;
		xyzabc_in_base[4] = rot_y;
		xyzabc_in_base[5] = rot_z;
		last_transform_endeffector_to_base = transform_endeffector_to_base;
	}
	
	std::vector<cv::Point2d> inverse_left_img_corners, inverse_right_img_corners;

	if (particle_filter_initialized)
	{
		particle_filter.computeCornersPosInCamPixel(&result_particle, inverse_left_img_corners, inverse_right_img_corners);
		#ifdef VISUALIZATION
		if(additional_info_ptr != nullptr){
			drawPolyLines(*show_left_img_ptr, inverse_left_img_corners, Scalar(255, 0, 0));
			drawPolyLines(*show_right_img_ptr, inverse_right_img_corners, Scalar(255, 0, 0));
		}
		#endif
	}
	#ifdef VISUALIZATION
		if(additional_info_ptr != nullptr){
			cv::hconcat(*show_left_img_ptr, *show_right_img_ptr, showImg);
			cv::putText(showImg, "    found!", cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, CV_RGB(0, 255, 0), 3);
			namedWindow("img", 0);
			imshow("img", showImg);
			waitKey(10);
		}
	#endif


	// transform to tool coordinate
	left_transformer_ptr->setTransEndEffectorToBase(transform_endeffector_to_base);
	left_transformer_ptr->getObjPoseInCamFromObjPoseInBase(xyzabc_in_base, xyzabc_in_cam);
	Point3d cxyz(xyzabc_in_cam[0], xyzabc_in_cam[1], xyzabc_in_cam[2]);

	//debugPubStaticTransform(xyzabc_in_base, "base", "object_pf");
	//debugPubStaticTransform(xyzabc_in_cam, "camera", "object_pf_camera");
	//debugPubStaticTransform(transform_endeffector_to_base);
	Point3d bxyz = cxyz2bxyz(cxyz);
	xyzabc.resize(6);
	xyzabc[0] = bxyz.x;
	xyzabc[1] = bxyz.y;
	xyzabc[2] = bxyz.z;
	xyzabc[3] = xyzabc_in_cam[3];
	xyzabc[4] = xyzabc_in_cam[4];
	xyzabc[5] = xyzabc_in_cam[5];
	if(additional_info_ptr != nullptr){
		additional_info_ptr->success = true;
		additional_info_ptr->left_corners_in_pixel = inverse_left_img_corners;
		additional_info_ptr->right_corners_in_pixel = inverse_right_img_corners;
		particle_filter.computeCornersPosInBase(&result_particle, additional_info_ptr->corners_in_base);
		additional_info_ptr->xyz_abc_in_cam = xyzabc_in_cam;
		additional_info_ptr->xyz_abc_in_tool = xyzabc;
		additional_info_ptr->xyz_abc_in_base = xyzabc_in_base;

	}
	return 0;
}

//开关处理
int CoordinateCalculation::processSwitchImgMorphologyImpl(const cv::Mat &img, const std::vector<cv::Mat> &tempImgVec,
													  const cv::Vec4f &tempMargin, vector<Point2d> &corner_points, bool visualization)
{
	/**
     *  img should be corrected firstly. 
     * 
     * 
     */
	// find ROI
	cv::Mat imgGray;

	cvtColor(img, imgGray, CV_RGB2GRAY);
	cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	//滤波
	blur(imgGray, imgGray, Size(5, 5));
	adaptiveThreshold(imgGray, imgGray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 3);
	//erode(imgGray, imgGray, element); //膨胀

	cv::Mat result;
	Canny(imgGray, result, 30, 90);
	//imshow("1", result);
	//waitKey(0);
	vector<Rect> ROIs;
	if (proposeROIs(result, ROIs) < 0)
		return -1;

	vector<vector<Point2d>> possibleCorners;
	possibleCorners.resize(ROIs.size());
	vector<double> matchScores;
	matchScores.resize(ROIs.size());
	for (unsigned int roi_i = 0; roi_i < ROIs.size(); roi_i++)
	{
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		cv::Mat debug_roi = img(ROIs[roi_i]);
		findContours(result(ROIs[roi_i]), contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //外轮廓
		std::vector<int> filteredContoursIndex;

		if (contours.size() < 1)
			continue;
		else if (contours.size() == 1)
		{
			filteredContoursIndex.push_back(0);
		}
		else if (contours.size() >= 2)
		{
			// find two contours with maximum area
			std::vector<NumberWithIndex> areaOfContours;
			areaOfContours.resize(contours.size());
			for (unsigned int i = 0; i < contours.size(); i++)
			{
				vector<Point> hull;
				convexHull(contours[i], hull);
				areaOfContours[i].data = contourArea(hull);
				areaOfContours[i].index = i;
			}
			if (contours.size() > 2)
			{
				std::nth_element(areaOfContours.begin(), areaOfContours.begin() + (areaOfContours.size() - 2), areaOfContours.end());
				filteredContoursIndex.push_back(areaOfContours[areaOfContours.size() - 1].index);
				filteredContoursIndex.push_back(areaOfContours[areaOfContours.size() - 2].index);
			}
			else
			{
				filteredContoursIndex.push_back(0);
				filteredContoursIndex.push_back(1);
			}
		}
		cv::Mat drawing;
		if (!visualization)
		{
			cout << "contours size: " << contours.size() << endl;
			drawing = cv::Mat(imgGray(ROIs[roi_i]).size(), CV_8U, Scalar(255));
			drawContours(drawing, contours, filteredContoursIndex[0], Scalar(0), 1);
			if (filteredContoursIndex.size() > 1)
				drawContours(drawing, contours, filteredContoursIndex[1], Scalar(125), 1);
			//imshow("xx", drawing);
			//waitKey(0);
		}

		RotatedRect rr0;
		if (filteredContoursIndex.size() == 1)
		{
			rr0 = minAreaRect(cv::Mat(contours[filteredContoursIndex[0]]));
		}
		else
		{
			// choose inner contour
			RotatedRect r1 = minAreaRect(cv::Mat(contours[filteredContoursIndex[0]]));
			Point2f v1[4]; //计算四个顶点
			r1.points(v1);
			double a1 = sqrt((v1[0].x - v1[1].x) * (v1[0].x - v1[1].x) + (v1[0].y - v1[1].y) * (v1[0].y - v1[1].y));
			double b1 = sqrt((v1[2].x - v1[1].x) * (v1[2].x - v1[1].x) + (v1[2].y - v1[1].y) * (v1[2].y - v1[1].y));
			double area1 = a1 * b1;
			RotatedRect r2 = minAreaRect(cv::Mat(contours[filteredContoursIndex[1]]));
			Point2f v2[4]; //计算四个顶点
			r2.points(v2);
			double a2 = sqrt((v2[0].x - v2[1].x) * (v2[0].x - v2[1].x) + (v2[0].y - v2[1].y) * (v2[0].y - v2[1].y));
			double b2 = sqrt((v2[2].x - v2[1].x) * (v2[2].x - v2[1].x) + (v2[2].y - v2[1].y) * (v2[2].y - v2[1].y));
			double area2 = a2 * b2;
			int dec = 0; //选定内框边缘
			if ((a1 / b1) > 0.87 && (a1 / b1) < 1.15 && area1 > 12000)
				if ((a2 / b2) > 0.87 && (a2 / b2) < 1.15)
					if (area1 > area2 && area2 > 12000)
						dec = 1;
					else
						dec = 0;
				else
					dec = 0;
			else if ((a2 / b2) > 0.87 && (a2 / b2) < 1.15 && area2 > 12000)
				dec = 1;
			else
			{
				continue;
			}
			dec == 0 ? rr0 = r1 : rr0 = r2;
		}

		vector<Point2f> vertices; //计算四个顶点
		if (getRotateRectVertices(rr0, cv::Size(ROIs[roi_i].width, ROIs[roi_i].height), vertices) < 0)
			continue;

		if (visualization)
		{
			cout << "角度" << rr0.angle << endl;
			for (int vert_i = 0; vert_i < 4; vert_i++)
			{
				line(drawing, vertices[vert_i], vertices[(vert_i + 1) % 4], Scalar(0, 255, 0), 2);
				//cout << vertices[i] << endl;
			}
			imshow("1", drawing);
			waitKey(0);
		}

		vector<Point2d> corners;
		corners.resize(4);
		for (int i = 0; i < 4; i++)
		{
			corners[i].x = vertices[i].x + ROIs[roi_i].x;
			corners[i].y = vertices[i].y + ROIs[roi_i].y;
		}
		//	corners.push_back(left_up_corner);
		//corners.push_back(right_up_corner);
		//corners.push_back(right_down_corner);
		//corners.push_back(left_down_corner);
		double matchVal = 0.0;
		Point matchLoc;

		// calculate match score by using template matching
		for(unsigned int template_i = 0; template_i < tempImgVec.size(); template_i++){
			vector<cv::Point2f> template_corners(4);
			cv::Mat tempImg = tempImgVec[template_i];
			template_corners[0] = cv::Point2f(0, 0);												  //left_up_corner
			template_corners[1] = cv::Point2f(tempImg.size().width, 0);						  //right_up_corner
			template_corners[2] = cv::Point2f(tempImg.size().width, tempImg.size().height); //right_down_corner
			template_corners[3] = cv::Point2f(0, tempImg.size().height);						  //left_down_corner
			cv::Mat transform_matrix = cv::getPerspectiveTransform(vertices, template_corners);
			cv::Mat corrected_switch;
			cv::warpPerspective(img(ROIs[roi_i]), corrected_switch, transform_matrix, cv::Size(tempImg.size().width, tempImg.size().height));
			cv::Point match_loc;
			double match_tmp;
			matchTemplateElastic(corrected_switch, tempImg, 1.0, match_tmp, matchLoc);
			if(match_tmp > matchVal){
				matchVal = match_tmp;
			}
		}
		matchScores[roi_i] = matchVal;
		possibleCorners[roi_i] = corners;
	}
	if (matchScores.size() <= 0)
		return -1;
	std::vector<double>::iterator maxMatchScore = std::max_element(matchScores.begin(), matchScores.end());

	corner_points = possibleCorners[std::distance(matchScores.begin(), maxMatchScore)];
	std::cout << "morphology method: max match score " << *maxMatchScore << std::endl;
	if (*maxMatchScore < 0.7)
	{
		std::cout << "morphology method: not sure the switch found is the correct switch, abort" << std::endl;
		return -1; //treshold
	}
	std::cout << "morphology method: all scores: ";
	std::sort(matchScores.begin(), matchScores.end());
	for (int i = matchScores.size() - 1; i >= 0; i--)
	{
		std::cout << matchScores[i] << " ";
	}
	std::cout << endl;

	return 0;
}

bool CoordinateCalculation::comparePixelPositions(const vector<Point2d> &method1_points, const vector<Point2d> &method2_points, double acceptable_error)
{
	for (unsigned int i = 0; i < method1_points.size(); i++)
	{
		if (fabs(method1_points[i].x - method2_points[i].x) > acceptable_error || fabs(method1_points[i].y - method2_points[i].y) > acceptable_error)
		{
			return false;
		}
	}
	return true;
}
void CoordinateCalculation::avgPixelPositions(const vector<Point2d> &method1_points, const vector<Point2d> &method2_points, vector<Point2d> &avg_points)
{
	assert(method1_points.size() == method2_points.size());
	avg_points.resize(method1_points.size());
	for (unsigned int i = 0; i < method1_points.size(); i++)
	{
		double avg_x = (method1_points[i].x + method2_points[i].x) / 2.0;
		double avg_y = (method1_points[i].y + method2_points[i].y) / 2.0;
		avg_points[i] = Point2d(avg_x, avg_y);
	}
}

void drawPolyLines(cv::Mat draw_img, vector<Point2d> points, Scalar rgb)
{
	vector<Point> pts;
	pts.resize(points.size()); //remove center point
	for (unsigned int i = 0; i < points.size(); i++)
	{
		pts[i].x = (int)round(points[i].x);
		pts[i].y = (int)round(points[i].y);
	}
	polylines(draw_img, pts, true, rgb, 2);
}

bool isMoved(const geometry_msgs::TransformStamped &last_transform, const geometry_msgs::TransformStamped &transform, double translation_threshold, double rotation_threshold)
{
	if (fabs(last_transform.transform.translation.x - transform.transform.translation.x) < translation_threshold && fabs(last_transform.transform.translation.y - transform.transform.translation.y) < translation_threshold && fabs(last_transform.transform.translation.z - transform.transform.translation.z) < translation_threshold && fabs(last_transform.transform.rotation.x - transform.transform.rotation.x) < rotation_threshold && fabs(last_transform.transform.rotation.y - transform.transform.rotation.y) < rotation_threshold && fabs(last_transform.transform.rotation.z - transform.transform.rotation.z) < rotation_threshold)
		return false;
	else
		return true;
}
int CoordinateCalculation::computeSwitchPose(const std::vector<Point3d>& p, vector<double> &xyzabc, double &length){
	/*
	p is the points of square, xyzabc is the 3d pose of the center of square, length is the length of the square
	p and xyzabc are presented in the same coordinate
	*/

	xyzabc.resize(6);
    RigidRectTransformSolver solver(50,50); // mm
    solver.setPointSets(p);
    solver.solveTransform(xyzabc, length);
	return 0;
}
int CoordinateCalculation::computeSwitchPoseInCameraCoord(const vector<Point2d> &leftPoints, const vector<Point2d> &rightPoints, vector<double> &xyzabc, double &length)
{
	std::vector<Point3d> p;
	uv2cxyzOpencv(leftPoints, rightPoints, p);
	computeSwitchPose(p, xyzabc, length);
	return 0;

}

//开关2的开关档位识别
int CoordinateCalculation::recognizeSwitchState(const cv::Mat &img, const cv::Mat &tempImg, const cv::Vec4f &tempMargin)
{
	/*
    *   Img should be corrected firstly
    * 
    */

	Point2d MatchT;
	cv::Rect2f rectPos;
	if (matchSelect(img, tempImg, tempMargin, MatchT, rectPos, true))
	{
		return -1;
	}
	int tx = MatchT.x;
	int ty = MatchT.y;
	Rect rect(tx, ty, 500, 500);
	/*rectangle(src, rect, Scalar(255, 0, 0), 2);
	resize(src, src, Size(src.cols / 4, src.rows / 4));
	imshow("111", src);
	waitKey();*/
	cv::Mat imgCut = cv::Mat(img, rect);
	cv::Mat imgCopy = imgCut.clone();
	//Canny算子
	cv::Mat result;
	cv::Mat imgGray;
	//imgCopy=gamaChange(imgCopy);
	cvtColor(imgCopy, imgGray, CV_RGB2GRAY);
	//threshold(imgGray, imgGray, 60, 255, THRESH_BINARY);
	threshold(imgGray, imgGray, 0, 255, CV_THRESH_OTSU);
	imshow("1", imgGray);
	waitKey(10);
	//int edgeThresh = 30;
	//滤波
	//blur(imgGray, imgGray, Size(5, 5));
	//膨胀腐蚀
	cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
	//erode(imgGray, imgGray, element);//膨胀
	dilate(imgGray, imgGray, element); //腐蚀

	/*blur(imgGray, imgGray, Size(3, 3));*/
	/*Canny(imgGray, result, edgeThresh, edgeThresh * 3);*/
	Canny(imgGray, result, 30, 90);
	/*imshow("1", result);
	waitKey(0);*/
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(result, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //外轮廓
	//findContours(result, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//所有轮廓
	cv::Mat redrawing(imgGray.size(), CV_8U, Scalar(255));
	cout << contours.size() << endl;
	drawContours(redrawing, contours, -1, Scalar(0), 1);
	imshow("xx", redrawing);
	waitKey(10);

	unsigned int cmin = 200;   //最小轮廓长度
	unsigned int cmax = 10000; //最大轮廓
	do
	{
		vector<vector<Point>>::iterator itc = contours.begin();
		while (itc != contours.end())
		{
			if (itc->size() < cmin || itc->size() > cmax)
			{
				itc = contours.erase(itc);
				if (contours.size() == 1)
					break;
			}
			else
				++itc;
		}
		cmin += 1;
	} while (contours.size() > 5);
	cout << contours.size() << endl;
	cv::Mat drawing(imgGray.size(), CV_8U, Scalar(255));
	drawContours(drawing, contours, -1, Scalar(0), 1);
	imshow("xx", drawing);
	waitKey(10);

	unsigned int cc = 1;
	int t = -1;
	while (cc <= contours.size())
	{
		RotatedRect box = fitEllipse(contours[cc - 1]);
		Point2f v1[4]; //计算四个顶点
		box.points(v1);
		double a = sqrt((v1[0].x - v1[1].x) * (v1[0].x - v1[1].x) + (v1[0].y - v1[1].y) * (v1[0].y - v1[1].y));
		double b = sqrt((v1[2].x - v1[1].x) * (v1[2].x - v1[1].x) + (v1[2].y - v1[1].y) * (v1[2].y - v1[1].y));
		double area = a * b;
		if (fabs(box.angle) > 115 && fabs(box.angle) < 145 && area < 40000 && area > 16000)
		{
			t = cc - 1;
		}
		else if (fabs(box.angle) > 30 && fabs(box.angle) < 60 && area < 40000 && area > 16000)
		{
			t = cc - 1;
		}
		cc++;
	}

	if (t == -1)
	{
		cout << "开关档位识别错误! 请重试" << endl;
	}
	RotatedRect box = fitEllipse(contours[t]);
	ellipse(drawing, box, Scalar(0, 0, 255), 3, CV_AA);
	cout << box.angle << endl;
	imshow("1", drawing);
	waitKey(0);
	if (box.angle > 90)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}


//检验开关识别是否正确
bool CoordinateCalculation::isSwitchRecognitionCorrect(double xyzabc1[6], double xyzabc2[6])
{
	double distance = sqrt((xyzabc2[0] - xyzabc1[0]) * (xyzabc2[0] - xyzabc1[0]) + (xyzabc2[1] - xyzabc1[1]) * (xyzabc2[1] - xyzabc1[1]) + (xyzabc2[2] - xyzabc1[2]) * (xyzabc2[2] - xyzabc1[2]));
	//cout << fabs(xyzabc1[3] - xyzabc2[3]) << endl;
	cout << "两开关之间距离为" << distance << endl;
	if (fabs(xyzabc1[3] - xyzabc2[3]) > 0.05)
	{
		cout << "开关识别失败，请调整拍照位置重试！" << endl;
		return false;
	}
	else if (fabs(xyzabc1[4] - xyzabc2[4]) > 0.05)
	{
		cout << "开关识别失败，请调整拍照位置重试！" << endl;
		return false;
	}
	else if (fabs(xyzabc1[5] - xyzabc2[5]) > 0.05)
	{
		cout << "开关识别失败，请调整拍照位置重试！" << endl;
		return false;
	}
	else if (distance > 139 && distance < 141)
	{
		cout << "开关识别成功！" << endl;
		return true;
	}
	else
	{
		cout << "开关识别失败，请调整拍照位置重试！" << endl;
		return false;
	}
}

int CoordinateCalculation::matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat &template_img, const double &size_factor, double &match_val, cv::Point &match_loc)
{
	double min_val, max_val;
	cv::Point min_loc, max_loc;
	double match_max_val = -1.0;
	cv::Point match_max_loc;
	for (double n = size_factor - 0.2 > 0 ? size_factor - 0.2 > 0 : 0.04; n < (size_factor + 0.1); n += 0.04)
	{
		cv::Mat template_img_resized;
		resize(template_img, template_img_resized, cv::Size(template_img.cols * n, template_img.rows * n));
		cv::Mat resultx;
		int result_cols = switch_img.cols - template_img_resized.cols + 1;
		int result_rows = switch_img.rows - template_img_resized.rows + 1;
		if (result_cols <= 0 || result_rows <= 0)
			continue;
		resultx.create(result_cols, result_rows, CV_32F);
		matchTemplate(switch_img, template_img_resized, resultx, CV_TM_CCOEFF_NORMED);
		minMaxLoc(resultx, &min_val, &max_val, &min_loc, &max_loc);
		//cout << minVal << endl;
		//cout << maxVal << endl;
		match_max_val = match_max_val > max_val ? match_max_val : max_val;
		match_max_loc = max_loc;
	}
	match_loc = match_max_loc;
	match_val = match_max_val;
	if (match_max_val < 0)
	{
		return -1;
	}
	else
		return 0;
}

int CoordinateCalculation::matchSelect(const cv::Mat &img, const cv::Mat &tem, const cv::Vec4f &temMargin, Point2d &ret, cv::Rect2f &rect_pos, bool visualization)
{
	cv::Mat imgGray;

	cvtColor(img, imgGray, CV_RGB2GRAY);
	cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
	erode(imgGray, imgGray, element); //膨胀
	//滤波
	blur(imgGray, imgGray, Size(5, 5));
	//cv::adaptiveThreshold(imgGray,imgGray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,7,5);
	threshold(imgGray, imgGray, 0, 255, CV_THRESH_OTSU);
	//threshold(imgGray, imgGray, 70, 255, THRESH_BINARY);
	if (visualization)
	{
		imshow("xx1", imgGray);
		waitKey(0);
	}
	//int edgeThresh = 50;

	cv::Mat result;
	Canny(imgGray, result, 30, 90);
	//imshow("1", result);
	//waitKey(0);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(result, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //外轮廓

	cout << contours.size() << endl;
	cv::Mat drawing(imgGray.size(), CV_8U, Scalar(255));
	cv::Mat drawing1;

	if (visualization)
	{
		drawing1 = drawing.clone();
		drawContours(drawing1, contours, -1, Scalar(0), 4);
		resize(drawing1, drawing1, Size(drawing1.cols / 2, drawing1.rows / 2));
		imshow("xx", drawing1);
		waitKey(0);
	}

	vector<Point2d> loc;
	vector<cv::Rect2f> rectLoc;
	vector<double> Mmax;
	if (contours.size() <= 0)
	{
		return -1;
	}
	for (unsigned int i = 0; i < contours.size(); i++)
	{

		vector<Point> hull;
		convexHull(contours[i], hull);
		double hullArea = contourArea(hull);
		if (hullArea < 10000)
			continue;
		float radius;
		Point2f center;
		minEnclosingCircle(cv::Mat(contours[i]), center, radius);
		//minEnclosingCircle(cv::Mat(contours[0]), center[i], radius[i]);
		int tx = (center.x - 250) > 0 ? (center.x - 250) : 0;
		tx = tx < 1708 ? tx : 1708;
		int ty = (center.y - 250) > 0 ? (center.y - 250) : 0;
		ty = ty < 742 ? ty : 742;
		//cout << tx <<endl<< ty << endl;
		Rect rect(tx, ty, fmin(radius * 2 + 100, img.cols - tx), fmin(radius * 2 + 100, img.rows - ty));
		cv::Mat imgCut = cv::Mat(img, rect);
		cv::Mat imgCopy = imgCut.clone();

		double MatchMax = 0;
		double minVal = 0;
		double maxVal = 0;
		Point minLoc;
		Point maxLoc;
		Point matchLoc;
		double matchSizeFactor;
		const Size size = tem.size();
		cv::Mat imgCopy1 = imgCopy.clone();
		double Size_factor = sqrt(hullArea / (size.height * size.width));

		for (double n = (Size_factor - 0.2) <= 0 ? Size_factor : 0.04; n < (Size_factor + 0.1); n += 0.04)
		{
			cv::Mat temp1;
			resize(tem, temp1, Size(tem.cols * n, tem.rows * n));
			cv::Mat resultx;
			int result_cols = imgCopy.cols - temp1.cols + 1;
			int result_rows = imgCopy.rows - temp1.rows + 1;
			if (result_cols <= 0 || result_rows <= 0)
				continue;
			resultx.create(result_cols, result_rows, CV_32FC1);
			matchTemplate(imgCopy, temp1, resultx, CV_TM_CCOEFF_NORMED);
			minMaxLoc(resultx, &minVal, &maxVal, &minLoc, &maxLoc);
			//cout << minVal << endl;
			//cout << maxVal << endl;
			MatchMax = MatchMax > maxVal ? MatchMax : maxVal;
			matchLoc = maxLoc;
			matchSizeFactor = n;
		}
		rectLoc.push_back(cv::Rect2f(matchLoc.x + tx + temMargin[0] * matchSizeFactor, matchLoc.y + ty + temMargin[2] * matchSizeFactor,
									 (tem.cols - temMargin[1] - temMargin[0]) * matchSizeFactor, (tem.rows - temMargin[2] - temMargin[3]) * matchSizeFactor));
		loc.push_back(Point2d(tx, ty));
		cout << "匹配最大" << MatchMax << endl;
		Mmax.push_back(MatchMax);

		if (visualization)
		{
			cv::drawContours(imgCopy1, contours, i, Scalar(255, 0, 0), 4, 8, cv::noArray(), 2147483647, Point(-tx, -ty));

			rectangle(imgCopy1, matchLoc, Point(matchLoc.x + tem.cols * matchSizeFactor, matchLoc.y + tem.rows * matchSizeFactor), Scalar(0, 255, 0), 2, 8, 0);
			rectangle(imgCopy1, Point(250 - tem.cols * Size_factor / 2, 250 - tem.cols * Size_factor / 2), Point(250 + tem.cols * Size_factor / 2, 250 + tem.rows * Size_factor / 2), Scalar(0, 0, 255), 2, 8, 0);

			imshow("sad1", imgCopy1);
			waitKey(0);
		}
	}

	int MatchResult = std::distance(Mmax.begin(), std::max_element(Mmax.begin(), Mmax.end()));
	cout << "匹配结果" << MatchResult << endl;

	ret.x = loc[MatchResult].x;
	ret.y = loc[MatchResult].y;

	rect_pos = rectLoc[MatchResult];

	if (visualization)
	{
		Rect rect(ret.x, ret.y, 500, 500);
		cv::Mat imgCut = cv::Mat(img, rect);
		cv::Mat imgCopy = imgCut.clone();
		imshow("sad", imgCopy);
		waitKey(0);
	}
	return 0;
}