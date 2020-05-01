#include <pluginlib/class_list_macros.h>

// Include your header
#include <camera_rectification.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(mm_zed_camera_rectification::CameraRectificationNodelet, nodelet::Nodelet)

namespace mm_zed_camera_rectification{
void CameraRectification::init()
{
	zed_wrapper.init();
    rect_image_pub = nh_.advertise< sensor_msgs::Image >("/camera/image_rect", 1);

    //相机内外参初始化
	cv::Size image_size = cv::Size(2208, 1242);
	//左相机
	double tem_leftIntrinsic[3][3] = { 1404.6805178647157, 0, 1093.7708076703914, 0, 1404.9562302837180, 650.19805634924830, 0, 0, 1 };

	double tem_leftDistortion[1][5] = { -0.15884502830994698, -0.017019860223729141, -0.00050845821974763127, -0.00049227193638006373, 0.042091851482026002 };

	//世界坐标系定在了左相机的相机坐标系
	double tem_leftRotation[3][3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

	double tem_leftTranslation[1][3] = { 0, 0, 0 };

	//右相机
	double tem_rightIntrinsic[3][3] = { 1395.0427725909017, 0, 1065.3453232201614, 0, 1393.9191585585920, 681.76116308075120, 0, 0, 1 };

	double tem_rightDistortion[1][5] = { -0.15815821923496060, -0.023252410765977595, -0.00088122276582216434, -0.00048619722968107478, 0.047581138754438562 };

	//两眼之间的关系
	double tem_rightRotation[3][3] = { 0.99985594561053415, 0.0022064006648113573, 0.016829136144518399, -0.0022230836705065750, 0.99999705589818755, 0.00097267360974158759,
		-0.016826940490128992, -0.0010099460695902113, 0.99985790694612109 };

	double tem_rightTranslation[1][3] = { -120.02236830747624, 0.66187592661751726, -1.5840147731881373 };

	//固定填充矩阵
	double tem_KFillmatrix[1][3] = { 0, 0, 0 };

	double tem_MFillmatrix[1][4] = { 0, 0, 0, 1 };

	cv::Mat mRightIntrinsic = cv::Mat(3, 3, CV_64F, tem_rightIntrinsic); //see https://stackoverflow.com/questions/44511057/fail-to-assign-a-cvmat-object-in-c-class
	cv::Mat mRightDistortion = cv::Mat(1, 5, CV_64F, tem_rightDistortion);
	cv::Mat mLeftIntrinsic = cv::Mat(3, 3, CV_64F, tem_leftIntrinsic);
	cv::Mat mLeftDistortion = cv::Mat(1, 5, CV_64F, tem_leftDistortion);
	initUndistortRectifyMap(mLeftIntrinsic, mLeftDistortion, cv::Mat(), mLeftIntrinsic, image_size, CV_32FC1, leftMap1, leftMap2);
	initUndistortRectifyMap(mRightIntrinsic, mRightDistortion, cv::Mat(), mRightIntrinsic, image_size, CV_32FC1, rightMap1, rightMap2);

}
void CameraRectification::captureRawImage(){
	zed_wrapper.grab(raw_image);
}

void CameraRectification::rectifyImage(){
    try
	{
		// Extract left and right images from side-by-side
        cv::Mat latest_left_image = raw_image(cv::Rect(0, 0, raw_image.cols / 2, raw_image.rows));
        cv::Mat latest_right_image = raw_image(cv::Rect(raw_image.cols / 2, 0, raw_image.cols / 2, raw_image.rows));
        cv::Mat rect_left_image, rect_right_image;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        cv::remap(latest_left_image, rect_left_image, leftMap1, leftMap2, cv::INTER_LINEAR);
    	cv::remap(latest_right_image, rect_right_image, rightMap1, rightMap2, cv::INTER_LINEAR);
		hconcat(rect_left_image, rect_right_image, cv_ptr->image);
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        rect_image_pub.publish(cv_ptr->toImageMsg());
    }
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert to image!");
	}
}
void CameraRectificationNodelet::onInit(){
    ros::NodeHandle nh = this->getPrivateNodeHandle();
    camera_rectification_ptr = std::make_shared<CameraRectification>(nh);
	camera_rectification_ptr->init();
	main();
}

void CameraRectificationNodelet::main()
  {
	ros::Rate r(15);
	while(ros::ok()){
		camera_rectification_ptr->captureRawImage();
		camera_rectification_ptr->rectifyImage();
		r.sleep();
		ros::spinOnce();
	}
  }
}