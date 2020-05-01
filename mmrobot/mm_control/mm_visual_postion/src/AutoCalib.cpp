#include "mm_visual_postion/AutoCalib.h"

template <typename T>
void cv2vpMat(cv::Mat cv_mat, vpHomogeneousMatrix& vp_mat){
    assert(cv_mat.size().width == 4 && cv_mat.size().height == 4);
    for(unsigned int i=0; i<4;i++){
        for(unsigned int j=0; j<4;j++){
            vp_mat[i][j] = cv_mat.at<T>(i,j);
        }
    }
}
template <typename T>
void vp2cvMat(vpHomogeneousMatrix vp_mat, cv::Mat& cv_mat){
    assert(cv_mat.size().width == 4 && cv_mat.size().height == 4);
    for(unsigned int i=0; i<4;i++){
        for(unsigned int j=0; j<4;j++){
            cv_mat.at<T>(i,j) = vp_mat[i][j];
        }
    }
}


bool AutoCalib::getRawImages(cv::Mat& left, cv::Mat& right, ros::Time& stamp){
    return images_receiver.getLatestRawImages(left, right, stamp);
}

bool AutoCalib::getEndeffectorPose(cv::Mat& T_endeffector_to_base, const ros::Time& stamp){
    geometry_msgs::TransformStamped transform;

    try
    {
        transform = tfBuffer.lookupTransform("tool0_controller", "base", stamp);
        transformToCvMat(transform, T_endeffector_to_base);
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
        
    }
}
void AutoCalib::manuallyMoveToNewPose(std::string text, cv::Scalar color){
    // manually move robot to position  then press any key
    int key = -1; 
    cv::Mat left_raw_image, right_raw_image;
    ros::Time stamp;
    cv::Mat image;

    while(key == -1){ //no key is pressed
        cv::namedWindow("Move to proper position", 0);
        while(ros::ok() && !getRawImages(left_raw_image, right_raw_image, stamp)){
            ros::spinOnce();
        }
        cv::hconcat(left_raw_image, right_raw_image, image);
        cv::putText(image, text, cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, color, 3);
        cv::imshow("Move to proper position", image);
        key = cv::waitKey(20);
    }
    cv::putText(image, "captured", cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 5.0, CV_RGB(0, 255, 0), 3);
    cv::imshow("Move to proper position", image);
    cv::waitKey(20);
    //cv::destroyWindow("Move to proper position");
}
void AutoCalib::collectData(std::string show_text, cv::Scalar text_color, int collect_num, bool auto_move){
    cv::Mat left_raw_image, right_raw_image;
    ros::Time stamp;
    cv::Mat T_endeffector_to_base_cv;
    left_corners_vec.clear();
    right_corners_vec.clear();
    T_endeffector_to_base_vec.clear();
    for(int i=0; i<collect_num;){

        if(auto_move) autoMoveToNewPose();
        else  manuallyMoveToNewPose(show_text,text_color);

        do{
            ros::spinOnce();
            while(!getRawImages(left_raw_image, right_raw_image, stamp) && ros::ok()){ros::spinOnce();};
            if(!ros::ok()) return;
        }
        while(!getEndeffectorPose(T_endeffector_to_base_cv, stamp));

        // process images
        std::vector<cv::Point2f> left_corners, right_corners;
        if(!findCornersPrecisely(left_raw_image, left_corners)){
            std::cout<<"cannot find corners, drop this frame"<<std::endl;
            continue;
        }
        if(!findCornersPrecisely(right_raw_image, right_corners)){
            std::cout<<"cannot find corners, drop this frame"<<std::endl;
            continue;
        }
        left_image_size = left_raw_image.size();
        right_image_size = right_raw_image.size();
        
        T_endeffector_to_base_vec.push_back(T_endeffector_to_base_cv.clone());
        left_corners_vec.push_back(left_corners);
        right_corners_vec.push_back(right_corners);
        i++;
    }
    /*
    cv::FileStorage transform_fs(save_path+"/endeffector_to_base_data.yaml", cv::FileStorage::WRITE);
    if(!transform_fs.isOpened()){
        std::cout<<"cannot write to file "<<save_path <<"/endeffector_to_base_data.yaml"<<std::endl;
    }
    //transform_fs << "{";
    for(unsigned int i=0; i<T_endeffector_to_base_vec.size(); i++){
        transform_fs << std::string("id_")+std::to_string(i) << T_endeffector_to_base_vec[i];
    }
    //transform_fs <<"}";
    */
    assert(collect_num == (int)T_endeffector_to_base_vec.size());
}

bool AutoCalib::findCornersPrecisely(const cv::Mat& image, std::vector<cv::Point2f>& corners){
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    bool found = cv::findChessboardCorners(image, board_size, corners);
    if(! found) return false;
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
    return true;
}
void AutoCalib::generateRealGrid(int n_data, std::vector<std::vector<cv::Point3f> >& obj_points_vec){
    std::vector<cv::Point3f> obj_points;
    for (int row_i = 0; row_i < board_size.height; row_i++)
    {
        for (int col_i = 0; col_i < board_size.width; col_i++)
        {
            obj_points.push_back(cv::Point3f(col_i * square_length, row_i * square_length, 0));
        }
    }
    
    obj_points_vec.resize(n_data);
    for(int i=0; i<n_data; i++)
    {
        obj_points_vec[i] = obj_points;
    }
}
void AutoCalib::calibCamera(){
    // generate real corners' coordinate
    std::vector<std::vector<cv::Point3f> > obj_points_vec;
    generateRealGrid(left_corners_vec.size(), obj_points_vec);

    // calibrate intrinsic parameters
    cv::calibrateCamera(obj_points_vec, left_corners_vec, left_image_size, model.left_camera.intrinsic_mat, model.left_camera.distortion_mat, left_r_vec, left_t_vec, 0);
    cv::calibrateCamera(obj_points_vec, right_corners_vec, right_image_size, model.right_camera.intrinsic_mat, model.right_camera.distortion_mat, right_r_vec, right_t_vec, 0);
    model.left_camera.image_size = left_image_size;
    model.right_camera.image_size = right_image_size;

    // calibrate extrinsic parameters
    cv::Mat R,T,E,F;
    stereoCalibrate(obj_points_vec, left_corners_vec, right_corners_vec, model.left_camera.intrinsic_mat, model.left_camera.distortion_mat, model.right_camera.intrinsic_mat, model.right_camera.distortion_mat,
		left_image_size, R, T, E, F, CV_CALIB_FIX_INTRINSIC, cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    Eigen::Matrix3d eigen_R;
    cv2eigen(R,eigen_R);
    model.right_cam_transform_mat.setIdentity();
    model.right_cam_transform_mat.block<3,3>(0,0) = eigen_R;
    model.right_cam_transform_mat(0,3) = T.at<double>(0);
    model.right_cam_transform_mat(1,3) = T.at<double>(1);
    model.right_cam_transform_mat(2,3) = T.at<double>(2);
    model.left_cam_transform_mat.setIdentity();
    cv2eigen(E, model.essential_mat);
    cv2eigen(F, model.fundamental_mat);

    // calculate projection matrix P=K*[R|t], where R=Id, t=0
    Eigen::Matrix3d left_intrinsic_mat, right_intrinsic_mat;
    cv2eigen(model.left_camera.intrinsic_mat, left_intrinsic_mat);
    cv2eigen(model.right_camera.intrinsic_mat, right_intrinsic_mat);
    model.left_camera.projection_mat = left_intrinsic_mat * model.left_cam_transform_mat.block<3,4>(0,0);
    model.right_camera.projection_mat = right_intrinsic_mat * model.right_cam_transform_mat.block<3,4>(0,0);

}

void AutoCalib::calibHandEye(){
    unsigned int n_data = left_corners_vec.size();

    // generate real corners' coordinate
    std::vector<std::vector<cv::Point3f> > obj_points_vec;
    generateRealGrid(1, obj_points_vec);

    left_r_vec.resize(n_data);
    left_t_vec.resize(n_data);
    for(unsigned int i=0;i<n_data;i++){
        cv::Mat left_r_mat;
        cv::Mat left_t_mat;
        cv::solvePnP(obj_points_vec[0], left_corners_vec[i], model.left_camera.intrinsic_mat, model.left_camera.distortion_mat, left_r_mat, left_t_mat);
        left_r_vec[i] = left_r_mat.clone();
        left_t_vec[i] = left_t_mat.clone();
    }
    std::vector<cv::Mat> T_left_cam_pose;
    RVecTVecToHomogenousMatrix(left_r_vec, left_t_vec, T_left_cam_pose);

    std::vector<vpHomogeneousMatrix> cam_to_obj_transform_vec(n_data);
    std::vector<vpHomogeneousMatrix> base_to_endeffector_transform_vec(n_data);
    for(unsigned int i=0; i<n_data; i++){
        cv2vpMat<double>(T_left_cam_pose[i], cam_to_obj_transform_vec[i]);
        cv2vpMat<double>(T_endeffector_to_base_vec[i].inv(), base_to_endeffector_transform_vec[i]);
    }
    vpHomogeneousMatrix endeffector_to_camera_transform; // hand (end-effector) to eye (camera) transformation 
    vpCalibration::calibrationTsai(cam_to_obj_transform_vec, base_to_endeffector_transform_vec, endeffector_to_camera_transform) ;
    cv::Mat T_endeffector_to_cam(4, 4, CV_64FC1);
    vp2cvMat<double>(endeffector_to_camera_transform, T_endeffector_to_cam);
    cv2eigen(T_endeffector_to_cam, model.endeffector_to_cam_transform);

}



bool AutoCalib::canSeeChessboardAndIsSafe(Eigen::Matrix4d& T_endeffector_to_base){
    const double safe_min_z = 200; //mm

    Eigen::Matrix4d T_cam_to_base = T_endeffector_to_cam_estimate.inverse() * T_endeffector_to_base;




    assert("Not implemented!");
    Eigen::Matrix<double,3,4> left_projection_matrix_in_base = model.left_camera.projection_mat * T_cam_to_base;
    Eigen::Matrix<double,3,4> right_projection_matrix_in_base = model.right_camera.projection_mat * T_cam_to_base;

    const double padding = 10; //in pixel
   
    for(unsigned int corner_i = 0; corner_i < 4; corner_i ++){
        Eigen::Vector4d point_in_board, point_in_cam;
        point_in_board = chess_board_corners[corner_i];
        point_in_cam = T_cam_to_base  * point_in_board;
        if(point_in_cam(2) < safe_min_z) return false;

        Eigen::Vector3d point_in_pixel_left, point_in_pixel_right;
        point_in_pixel_left = left_projection_matrix_in_base * point_in_board;
        point_in_pixel_right = right_projection_matrix_in_base * point_in_board;
        normalizeVector3d(point_in_pixel_left);
        normalizeVector3d(point_in_pixel_right);
        if( point_in_pixel_left(0) < padding || point_in_pixel_left(0) > left_image_size.width - padding
            || point_in_pixel_left(1) < padding || point_in_pixel_left(1) > left_image_size.height - padding
            || point_in_pixel_right(0) < padding || point_in_pixel_right(0) > right_image_size.width - padding
            || point_in_pixel_right(1) < padding || point_in_pixel_right(1) > right_image_size.height- padding){
            return false;
        }
    }
    return true;
}
void AutoCalib::generateMoveCommand(std::vector<double>& xyzabc_cmd, bool always_center){
    // direct camera to ground truth position.    
    // sample the  T_endeffector_to_base       
    Eigen::Vector3d origin_T_translate;

    Eigen::Matrix4d origin_T_base_to_endeffector = origin_T_endeffector_to_base.inverse(); //origin_T_base_to_endeffector is the same data from the control panel.
    origin_T_translate = origin_T_base_to_endeffector.block<3,1>(0,3);
    Eigen::Quaternion<double> origin_q(origin_T_base_to_endeffector.block<3,3>(0,0));
    
    // sample T_cam_to_chessboard
    Eigen::Matrix4d sample_T_cam_to_chessboard;
    const double x_var = 300; //mm
    const double y_var = 300;
    const double z_var = 400;
    const double q_var = 0.5;
    // unifrom sample //http://mathworld.wolfram.com/SpherePointPicking.html
    std::uniform_real_distribution<double> dist_x(x_var , x_var);
    std::uniform_real_distribution<double> dist_y(y_var , y_var);
    std::uniform_real_distribution<double> dist_z(z_var , z_var);
    std::uniform_real_distribution<double> dist_q_x(- q_var, q_var);
    std::uniform_real_distribution<double> dist_q_y( - q_var, q_var);
    std::uniform_real_distribution<double> dist_q_z( - q_var, q_var);
    std::uniform_real_distribution<double> dist_q_w( - q_var, q_var);
    
    
    // reject if (q_x^2 + q_y^2 + q_z^2 + q_w^2 >1)
    while(true){
        double xyz_q[7];
        xyz_q[0] = always_center ? 0 :dist_x(gen);
        xyz_q[1] = always_center ? 0 :dist_y(gen);
        xyz_q[2] = always_center ? 0 :dist_z(gen);
        xyz_q[3] = dist_q_x(gen);
        xyz_q[4] = dist_q_y(gen);
        xyz_q[5] = dist_q_z(gen);
        xyz_q[6] = dist_q_w(gen);

        if(xyz_q[3]*xyz_q[3] + xyz_q[4]*xyz_q[4] + xyz_q[5]*xyz_q[5] + xyz_q[6]*xyz_q[6] > 1){
            continue;
        }


        // check whether the sample is valid
        Eigen::Matrix4d sample_T_endeffector_to_base, sample_T_base_to_endeffector;
        Eigen::Quaternion<double> sample_q( xyz_q[6], xyz_q[3], xyz_q[4], xyz_q[5]); // w,x,y,z
        sample_q.normalize();
        sample_T_cam_to_chessboard.setIdentity();
        sample_T_cam_to_chessboard.block<3,3>(0,0) = sample_q.toRotationMatrix();
       
        Eigen::Vector3d translate = sample_T_cam_to_chessboard.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();
        sample_T_cam_to_chessboard(0,3) = xyz_q[0] - translate(0);
        sample_T_cam_to_chessboard(1,3) = xyz_q[1] - translate(1);
        sample_T_cam_to_chessboard(2,3) = xyz_q[2] - translate(2);

        sample_T_endeffector_to_base = sample_T_base_to_endeffector.inverse();
        
        if(canSeeChessboardAndIsSafe(sample_T_endeffector_to_base)){
            translateTbase2endeffectorToAngleAxisCmd(sample_T_base_to_endeffector, xyzabc_cmd);
            break;
        }
    }
}
void AutoCalib::autoMoveToNewPose(){
    std::vector<double> xyzabc_cmd;
    generateMoveCommand(xyzabc_cmd);

    arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                            xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
    
    while(ros::ok()){
        bool time_out = arm_commander.isTimeOut();
        if(time_out){
            generateMoveCommand(xyzabc_cmd);
            arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                            xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
            std::cout<<"Timeout, generate and move to new target position for end effector..."<<std::endl;
        } 
        if(arm_commander.isArrived() || time_out){
            // arm arrives the cmd position
            // capture the image
            sleep(1); // wait for camera to stablize
            break;
        }
        ros::spinOnce();
    }
    return;
}





int main(int argc, char** argv){
    if(argc <7){
        std::cout<<"Please input necessary parameters: \n"
                 <<"\t (int) number of images to be used for camera calibration"
                 <<"\t (int) number of images to be used for eye hand calibration"
                 <<"\t (float) length of the squares on the chessboard (mm)"
                 <<"\t (int) number of cols on the chessboard"
                 <<"\t (int) number of rows on the chessboard"
                 <<"\t save path, if the calibration file is already existed, this program will run in automatic mode"
        <<std::endl;
        return 0;
    }
    // AutoCalib 20 10 15 12 9 ~/mrobot/src/mmrobot/mm_control/mm_visual_postion/stereo_cam_arm_model.yaml
    ros::init(argc, argv, "auto_calib_node");
	ros::NodeHandle nh;
    AutoCalib auto_calib(std::atoi(argv[1]), std::atoi(argv[2]), std::atof(argv[3]), std::atoi(argv[4]), std::atoi(argv[5]), argv[6], nh);
    return 0;
}