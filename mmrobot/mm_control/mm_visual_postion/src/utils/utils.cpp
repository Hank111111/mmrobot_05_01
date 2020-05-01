#include "mm_visual_postion/utils/utils.h"


std::ostream &operator << (std::ostream &f, const Eigen::Vector4d &vec){
    f << vec(0) <<","<<vec(1)<<","<<vec(2)<<","<<vec(3);
    return f;
}

void normalizeVector3d(Eigen::Vector3d& vec){
    vec(0) = vec(0) / vec(2);
    vec(1) = vec(1) / vec(2);
    vec(2) = 1.0;
}

bool isQuaternionSimilar(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2, const double epsilon){
    // see https://gamedev.stackexchange.com/questions/75072/how-can-i-compare-two-quaternions-for-logical-equality 
    if(fabs(q1.dot(q2)) > 1-epsilon)
        return true;
    else 
        return false;
}

void transformToMatrix(const geometry_msgs::TransformStamped& transform, Eigen::Matrix4d& T){
    transformToMatrix(transform.transform, T);
} 
void transformToMatrix(const geometry_msgs::Transform& transform, Eigen::Matrix4d& T){
    // translate from geometry_msgs::TransformStamped to eigen matrix
    Eigen::Quaternion<double> q;
    Eigen::fromMsg(transform.rotation, q);
    q.normalize();
    T.setIdentity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = transform.translation.x * 1000.0; //millimiter
    T(1,3) = transform.translation.y * 1000.0;
    T(2,3) = transform.translation.z * 1000.0;
    T(3,3) = 1.0;
} 
void transformToCvMat(geometry_msgs::TransformStamped& transform, cv::Mat& T){
    transformToCvMat(transform.transform, T);
}
void transformToCvMat(geometry_msgs::Transform& transform, cv::Mat& T){
    // translate from geometry_msgs::TransformStamped to opencv matrix
    Eigen::Matrix4d eigen_T;
    transformToMatrix(transform, eigen_T);
    std::cout<<"transform"<<eigen_T<<std::endl;
    eigen2cv(eigen_T, T);
}
void translateTbase2endeffectorToAngleAxisCmd(Eigen::Matrix4d& T_base_to_endeffector, std::vector<double>& XYZRxRyRz){
    // T_base_to_endeffector in milimeter
    //XYZRxRyRz is in mm, rad
    XYZRxRyRz.resize(6);
    Eigen::AngleAxisd angle_axis(T_base_to_endeffector.block<3,3>(0,0));
    Eigen::Vector3d rx_ry_rz = angle_axis.axis() * angle_axis.angle();
    XYZRxRyRz[0] = T_base_to_endeffector(0,3);
    XYZRxRyRz[1] = T_base_to_endeffector(1,3);
    XYZRxRyRz[2] = T_base_to_endeffector(2,3);
    XYZRxRyRz[3] = rx_ry_rz(0);
    XYZRxRyRz[4] = rx_ry_rz(1);
    XYZRxRyRz[5] = rx_ry_rz(2);    
}



void averageQuaternion(const std::vector<Eigen::Quaterniond>& quat_vec, Eigen::Quaterniond& averaged_quat){
    // use the algorithm introduced in https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    // see also http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
    // this function assumes the quaternions in quat_vec are normalized.
    Eigen::MatrixXd Q;
    Q.resize(4, quat_vec.size());
    for(unsigned int i=0; i< quat_vec.size(); i++){
        Q(0,i) = quat_vec[i].x();
        Q(1,i) = quat_vec[i].y();
        Q(2,i) = quat_vec[i].z();
        Q(3,i) = quat_vec[i].w();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(4);
    eigen_solver.compute(Q * Q.transpose());// The eigenvalues are sorted in increasing order.
    Eigen::VectorXd average_q_value = eigen_solver.eigenvectors().col(3); // eigen vector corresponds to the largest eigen value
    averaged_quat.x() = average_q_value(0);
    averaged_quat.y() = average_q_value(1);
    averaged_quat.z() = average_q_value(2);
    averaged_quat.w() = average_q_value(3);    
}
void averageTransform(const std::vector<Eigen::Matrix4d>& T_vec, Eigen::Matrix4d& avg_T){
    std::vector<Eigen::Quaterniond> q_vec;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    for(unsigned int i=0; i<T_vec.size(); i++){
        Eigen::Quaterniond q(T_vec[i].block<3,3>(0,0));
        q_vec.push_back(q);
        translation += T_vec[i].block<3,1>(0,3);
    }
    translation = translation / T_vec.size();
    avg_T.setIdentity();
    Eigen::Quaterniond avg_q;
    averageQuaternion(q_vec, avg_q);
    avg_T.block<3,3>(0,0) = Eigen::Matrix3d(avg_q);
    avg_T.block<3,1>(0,3) = translation;
}

bool isRectCompletelyInImage(const cv::Rect& rect, const cv::Size img_size){
    return (rect.x >= 1e-5 &&  img_size.width - 1.0 - (rect.x + rect.width) >=1e-5
        && rect.y >=1e-5 && img_size.height - 1.0 - (rect.y + rect.height) >=1e-5);
}


void ROIMsgToCv(const mm_visual_postion::ROI& msg, cv::Rect2f& cv_roi){
    cv_roi.x = msg.x;
    cv_roi.y = msg.y;
    cv_roi.width = msg.width;
    cv_roi.height = msg.height;
}

void ROICvToMsg(const cv::Rect2f& cv_roi,  mm_visual_postion::ROI& msg){
    msg.x = cv_roi.x;
    msg.y = cv_roi.y;
    msg.width = cv_roi.width;
    msg.height = cv_roi.height;
}


bool isTransfromSimilaireToIdentity(Eigen::Matrix4d & T, const double pos_tolerance, const double q_epsilon){
    if(fabs(T(0,3)) > pos_tolerance || fabs(T(1,3)) > pos_tolerance || fabs(T(2,3)) > pos_tolerance)
        return false;
    Eigen::Matrix3d rotation = T.block<3,3>(0,0);
    Eigen::Quaterniond q(rotation);
    Eigen::Quaterniond q_Id(1,0,0,0);
    if(! isQuaternionSimilar(q, q_Id, q_epsilon)) 
        return false;

    return true;
}

void transformToPose4WithQuaternion(Eigen::Matrix4d& T, mm_robot_decision::pose4& pose){
    Eigen::Quaterniond q(T.block<3,3>(0,0));
    q.normalize();
    pose.x = T(0,3) / 1000.0;
    pose.y = T(1,3) / 1000.0;
    pose.z = T(2,3) / 1000.0;
    pose.a = q.x();
    pose.b = q.y();
    pose.c = q.z();
    pose.w = q.w();
}

void msgToT_frame_to_obj(const mm_robot_decision::VisualAppResponse& res, Eigen::Matrix4d& T_frame_to_obj, std::string expected_presented_frame){
    assert(res.frame_id == expected_presented_frame);
    T_frame_to_obj.setIdentity();
    Eigen::Quaterniond q;
    q.x() = res.pose.a;
    q.y() = res.pose.b;
    q.z() = res.pose.c;
    q.w() = res.pose.w;
    
    T_frame_to_obj.block<3,3>(0,0) = Eigen::Matrix3d(q);
    T_frame_to_obj(0,3) = res.pose.x * 1000.0;
    T_frame_to_obj(1,3) = res.pose.y * 1000.0;
    T_frame_to_obj(2,3) = res.pose.z * 1000.0;
}


void RVecTVecToHomogenousMatrix(const cv::Mat& r_vec, const cv::Mat& t_vec, cv::Mat& trans){
    double array_to_fill[1][4] = { 0, 0, 0, 1 };
    cv::Mat last_row = cv::Mat(1, 4, CV_64F, array_to_fill);
    cv::Mat R(3, 3, CV_64FC1);
    cv::Rodrigues(r_vec, R);
    cv::Mat except_last_row = cv::Mat(3, 4, CV_64F);
    hconcat(R, t_vec, except_last_row);
    
    trans.create(4, 4, CV_64F);
    vconcat(except_last_row, last_row, trans);
}
void RVecTVecToHomogenousMatrix(const std::vector<cv::Mat>& r_vec_vec, const std::vector<cv::Mat>& t_vec_vec, std::vector<cv::Mat>& trans_vec){
    assert(r_vec_vec.size() == t_vec_vec.size());
    trans_vec.resize(t_vec_vec.size());
    for(unsigned int i=0; i< r_vec_vec.size(); i++){
        RVecTVecToHomogenousMatrix(r_vec_vec[i], t_vec_vec[i], trans_vec[i]);
    }
}
void getPointsFromRect(cv::Rect& rect, std::vector<cv::Point2d>& pts){
  cv::Point2d point;
  point.x = rect.x ;
  point.y = rect.y ;
  pts.push_back(point);
  point.x = rect.x + rect.width;
  point.y = rect.y ;
  pts.push_back(point);
  point.x = rect.x + rect.width;
  point.y = rect.y + rect.height;
  pts.push_back(point);
  point.x = rect.x ;
  point.y = rect.y + rect.height;
  pts.push_back(point);
}

void expandROI(cv::Rect& rect, double pattern, cv::Size image_size){
    rect.x -= pattern;
    rect.y -= pattern;
    rect.width += 2* pattern;
    rect.height += 2*pattern;
    if(rect.x < 0) rect.x = 0;
    if(rect.y < 0) rect.y = 0;
    if(rect.width > image_size.width) rect.width = image_size.width;
    if(rect.height > image_size.height) rect.height = image_size.height;
}

void convertMsgInBaseToMsgInEndeffector(const mm_robot_decision::VisualAppResponse& res_in_base, 
                                        const Eigen::Matrix4d& T_endeffector_to_base, 
                                        mm_robot_decision::VisualAppResponse& res_in_endeffector){
    assert(res_in_base.frame_id == "base");
    Eigen::Matrix4d T_base_to_obj, T_endeffector_to_obj;
    msgToT_frame_to_obj(res_in_base, T_base_to_obj, "base");
    T_endeffector_to_obj = T_endeffector_to_base * T_base_to_obj;
    res_in_endeffector = res_in_base; // copy all the things into res_in_endeffector, only change the frame
    Eigen::Quaterniond q(T_endeffector_to_obj.block<3,3>(0,0));
    res_in_endeffector.pose.a = q.x();
    res_in_endeffector.pose.b = q.y();
    res_in_endeffector.pose.c = q.z();
    res_in_endeffector.pose.w = q.w();

    res_in_endeffector.pose.x = T_endeffector_to_obj(0,3)/1000.0;
    res_in_endeffector.pose.y = T_endeffector_to_obj(1,3)/1000.0;
    res_in_endeffector.pose.z = T_endeffector_to_obj(2,3)/1000.0;
    res_in_endeffector.frame_id = ENDEFFECTOR_FRAME_NAME;
    

}