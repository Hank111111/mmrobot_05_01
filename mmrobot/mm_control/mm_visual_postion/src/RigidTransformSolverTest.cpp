#include "mm_visual_postion/utils/RigidTransfomSolver.h"
#include <opencv2/opencv.hpp>

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

	//assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2));

	//bool singular = sy < 1e-6; // If

	float x, y, z;
	/*if (!singular)
	{*/
	x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
	y = atan2(-R.at<double>(2, 0), sy);
	z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	//}
	//else
	//{
	//	x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
	//	y = atan2(-R.at<double>(2, 0), sy);
	//	z = 0;
	//}
	return cv::Vec3f(x, y, z);
}
void computeSwitchPose(const std::vector<cv::Point3d> p, std::vector<double> &xyzabc, double &length){

	xyzabc.resize(6);

	xyzabc[0] = (p[0].x + p[1].x + p[2].x + p[3].x) / 4.0;
	xyzabc[1] = (p[0].y + p[1].y + p[2].y + p[3].y) / 4.0;
	xyzabc[2] = (p[0].z + p[1].z + p[2].z + p[3].z) / 4.0;

	length = sqrt((p[1].x - p[2].x) * (p[1].x - p[2].x) + (p[1].y - p[2].y) * (p[1].y - p[2].y) + (p[1].z - p[2].z) * (p[1].z - p[2].z));
	//求abc角度
	//求xy轴单位向量
	double v_x[3];
	double v_y[3];
	v_x[0] = p[1].x - p[0].x;
	v_x[1] = p[1].y - p[0].y;
	v_x[2] = p[1].z - p[0].z;
	v_y[0] = p[2].x - p[1].x;
	v_y[1] = p[2].y - p[1].y;
	v_y[2] = p[2].z - p[1].z;
	double Mvx = sqrt(v_x[0] * v_x[0] + v_x[1] * v_x[1] + v_x[2] * v_x[2]);
	double Mvy = sqrt(v_y[0] * v_y[0] + v_y[1] * v_y[1] + v_y[2] * v_y[2]);
	double u_x[3];
	double u_y[3];
	u_x[0] = v_x[0] / Mvx;
	u_x[1] = v_x[1] / Mvx;
	u_x[2] = v_x[2] / Mvx;
	u_y[0] = v_y[0] / Mvy;
	u_y[1] = v_y[1] / Mvy;
	u_y[2] = v_y[2] / Mvy;
	double v_z[3];
	v_z[0] = u_x[1] * u_y[2] - u_x[2] * u_y[1];
	v_z[1] = u_x[2] * u_y[0] - u_x[0] * u_y[2];
	v_z[2] = u_x[0] * u_y[1] - u_x[1] * u_y[0];
	double u_z[3];
	double Mvz = sqrt(v_z[0] * v_z[0] + v_z[1] * v_z[1] + v_z[2] * v_z[2]);
	u_z[0] = v_z[0] / Mvz;
	u_z[1] = v_z[1] / Mvz;
	u_z[2] = v_z[2] / Mvz;
	//cout << u_x[0] << endl << u_x[1] << endl << u_x[2] << endl;
	//cout << u_y[0] << endl << u_y[1] << endl << u_y[2] << endl;
	//double xy = u_x[0] * u_y[0] + u_x[1] * u_y[1] + u_x[2] * u_y[2];
	//cout << xy << endl;
	//cout << Mvz << endl;
	cv::Mat rotate = cv::Mat(3, 3, CV_64F);
	rotate.at<double>(0, 0) = u_x[0];
	rotate.at<double>(1, 0) = u_x[1];
	rotate.at<double>(2, 0) = u_x[2];
	rotate.at<double>(0, 1) = u_y[0];
	rotate.at<double>(1, 1) = u_y[1];
	rotate.at<double>(2, 1) = u_y[2];
	rotate.at<double>(0, 2) = u_z[0];
	rotate.at<double>(1, 2) = u_z[1];
	rotate.at<double>(2, 2) = u_z[2];
	//cout << rotate << endl;
	cv::Vec3f abc = rotationMatrixToEulerAngles(rotate);
	xyzabc[3] = abc[0];
	xyzabc[4] = abc[1];
	xyzabc[5] = abc[2];
}

Eigen::Quaterniond addTwoQuat(Eigen::Quaterniond q1, Eigen::Quaterniond q2){
	Eigen::Quaterniond result_q;
	result_q.x() = q1.x() + q2.x();
	result_q.y() = q1.y() + q2.y();
	result_q.z() = q1.z() + q2.z();
	result_q.w() = q1.w() + q2.w();
	return result_q;
}

Eigen::Quaterniond subTwoQuat(Eigen::Quaterniond q1, Eigen::Quaterniond q2){
	Eigen::Quaterniond result_q;
	result_q.x() = q1.x() - q2.x();
	result_q.y() = q1.y() - q2.y();
	result_q.z() = q1.z() - q2.z();
	result_q.w() = q1.w() - q2.w();
	return result_q;
}

double distTwoQuat(Eigen::Quaterniond q1, Eigen::Quaterniond q2){
	//note: this function only takes accout the numerical similarity of q1 and q2
	Eigen::Quaterniond q1_minus_q2 = subTwoQuat(q1, q2);
	Eigen::Quaterniond q1_plus_q2 = addTwoQuat(q1, q2);
	double q1_minus_q2_norm = q1_minus_q2.norm();
	double q1_plus_q2_norm = q1_plus_q2.norm();
	return std::min(q1_minus_q2_norm, q1_plus_q2_norm);
}


Eigen::Vector3d rotate(Eigen::Vector3d& vec, Eigen::Matrix4d& rot){
	Eigen::Vector4d vec_homo;
	vec_homo.block<3,1>(0,0) = vec;
	vec_homo = rot * vec_homo.eval();
	return vec_homo.block<3,1>(0,0);
}

int main(int argc, char ** argv){
    std::vector<Eigen::Vector3d> set_points(4);
	std::vector<double> data = {10,  10, 0,
								-10, 10, 0,
								-10, -10, 0,
								10, -10, 0};

	Eigen::Quaterniond rot_quat = Eigen::AngleAxisd(0.1*M_PI, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.3*M_PI , Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitZ());
	std::cout<<"gt: q inv: "<<rot_quat.inverse().coeffs()<<std::endl;
	std::cout<<"gt: q: "<<rot_quat.coeffs()<<std::endl;

	Eigen::Matrix3d rot(rot_quat);
    set_points[0] = Eigen::Vector3d(data[0],data[1],data[2]);
    set_points[1] = Eigen::Vector3d(data[3],data[4],data[5]);
    set_points[2] = Eigen::Vector3d(data[6],data[7],data[8]);
    set_points[3] = Eigen::Vector3d(data[9],data[10],data[11]);
	set_points[0] = rot * set_points[0].eval();;
	set_points[1] = rot * set_points[1].eval();;
	set_points[2] = rot * set_points[2].eval();;
	set_points[3] = rot * set_points[3].eval();;

    RigidRectTransformSolver solver(50, 50); // mm
    solver.setPointSets(set_points);
    
    std::vector<double> xyzabc_test;
    double length_test;
    solver.solveTransform(xyzabc_test, length_test);
    





	// original method
    std::vector<cv::Point3d> cv_points(4);
    cv_points[0] = cv::Point3d(set_points[0](0),set_points[0](1),set_points[0](2));
    cv_points[1] = cv::Point3d(set_points[1](0),set_points[1](1),set_points[1](2));
    cv_points[2] = cv::Point3d(set_points[2](0),set_points[2](1),set_points[2](2));
    cv_points[3] = cv::Point3d(set_points[3](0),set_points[3](1),set_points[3](2));
    
    std::vector<double> xyzabc;
    double l;
    computeSwitchPose(cv_points, xyzabc, l);
    Eigen::Quaterniond q_test;
    q_test = Eigen::AngleAxisd(xyzabc_test[3], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(xyzabc_test[4], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(xyzabc_test[5], Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q_real;
    q_real = Eigen::AngleAxisd(xyzabc[3], Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(xyzabc[4], Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(xyzabc[5], Eigen::Vector3d::UnitZ());

	
    std::cout<<"xyzabc_test"<<xyzabc_test[0] << ", "<< xyzabc_test[1] << ", "<<  xyzabc_test[2] << ", " << xyzabc_test[3] << ", "<< xyzabc_test[4] << ", "<<  xyzabc_test[5]<<", q\n"<< q_test.coeffs()<<std::endl <<std::endl;
    std::cout<<"xyzabc     "<<xyzabc[0] << ", "<< xyzabc[1] << ", "<<  xyzabc[2] << ", " << xyzabc[3] << ", "<< xyzabc[4] << ", "<<  xyzabc[5] <<  ", q\n"<<q_real.coeffs() <<std::endl <<std::endl;
	std::cout<<"distance q: "<<distTwoQuat(q_test, q_real)<<std::endl;
    std::cout<<"length_test "<< length_test <<std::endl;
    std::cout <<"length     "<<l<<std::endl;


    return 0;
}