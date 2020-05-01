#include "mm_visual_postion/hand_trunck/PreciseCircleSolver.h"



void PointToEllipseDistance::Func_0::forward(const EigenVector8d& param, Eigen::Matrix4d& transform){
    q_x = param(3);
    q_y = param(4);
    q_z = param(5);
    q_w = param(6);

    transform << -2*q_y*q_y - 2*q_z*q_z + 1, -2*q_w*q_z + 2*q_x*q_y, 2*q_w*q_y + 2*q_x*q_z, param(0),
                    2*q_w*q_z + 2*q_x*q_y, -2*q_x*q_x - 2*q_z*q_z + 1, -2*q_w*q_x + 2*q_y*q_z, param(1),
                    -2*q_w*q_y + 2*q_x*q_z, 2*q_w*q_x + 2*q_y*q_z, -2*q_x*q_x - 2*q_y*q_y + 1, param(2),
                    0.0, 0.0, 0.0, 1.0;
}
void PointToEllipseDistance::Func_0::backward(const Eigen::Matrix4d& deriv_transform, EigenVector8d& deriv_param){
    deriv_param(0) = deriv_transform(0,3); // x
    deriv_param(1) = deriv_transform(1,3); // y
    deriv_param(2) = deriv_transform(2,3); // z
    
    Eigen::Matrix4d d_rot_dq_x, d_rot_dq_y, d_rot_dq_z, d_rot_dq_w;
    d_rot_dq_x <<   0 , 2*q_y , 2*q_z , 0,
                    2*q_y , -4*q_x , -2*q_w , 0,
                    2*q_z , 2*q_w , -4*q_x , 0 ,
                    0 , 0 , 0 , 0 ;
    d_rot_dq_y <<   -4*q_y , 2*q_x , 2*q_w , 0,
                    2*q_x , 0 , 2*q_z , 0 ,
                    -2*q_w , 2*q_z , -4*q_y , 0 ,
                    0 , 0 , 0 , 0;
    d_rot_dq_z <<   -4*q_z , -2*q_w , 2*q_x , 0,
                    2*q_w , -4*q_z , 2*q_y , 0,
                    2*q_x , 2*q_y , 0 , 0,
                    0 , 0 , 0 , 0;
    d_rot_dq_w <<   0 , -2*q_z , 2*q_y , 0,
                    2*q_z , 0 , -2*q_x , 0,
                    -2*q_y , 2*q_x , 0 , 0,
                    0 , 0 , 0 , 0;
    deriv_param(3) = (d_rot_dq_x.cwiseProduct(deriv_transform)).sum(); //q_x
    deriv_param(4) = (d_rot_dq_y.cwiseProduct(deriv_transform)).sum(); //q_y
    deriv_param(5) = (d_rot_dq_z.cwiseProduct(deriv_transform)).sum(); //q_z
    deriv_param(6) = (d_rot_dq_w.cwiseProduct(deriv_transform)).sum(); //q_w

}


void PointToEllipseDistance::Func_1::forward(const Eigen::Matrix<double,3,4>* A_ptr_input, Eigen::Matrix4d& X, Eigen::Matrix<double, 3,4>& Y){
    A_ptr = A_ptr_input;
    Y = (*A_ptr) * X;
}
void PointToEllipseDistance::Func_1::backward(const Eigen::Matrix<double, 3,4>& deriv_Y, Eigen::Matrix4d& deriv_X){
    deriv_X = A_ptr->transpose() * deriv_Y;
}

void PointToEllipseDistance::Func_2::forward(const Eigen::Matrix<double, 4, 3>* B_ptr_input, Eigen::Matrix<double,3,4>& X, Eigen::Matrix<double,3,3>& Y){
    B_ptr = B_ptr_input;
    Y = X * (*B_ptr_input);
}
void PointToEllipseDistance::Func_2::backward(const Eigen::Matrix<double,3,3>& deriv_Y, Eigen::Matrix<double, 3, 4>& deriv_X){
    deriv_X = deriv_Y * B_ptr->transpose();
}


void PointToEllipseDistance::Func_3::forward(const Eigen::Matrix3d& X, Eigen::Matrix3d& Y){
    Y = X.inverse();
    Y_local = Y;
}
void PointToEllipseDistance::Func_3::backward(const Eigen::Matrix3d& deriv_Y, Eigen::Matrix3d& deriv_X){
    for(unsigned int i=0; i<3; i++){
        for(unsigned int j=0; j<3; j++){
            Eigen::Matrix3d tmp = Eigen::Matrix3d::Zero();
            tmp(i,j) = 1.0;
            Eigen::Matrix3d dY_dx_ij = -Y_local * tmp * Y_local;
            Eigen::Matrix3d deriv_mat = deriv_Y.cwiseProduct(dY_dx_ij);
            deriv_X(i,j) = deriv_mat.sum();
        }
    }
}
void PointToEllipseDistance::Func_3::testDerivate(const Eigen::Matrix3d& X){
    double step = 0.001;
    Eigen::Matrix3d Y_0, Y_1;

    Eigen::Matrix3d deriv_Y;
    deriv_Y << 1.0, 0.4, 0.1, 0.3, 0.4, 0.9, 1.2, 0.7, 0.2;
    
    Eigen::Matrix3d X_0, X_1;
    X_0 = X; X_1 = X;
    X_0(0,0) =  step;
    X_1(0,0) = - step;
    forward(X_0, Y_0);
    forward(X_1, Y_1);
    
    Eigen::Matrix3d deriv_X_00 = (Y_0 - Y_1).cwiseProduct(deriv_Y) / step / 2.0;
    Eigen::Matrix3d deriv_X;
    backward(deriv_Y, deriv_X);
    std::cout<<"func_6: analytical backward: x "<< deriv_X(0, 0) <<std::endl;
    
    std::cout<<"func_6: numerical  backward: x "<< deriv_X_00(0,0) <<std::endl;
}


void PointToEllipseDistance::Func_4::forward(const Eigen::Vector3d& V_input, const Eigen::Matrix3d X, Eigen::Vector3d& Y){
    V = V_input;
    Y = X * V;
}
void PointToEllipseDistance::Func_4::backward(const Eigen::Vector3d& deriv_Y, Eigen::Matrix3d& deriv_X){
    deriv_X = deriv_Y * V.transpose();

}




void PointToEllipseDistance::Func_5::forward(const Eigen::Vector3d& X, Eigen::Vector3d& X_normalized){
    u = X(0);
    v = X(1);
    w = X(2);
    X_normalized(0) = X(0)/X(2);
    X_normalized(1) = X(1)/X(2);
    X_normalized(2) = 1.0;
}
void PointToEllipseDistance::Func_5::backward(const Eigen::Vector3d& deriv_X_normalized, Eigen::Vector3d& deriv_X){
    deriv_X(0) = deriv_X_normalized(0) / w;
    deriv_X(1) = deriv_X_normalized(1) / w;
    deriv_X(2) = deriv_X_normalized(0) * (-u/w/w) + deriv_X_normalized(1) * (-v/w/w);
}
void PointToEllipseDistance::Func_5::testDerivate(const Eigen::Vector3d& X){
    double step = 0.001;
    //double res_0, res_1;

    Eigen::Vector3d deriv_X_normalized;
    deriv_X_normalized << 1.0, 0.4, 0.1;
    
    Eigen::Vector3d X_0, X_1;
    X_0 << X(0) + step, X(1), X(2);
    X_1 << X(0) - step, X(1), X(2);
    Eigen::Vector3d X_0_normalized, X_1_normalized;
    forward(X_0, X_0_normalized);
    forward(X_1, X_1_normalized);
    

    Eigen::Vector3d deriv_X;
    backward(deriv_X_normalized, deriv_X);
    std::cout<<"func_6: analytical backward: x "<< deriv_X(0) <<std::endl;
    
    std::cout<<"func_6: numerical  backward: x "<< (X_0_normalized - X_1_normalized).transpose() * deriv_X_normalized / step / 2.0 <<std::endl;
}



void PointToEllipseDistance::Func_6::forward(double x_input, double y_input, double r_input, double& res){
    x = x_input;
    y = y_input;
    r = r_input;
    rr = r*r;
    res = x*x  + y*y - rr;
}
void PointToEllipseDistance::Func_6::backward(double deriv_res, double& deriv_x, double& deriv_y, double& deriv_r){

    deriv_x = deriv_res * 2*x;
    deriv_y = deriv_res * 2*y;
    deriv_r = -deriv_res * 2*r;
}
void PointToEllipseDistance::Func_6::testDerivate(double x_input, double y_input, double r_input){
    double step = 0.001;
    double res_0, res_1;

    double der_x = 0;
    double der_y = 0;
    double der_r = 0;
    backward(1.0, der_x, der_y, der_r);
    std::cout<<"func_6: analytical backward: x "<< der_x <<std::endl;
    
    forward(x_input + step, y_input, r_input, res_0);
    forward(x_input - step, y_input, r_input, res_1);
    std::cout<<"func_6: numerical  backward: x "<< (res_0 - res_1) / step / 2.0 <<std::endl;
}


bool PointToEllipseDistance::Evaluate(double const* const* x,
                            double* residuals,
                            double** jacobians) const {
    EigenVector8d param;
    param << x[1][0], x[1][1], x[1][2], x[0][0], x[0][1],x[0][2], x[0][3], x[1][3];
    // x: first row: x,y,z,radius
    //    second row: q_x, q_y, q_z, q_w;

    Eigen::Matrix4d transform;
    Func_0 func_0;
    func_0.forward(param, transform);

    Eigen::Matrix<double, 4, 3> Q_to_M = Eigen::Matrix<double, 4, 3>::Zero();
    Q_to_M(0,0) =1.0; Q_to_M(1,1) = 1.0; Q_to_M(3,2) = 1.0;
    Eigen::Matrix<double,4,3> B = Q_to_M;

    Func_1 func_1; Eigen::Matrix<double, 3,4> output_1;
    func_1.forward(&projection_mat, transform, output_1);

    Func_2 func_2; Eigen::Matrix3d M; // M
    func_2.forward(&B, output_1, M);

    Func_3 func_3; Eigen::Matrix3d M_inv;
    func_3.forward(M, M_inv);
    //func_3.testDerivate(M);

    double total_res = 0.0;
    std::vector<Func_4> func_4_vec(points.size());
    std::vector<Func_5> func_5_vec(points.size());
    std::vector<Func_6> func_6_vec(points.size());
    std::vector<double> res_vec(points.size());
    double radius;
    if(circle_id == INNER_CIRCLE)
        radius = x[1][3];
    else
        assert("currently only one circle [INNER_CIRCLE] is supported");

    for(unsigned int point_i = 0; point_i < points.size(); point_i ++){
        Eigen::Vector3d un_normalized_point_in_plane;
        Eigen::Vector3d point_in_image;
        
        point_in_image << points[point_i].x + rect_roi.x, points[point_i].y + rect_roi.y, 1.0;
        func_4_vec[point_i].forward(point_in_image, M_inv, un_normalized_point_in_plane);

        Eigen::Vector3d point_in_plane;
        func_5_vec[point_i].forward(un_normalized_point_in_plane, point_in_plane);
        //func_5.testDerivate(un_normalized_point_in_plane);

        double res;
        func_6_vec[point_i].forward(point_in_plane(0), point_in_plane(1), radius, res);
        //func_6.testDerivate(point_in_plane(0), point_in_plane(1), radius);
        total_res += res*res;
        res_vec[point_i] = res;
    }
    residuals[0] = std::sqrt(total_res);

    if (jacobians != NULL && (jacobians[0] != NULL || jacobians[1] != NULL)) {
        double total_deriv_r = 0.0;
        Eigen::Matrix3d total_deriv_M_inv = Eigen::Matrix3d::Zero();
        for(unsigned int point_i = 0; point_i < points.size(); point_i++){
            double deriv_r, deriv_x, deriv_y;
            func_6_vec[point_i].backward(res_vec[point_i]/residuals[0], deriv_x, deriv_y, deriv_r);

            Eigen::Vector3d deriv_point_normalized, deriv_point;
            deriv_point_normalized << deriv_x, deriv_y, 0.0;
            func_5_vec[point_i].backward(deriv_point_normalized, deriv_point);

            Eigen::Matrix3d deriv_M_inv;
            func_4_vec[point_i].backward(deriv_point, deriv_M_inv);

            total_deriv_r += deriv_r;
            total_deriv_M_inv += deriv_M_inv;
        }
            Eigen::Matrix3d deriv_M;
            func_3.backward(total_deriv_M_inv, deriv_M);

            Eigen::Matrix<double,3,4> deriv_output1;
            func_2.backward(deriv_M, deriv_output1);

            Eigen::Matrix4d deriv_transform;
            func_1.backward(deriv_output1, deriv_transform);

            EigenVector8d deriv_param;
            func_0.backward(deriv_transform, deriv_param);
            if(jacobians[1] != NULL){
                jacobians[1][0] = deriv_param(0);//x
                jacobians[1][1] = deriv_param(1);//y
                jacobians[1][2] = deriv_param(2);//z
                if(circle_id == INNER_CIRCLE){
                    jacobians[1][3] = total_deriv_r; //r_inner
                    jacobians[1][4] = 0;           
                }
                else{
                    jacobians[1][3] = 0;
                    jacobians[1][4] = total_deriv_r; //r_outer
                }
            }
            if(jacobians[0] != NULL){
                jacobians[0][0] = deriv_param(3);//q_x
                jacobians[0][1] = deriv_param(4);//q_y
                jacobians[0][2] = deriv_param(5);//q_z
                jacobians[0][3] = deriv_param(6);//q_w
            }
    }
    return true;

}


void PreciseCircleSolverImpl::init(const cv::Rect& left_rect_roi_input, const cv::Rect& right_rect_roi_input,
            const StereoCircleValidContours& stereo_valid_contours_input,
            const Eigen::Matrix<double, 3, 4> &left_projection_input, const Eigen::Matrix<double, 3, 4> &right_projection_input,
            cv::Mat* left_img_ptr_input, cv::Mat* right_img_ptr_input)
{
    stereo_valid_contours = stereo_valid_contours_input;
    left_projection = left_projection_input;
    right_projection = right_projection_input;
    left_img_ptr = left_img_ptr_input;
    right_img_ptr = right_img_ptr_input;
    left_rect_roi = left_rect_roi_input;
    right_rect_roi = right_rect_roi_input;
}

void PreciseCircleSolverImpl::getInitCircleParams(const Circle3D &init_circle, CircleParams *init_params_ptr)
{
    EigenVector8d params = init_circle.getParams();
    translateParam(params, *init_params_ptr);

}
void PreciseCircleSolverImpl::buildBlocks(ceres::Problem& problem, ceres::LossFunction* loss, const VecPoint& contours_point, int circle_id, 
                const cv::Rect& rect_roi, const Eigen::Matrix<double,3,4>& projection_mat, CircleParams *circle_params_ptr)
{
    // Use LocalParameterization for quaternions
    ceres::CostFunction *cost_function = new PointToEllipseDistance(rect_roi, contours_point,
                                                    circle_id, projection_mat);
    problem.AddResidualBlock(cost_function, loss, circle_params_ptr->quaternion, circle_params_ptr->rest);

}
void PreciseCircleSolverImpl::buildProblem(ceres::Problem &problem, CircleParams *circle_params_ptr, double robust_threshold)
{

    //ceres::LossFunction* loss = new ceres::TolerantLoss(1.0, 20.0);
    //ceres::LossFunctionWrapper* loss = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(robust_threshold), ceres::TAKE_OWNERSHIP);
    //ceres::LossFunction *loss = new ceres::CauchyLoss(robust_threshold);
    ceres::LossFunction* loss = NULL;

    

    {
        VecPoint& contours_point = *(stereo_valid_contours.left);
        if(left_img_ptr != nullptr){
            for (unsigned int show_i = 0; show_i < contours_point.size(); show_i++)
            {
                left_img_ptr->at<cv::Vec3b>(contours_point[show_i]) = cv::Vec3b(255, 255, 0);
            }
        }
    }   
    
    {
        VecPoint& contours_point = *(stereo_valid_contours.right);
        if(right_img_ptr != nullptr){
            for (unsigned int show_i = 0; show_i < contours_point.size(); show_i++)
            {
                right_img_ptr->at<cv::Vec3b>(contours_point[show_i]) = cv::Vec3b(255, 255, 0);
            }
        }
    }
    // left
    
    buildBlocks(problem, loss, *(stereo_valid_contours.left), INNER_CIRCLE, 
               left_rect_roi, left_projection, circle_params_ptr);
    
    // right
    buildBlocks(problem, loss, *(stereo_valid_contours.right), INNER_CIRCLE, 
               right_rect_roi, right_projection, circle_params_ptr);

    
}
void PreciseCircleSolverImpl::solveProblem(ceres::Problem &problem, CircleParams *circle_params_ptr, Circle3D &result, double& final_cost_per_point)
{
    EigenVector8d init_params, final_params;
    translateParam(*circle_params_ptr, init_params);


    ceres::LocalParameterization *quaternion_parameterization = new ceres::EigenQuaternionParameterization;
    problem.SetParameterization(circle_params_ptr->quaternion, quaternion_parameterization);

    // Build and solve the problem.
    ceres::Solver::Options options;
    //options.check_gradients = true; // if true, check the gradients' correctness, you should use FullReport to see the detail
    //options.gradient_check_relative_precision = 1e+6;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::TRUST_REGION;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.FullReport() << "\n";
    std::cout << summary.BriefReport() << "\n";
    translateParam(*circle_params_ptr, final_params);
    std::cout << "x  : " << init_params(0) << " -> " << final_params(0) << "\n";
    std::cout << "y  : " << init_params(1) << " -> " << final_params(1) << "\n";
    std::cout << "z  : " << init_params(2) << " -> " << final_params(2) << "\n";
    std::cout << "q_x  : " << init_params(3) << " -> " << final_params(3) << "\n";
    std::cout << "q_y  : " << init_params(4) << " -> " << final_params(4) << "\n";
    std::cout << "q_z  : " << init_params(5) << " -> " << final_params(5) << "\n";
    std::cout << "q_w  : " << init_params(6) << " -> " << final_params(6) << "\n";
    std::cout << "r_i: " << init_params(7) << " -> " << final_params(7) << "\n";

    result.setParams(final_params);
    double point_num = stereo_valid_contours.left->size() + stereo_valid_contours.right->size();
    final_cost_per_point = summary.final_cost / point_num;
}

void TestLoss::visualize(){
    Circle3D circles;
    EigenVector8d params;
    double x = x_origin + x_bar.value * xyz_ratio;
    double y = y_origin + y_bar.value * xyz_ratio;
    double z = z_origin + z_bar.value * xyz_ratio;
    double a = a_origin + a_bar.value * abc_ratio;
    double b = b_origin + b_bar.value * abc_ratio;
    double c = c_origin + c_bar.value * abc_ratio;
    double r_i = r_i_origin + r_inner_bar.value * r_ratio;

    Eigen::Matrix3d rot_mat;
    translateParam(a,b,c,rot_mat);
    rot_mat = rot_mat.eval() * init_rot;
    Eigen::Quaterniond q(rot_mat);
    params << x,y,z,q.x(),q.y(),q.z(), q.w(),r_i;
    
    circles.setParams(params);
    std::cout<<"plane: "<<circles.plane.transpose()<<std::endl;
    cv::Mat left_image_visualize = left_image.clone();
    cv::Mat right_image_visualize = right_image.clone();
    cv::Rect full_roi(0,0,left_image_visualize.cols, left_image_visualize.rows);
    reprojectCircles(left_image_visualize, full_roi, circles, LEFT_CAMERA, 100, CV_RGB(255,255,0));
    reprojectCircles(right_image_visualize, full_roi, circles, RIGHT_CAMERA, 100, CV_RGB(255,255,0));

    cv::imshow("left", left_image_visualize);
    cv::imshow("right", right_image_visualize);
    std::cout<<"x: "<<x<<", y: "<<y<<", z: "<<z
            <<", a: "<<a<<", b: "<<b<<", c: "<<c
            <<", r_i: "<<r_i<<std::endl;
    translateParam(params, *circle_params_ptr);
    ceres::Problem::EvaluateOptions eval_opt;
    std::vector<double> ceres_loss; double total_ceres_loss=0.0;
    problem_ptr->Evaluate(eval_opt, NULL, &ceres_loss, NULL, NULL);
    for(unsigned int i=0; i<ceres_loss.size(); i++){
        total_ceres_loss += ceres_loss[i]*ceres_loss[i];
    }
    std::cout<<"ceres loss: "<<total_ceres_loss<<std::endl;
}