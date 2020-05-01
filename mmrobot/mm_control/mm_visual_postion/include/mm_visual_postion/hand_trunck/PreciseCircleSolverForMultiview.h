#ifndef __PRECISE_CIRCLE_SOLVER_FOR_MULTIVIEW_H__
#define __PRECISE_CIRCLE_SOLVER_FOR_MULTIVIEW_H__
#include "mm_visual_postion/hand_trunck/PreciseCircleSolver.h"
class PreciseCircleSolverForMultiView:public BaseCircleSolver{
private:
    PreciseCircleSolverImpl solver_impl;
    CircleParams circle_params;
    std::shared_ptr<ceres::Problem> problem_ptr;
public:
    PreciseCircleSolverForMultiView(std::shared_ptr<StereoCameraArmModel> &stereo_cam_ptr_) : BaseCircleSolver(stereo_cam_ptr_){};
    void init(const Circle3D &init_circle, 
              std::vector<cv::Rect>& left_rect_roi_vec, std::vector<cv::Rect>& right_rect_roi_vec,
              std::vector<StereoCircleValidContours>& stereo_valid_contours_in_roi_vec,
              std::vector<Eigen::Matrix4d> &transform_cam_to_base_vec, double robust_threshold,
              cv::Mat* left_image_ptr=nullptr, cv::Mat* right_image_ptr=nullptr)
    {
        problem_ptr = std::make_shared<ceres::Problem>();// new problem

        // we calculate the circles pose in base corrdiante.
        for(unsigned int i=0; i<stereo_valid_contours_in_roi_vec.size(); i++){
            Eigen::Matrix<double,3,4> left_projection_to_base = stereo_cam_ptr->left_camera.projection_mat * transform_cam_to_base_vec[i];
            Eigen::Matrix<double,3,4> right_projection_to_base = stereo_cam_ptr->right_camera.projection_mat * transform_cam_to_base_vec[i];

            /*
            Eigen::Vector4d center_in_base;
            center_in_base <<-1374.660000,	330.434000,	139.820000, 1.0;
            Eigen::Vector3d center_in_cam;
            center_in_cam = left_projection_to_base * center_in_base;
            center_in_cam(0) /= center_in_cam(2);
            center_in_cam(1) /= center_in_cam(2);
            center_in_cam(2) = 1.0;
            std::cout<<"cetner_in_cam :" << center_in_cam.transpose() <<std::endl;
            */
            solver_impl.init( left_rect_roi_vec[i], right_rect_roi_vec[i], stereo_valid_contours_in_roi_vec[i], left_projection_to_base, right_projection_to_base, left_image_ptr, right_image_ptr);
            solver_impl.getInitCircleParams(init_circle, &circle_params);
            solver_impl.buildProblem(*problem_ptr, &circle_params, robust_threshold);
        }
        
            
    }
    void solve(Circle3D &result, double& final_cost_per_point)
    {
        solver_impl.solveProblem(*problem_ptr, &circle_params, result, final_cost_per_point);
    }
};

#endif