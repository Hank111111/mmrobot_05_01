#include "mm_visual_postion/hand_trunck/RoughCircleSolver.h"
#include "mm_visual_postion/hand_trunck/PreciseCircleSolverForMultiview.h"
#include "mm_visual_postion/hand_trunck/Z_AngleSolver.h"
#include <limits>

class HandTrunkSolver{
private:
    RoughCircleSolver rough_solver;
    PreciseCircleSolverForMultiView precise_solver;
    Z_AngleSolver z_angle_solver;
    std::shared_ptr<StereoCameraArmModel> stereo_cam_ptr;
public:
    HandTrunkSolver(std::shared_ptr<StereoCameraArmModel>& stereo_cam_ptr):
        rough_solver(stereo_cam_ptr), precise_solver(stereo_cam_ptr), stereo_cam_ptr(stereo_cam_ptr)
    {}
    
    bool findTrunk(const std::vector<cv::Mat>& left_rectified_image_roi_vec,const std::vector<cv::Mat>& right_rectified_image_roi_vec, 
                    std::vector<cv::Rect>& left_rect_roi_vec, std::vector<cv::Rect>& right_rect_roi_vec,
                    std::vector<Eigen::Matrix4d>& transform_cam_to_base_vec, 
                    double estimate_radius, double radius_threshold, double max_acceptable_cost_per_point,
                    double estimate_square_trunk_length,
                    Circle3D& result_circle, Eigen::Matrix4d& T_base_to_trunk,
                    cv::Mat* visualize_img_ptr=nullptr){
        /**
         * result_concentric_circle does not contain the information of the trunk's rotation about z-axis, 
         * which is presented in T_base_to_trunk.
         * (1,0,0) vector in the trunk coordinate is the direction of the upper line of the trunk.
         */ 
        assert(right_rectified_image_roi_vec.size() == left_rectified_image_roi_vec.size());
        assert(right_rectified_image_roi_vec.size() == transform_cam_to_base_vec.size());
        assert(left_rectified_image_roi_vec.size() >= 1);
        std::vector<StereoCircleValidContours> chosen_circle_valid_contours_in_roi_vec;
        std::vector<Circle3D> roughly_circle_vec_in_base;
        // get circles' roughly pose in each camera's coordinate.
        std::vector<cv::RotatedRect> left_ellipse_vec, right_ellipse_vec;
        std::vector<cv::Mat> left_edge_roi_vec, right_edge_roi_vec; // save the roi edge image for trunk

        std::vector<int> valid_index; // the index of valid image view
        for(unsigned int i=0; i<left_rectified_image_roi_vec.size(); i++){
            cv::Mat left_gray, right_gray;
            cv::cvtColor(left_rectified_image_roi_vec[i], left_gray, CV_BGR2GRAY);
            cv::cvtColor(right_rectified_image_roi_vec[i], right_gray, CV_BGR2GRAY);
            cv::Mat left_edge, right_edge;
            /* //tune the canny params
                CannyTrackBar canny_bar_left(left_gray, "canny_bar_of_left_image", 500, 1500, 3000, 5000);
                CannyTrackBar canny_bar_right(right_gray, "canny_bar_of_right_image", 500, 1500, 3000, 5000);
                left_edge = canny_bar_left.edge;
                right_edge = canny_bar_right.edge;
            */
            cv::Canny(left_gray, left_edge, 500, 1500, 5);
            cv::Canny(right_gray, right_edge, 500, 1500, 5);
            left_edge_roi_vec.push_back(left_edge);
            right_edge_roi_vec.push_back(right_edge);
            std::vector<Circle3D> circle_3d_vec;
            std::vector<StereoCircleValidContours> stereo_valid_contours_in_roi_vec;
            std::vector<EllipseWithScore> valid_left_ellipses_box, valid_right_ellipses_box;
            rough_solver.getPossibleCircles(left_edge, right_edge, left_rect_roi_vec[i], right_rect_roi_vec[i],
                                            circle_3d_vec, 
                                            stereo_valid_contours_in_roi_vec,
                                            valid_left_ellipses_box, valid_right_ellipses_box, estimate_radius, radius_threshold);

            if(circle_3d_vec.size() == 0){
                ROS_WARN("Cannot find the circle pair in images.");
                return false;
            } 
            // choose the circle with minimum error to refine
            
            double max_score = std::numeric_limits<double>::min(); 
            int max_index = 0;
            for(unsigned int i=0; i<circle_3d_vec.size(); i++){
                if(circle_3d_vec[i].score > max_score){
                    max_score = circle_3d_vec[i].score;
                    max_index = i;
                }    
            }
            /*
           // choose the outer circle to refine
            double max_radius = std::numeric_limits<double>::min(); 
            int max_index = 0;
            for(unsigned int i=0; i<circle_3d_vec.size(); i++){
                if(circle_3d_vec[i].radius > max_radius){
                    max_radius = circle_3d_vec[i].radius;
                    max_index = i;
                }    
            }
            */            
            cv::RotatedRect left_ellipse = valid_left_ellipses_box[max_index].box;
            cv::RotatedRect right_ellipse = valid_right_ellipses_box[max_index].box;           
            left_ellipse_vec.push_back(left_ellipse);
            right_ellipse_vec.push_back(right_ellipse);

             // judge whether ellipses are completely in the image
            if(left_ellipse.center.x + left_ellipse.size.width/2.0 > left_edge.cols + left_rect_roi_vec[i].x || 
                left_ellipse.center.x - left_ellipse.size.width/2.0 <  left_rect_roi_vec[i].x ||
                left_ellipse.center.y + left_ellipse.size.height/2.0 > left_edge.rows +  left_rect_roi_vec[i].y|| 
                left_ellipse.center.y - left_ellipse.size.height/2.0 <  left_rect_roi_vec[i].y){
                ROS_WARN("circle is not in the center of left ROI, please move the camera...");
                return false;
            }
            
            if(right_ellipse.center.x + right_ellipse.size.width/2.0 > right_edge.cols + right_rect_roi_vec[i].x || 
                right_ellipse.center.x - right_ellipse.size.width/2.0 <right_rect_roi_vec[i].x ||
                right_ellipse.center.y + right_ellipse.size.height/2.0 > right_edge.rows +right_rect_roi_vec[i].y || 
                right_ellipse.center.y - right_ellipse.size.height/2.0 < right_rect_roi_vec[i].y){
                ROS_WARN("circle is not in the center of right ROI, please move the camera...");
                return false;
            }
            
            valid_index.push_back(i);
            // visualize the circles (by reprojection)
            cv::Mat left_init_show_image;// = left_rectified_image.clone();
            cv::Mat right_init_show_image;// = right_rectified_image.clone();
            if(visualize_img_ptr != nullptr){

                cv::cvtColor(left_edge, left_init_show_image, CV_GRAY2BGR);
                cv::cvtColor(right_edge, right_init_show_image, CV_GRAY2BGR);


                rough_solver.reprojectCircles(left_init_show_image, left_rect_roi_vec[i], circle_3d_vec[max_index], LEFT_CAMERA, 500, cv::Scalar(255,0,0));
                rough_solver.reprojectCircles(right_init_show_image, right_rect_roi_vec[i], circle_3d_vec[max_index], RIGHT_CAMERA, 500, cv::Scalar(255,0,0));
                cv::hconcat(left_init_show_image, right_init_show_image, *visualize_img_ptr);
            }
            chosen_circle_valid_contours_in_roi_vec.push_back(stereo_valid_contours_in_roi_vec[max_index]);
            roughly_circle_vec_in_base.push_back(Circle3D::transformCircle3D(circle_3d_vec[max_index], transform_cam_to_base_vec[i].inverse()));

        }
        Circle3D avg_roughly_circle_in_base = Circle3D::average(roughly_circle_vec_in_base);



        // visualize the circles (by reprojection)
        cv::Mat left_show_image, right_show_image;
        cv::Mat *left_show_image_ptr = nullptr;
        cv::Mat *right_show_image_ptr = nullptr;
        cv::Mat left_edge, right_edge;
        if(visualize_img_ptr != nullptr){
            left_show_image = (left_rectified_image_roi_vec[0]).clone();
            right_show_image = (right_rectified_image_roi_vec[0]).clone();            
            left_show_image_ptr = &(left_show_image);
            right_show_image_ptr = &(right_show_image);

            cv::Mat left_gray, right_gray;
            cv::cvtColor(left_rectified_image_roi_vec[0], left_gray, CV_BGR2GRAY);
            cv::cvtColor(right_rectified_image_roi_vec[0], right_gray, CV_BGR2GRAY);
            
            cv::Canny(left_gray, left_edge, 500, 1500, 5);
            cv::Canny(right_gray, right_edge, 500, 1500, 5);
            cv::cvtColor(left_edge, left_edge, CV_GRAY2BGR);
            cv::cvtColor(right_edge, right_edge, CV_GRAY2BGR);

            cv::Mat left_show_image = left_rectified_image_roi_vec[0].clone();
            Circle3D avg_roughly_circle_in_cam = Circle3D::transformCircle3D(avg_roughly_circle_in_base, transform_cam_to_base_vec[0]);
            precise_solver.reprojectCircles(left_show_image, left_rect_roi_vec[0], avg_roughly_circle_in_cam, LEFT_CAMERA, 500, cv::Scalar(255,0,0));
        }

        double final_cost_per_point;
        precise_solver.init(avg_roughly_circle_in_base, left_rect_roi_vec, right_rect_roi_vec, chosen_circle_valid_contours_in_roi_vec, transform_cam_to_base_vec, 10, left_show_image_ptr, right_show_image_ptr);
        precise_solver.solve(result_circle, final_cost_per_point);
        std::cout<<"final_cost_per_point: "<<final_cost_per_point<<std::endl;
        if(visualize_img_ptr != nullptr){
            Circle3D circle_in_cam = Circle3D::transformCircle3D(result_circle, transform_cam_to_base_vec[0]);
            precise_solver.reprojectCircles(left_edge, left_rect_roi_vec[0], circle_in_cam, LEFT_CAMERA, 500, cv::Scalar(255,0,0));
            precise_solver.reprojectCircles(right_edge, right_rect_roi_vec[0], circle_in_cam, RIGHT_CAMERA, 500, cv::Scalar(255,0,0));

            cv::hconcat(left_edge, right_edge, *visualize_img_ptr);
            cv::namedWindow("handtrunk_visualize", 0);
            cv::imshow("handtrunk_visualize", *visualize_img_ptr);
            cv::waitKey(20);
        }
        if(final_cost_per_point > max_acceptable_cost_per_point){
            ROS_WARN("final cost per point: [%f] is bigger than max acceptable cost per point [%f], failed.", final_cost_per_point, max_acceptable_cost_per_point);
            return false;
        }

        // compute z-axis rotation from plane to trunk
        std::vector<Eigen::Matrix4d> T_camera_to_plane_vec;
        Eigen::Matrix4d T_base_to_plane;
        result_circle.getTransformOriginToCircle(T_base_to_plane);
        T_camera_to_plane_vec.resize(valid_index.size());
        for(unsigned int i=0; i<valid_index.size(); i++){
            T_camera_to_plane_vec[i] = transform_cam_to_base_vec[valid_index[i]] * T_base_to_plane;
        }
        Eigen::Matrix4d T_plane_to_trunk;
        if(!z_angle_solver.getTrunkPoseInPlane(T_camera_to_plane_vec,
                                            left_ellipse_vec, right_ellipse_vec,
                                            left_edge_roi_vec, right_edge_roi_vec,
                                            left_rect_roi_vec, right_rect_roi_vec,
                                            stereo_cam_ptr->left_camera.projection_mat,
                                            stereo_cam_ptr->right_camera.projection_mat,
                                            estimate_square_trunk_length, result_circle.radius,
                                            T_plane_to_trunk
                                          )){
            ROS_WARN("Cannot determinate the trunk's rotation around z-axis");
            return false;
        } 
    
        T_base_to_trunk = T_base_to_plane * T_plane_to_trunk;
        Eigen::Matrix4d T_cam_to_trunk = transform_cam_to_base_vec[0] * T_base_to_trunk;
        Eigen::Quaterniond q(T_cam_to_trunk.block<3,3>(0,0));
        Eigen::Quaterniond q_rot_total(1,0,0,0);
        double q_z = q.w() > 0? q.z(): -q.z();
        while(! (q_z > -0.1 && q_z < 0.7)){ // between -0.=7
            Eigen::Quaterniond q_rot(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
            q_rot_total *= q_rot;
            q = q * q_rot;
            if(q.w()<0) q_z = -q.z();
            else q_z = q.z();
        }
        Eigen::Matrix4d T_rot_total = Eigen::Matrix4d::Identity();
        T_rot_total.block<3,3>(0,0) = Eigen::Matrix3d(q_rot_total);
        T_base_to_trunk *= T_rot_total;
        
        return true;
    }
};