#ifndef __Z_ANGLE_SOLVER_H__
#define __Z_ANGLE_SOLVER_H__
#include "mm_visual_postion/hand_trunck/CircleModel.h"
#include "mm_visual_postion/hand_trunck/BaseCircleSolver.h"
#include "mm_visual_postion/switch/SwitchFinderByHough.h"
#include "mm_visual_postion/utils/utils.h"

double normalizeSlopeAngle(double theta){
    // 0 to pi
    while(theta < 0){
        theta += M_PI;
    }
    while(theta > M_PI){
        theta -= M_PI;
    }
    return theta;
}

double normalizeSlopeAngleCenter(double theta){
    // -pi/2 to pi/2
    while(theta < -M_PI/2.0){
        theta += M_PI;
    }
    while(theta > M_PI/2.0){
        theta -= M_PI;
    }
    return theta;
}
bool getEndpoint(const cv::Vec3f& line, const cv::Size& size, cv::Point2f& end_point_0, cv::Point2f& end_point_1){
    // line equation rho - x*cos(theta) - y* sin(theta) = 0 
    double rho = line[0];
    double cos_theta = std::cos(line[1]); //radian
    double sin_theta = std::sin(line[1]);

    // intersect this line with four borders
    cv::Point2f left_intersect_point, up_intersect_point, right_intersect_point, down_intersect_point;
    std::vector<cv::Point2f> unsort_end_points;
    // left (x=0)
    left_intersect_point.x = 0;
    left_intersect_point.y = rho / sin_theta;
    if((!std::isnan(left_intersect_point.y)) && left_intersect_point.y <= size.height && left_intersect_point.y >= 0)
        unsort_end_points.push_back(left_intersect_point);
    // right (x=size.width)
    right_intersect_point.x = size.width;
    right_intersect_point.y = (rho - size.width * cos_theta) / sin_theta;
    if((!std::isnan(right_intersect_point.y)) && right_intersect_point.y <= size.height && right_intersect_point.y >= 0)
        unsort_end_points.push_back(right_intersect_point);
    // up (y=0)
    up_intersect_point.x = rho / cos_theta;
    up_intersect_point.y = 0;
    if((!std::isnan(up_intersect_point.x)) && up_intersect_point.x <= size.width && up_intersect_point.x >= 0)
        unsort_end_points.push_back(up_intersect_point);
    // down(y=size.height)
    down_intersect_point.x = (rho - size.height * sin_theta) / cos_theta;
    down_intersect_point.y = size.height;
    if((!std::isnan(down_intersect_point.x)) && down_intersect_point.x <= size.width && down_intersect_point.x >= 0)
        unsort_end_points.push_back(down_intersect_point);
    
    if(unsort_end_points.size() < 2) return false; // perhaps the line isn't in the area of "size"
    if(unsort_end_points[0].x < unsort_end_points[1].x){
        end_point_0 = unsort_end_points[0];
        end_point_1 = unsort_end_points[1];
    }
    else{
        end_point_0 = unsort_end_points[1];
        end_point_1 = unsort_end_points[0];
    }
    return true;
}
struct TrunkLines
{
    std::vector<cv::Point2f> vertical_lines_endpoints[2];
    std::vector<cv::Point2f> horizontal_lines_endpoints[2];
    cv::Rect roi_rect;
    TrunkLines(){
    }
};

class Z_AngleSolver{
private:
public:
    Z_AngleSolver(){}
    
    bool findTrunkSquare(const cv::Mat& roi_edge, const cv::Rect& roi_rect, cv::RotatedRect ellipse,
                        const Eigen::Matrix<double,3,4>& projection_matrix, const Eigen::Matrix4d& T_cam_to_circle, double square_trunk_length, double circle_radius, TrunkLines& trunk_lines){
        /**
         * trunk_length is in millimeter
         */ 
        
        // estimate the trunk's pixel length in the image
        Eigen::Vector4d trunk_origin, trunk_endpoint, circle_endpoint;
        trunk_origin << 0,0,0,1;
        trunk_endpoint << square_trunk_length/std::sqrt(2.0), 0, 0, 1;
        circle_endpoint << circle_radius, 0,0, 1;
        Eigen::Vector3d trunk_origin_in_image, trunk_endpoint_in_image, circle_endpoint_in_image;
        trunk_origin_in_image = projection_matrix * T_cam_to_circle * trunk_origin;
        trunk_endpoint_in_image = projection_matrix * T_cam_to_circle * trunk_endpoint;
        circle_endpoint_in_image = projection_matrix * T_cam_to_circle * circle_endpoint;
        
        normalizeVector3d(trunk_origin_in_image);
        normalizeVector3d(trunk_endpoint_in_image);
        normalizeVector3d(circle_endpoint_in_image);
        double trunk_circle_ratio = (trunk_endpoint_in_image - trunk_origin_in_image).norm() / (circle_endpoint_in_image - trunk_origin_in_image).norm();

        

        cv::Rect trunk_roi(ellipse.center.x - roi_rect.x - ellipse.size.width * trunk_circle_ratio / 2.0 - 10,
                            ellipse.center.y -roi_rect.y - ellipse.size.height * trunk_circle_ratio / 2.0 - 10,
                            ellipse.size.width * trunk_circle_ratio + 20,
                            ellipse.size.height * trunk_circle_ratio + 20);
        cv::RotatedRect ellipse_round_trunk;
        ellipse_round_trunk.center = cv::Point2f(0,0);
        ellipse_round_trunk.size.width = ellipse.size.width * trunk_circle_ratio + 10;
        ellipse_round_trunk.size.height = ellipse.size.height * trunk_circle_ratio + 10;
        
        cv::Mat filtered_edge;
        maskEllipseEdge(roi_edge(trunk_roi), ellipse_round_trunk, filtered_edge, 0);

        std::vector<cv::Vec3f> possible_lines;
        
        cv_v3_4_5::HoughLines(filtered_edge, possible_lines, 1, M_PI / 180. * 0.2, 10, 0, 0, 0, M_PI); 
        std::vector<cv::Vec3f> filterd_lines;
        if(!possible_lines.empty())
            linesNMS(possible_lines, filterd_lines,3); 

        // visualize lines
        cv::Mat show_img;
        cv::cvtColor(filtered_edge, show_img, CV_GRAY2BGR);
        drawLines(show_img, filterd_lines, 255, 128, 0);

        if(filterd_lines.empty()) return false;

        // cluster the lines
        std::vector<cv::Vec3f> horizontal_lines, vertical_lines;
        clusterLines(filterd_lines, horizontal_lines, vertical_lines, M_PI/180.0 * 5);
        drawLines(show_img, horizontal_lines, 128, 0, 0);
        drawLines(show_img, vertical_lines, 0, 128, 0);


        // refine lines
        cv::Point2f end_point_0, end_point_1;
        trunk_lines.roi_rect = roi_rect;
        for(unsigned int i=0; i<horizontal_lines.size(); i++){
            if(getEndpoint(horizontal_lines[i], cv::Size(filtered_edge.cols, filtered_edge.rows), end_point_0, end_point_1)){
                cv::Vec3f refine_line;
                LineRefiner refiner(filtered_edge, end_point_0, end_point_1, refine_line);
                getEndpoint(refine_line, cv::Size(filtered_edge.cols, filtered_edge.rows), end_point_0, end_point_1);
                end_point_0.x += roi_rect.x;
                end_point_0.y += roi_rect.y;
                end_point_1.x += roi_rect.x;
                end_point_1.y += roi_rect.y;
                
                trunk_lines.horizontal_lines_endpoints[0].push_back(end_point_0);
                trunk_lines.horizontal_lines_endpoints[1].push_back(end_point_1);
            }
        }
        for(unsigned int i=0; i<vertical_lines.size(); i++){
            if(getEndpoint(vertical_lines[i], cv::Size(filtered_edge.cols, filtered_edge.rows), end_point_0, end_point_1)){
                cv::Vec3f refine_line;
                LineRefiner refiner(filtered_edge, end_point_0, end_point_1, refine_line);
                getEndpoint(refine_line, cv::Size(filtered_edge.cols, filtered_edge.rows), end_point_0, end_point_1);
                end_point_0.x += roi_rect.x;
                end_point_0.y += roi_rect.y;
                end_point_1.x += roi_rect.x;
                end_point_1.y += roi_rect.y;
                trunk_lines.vertical_lines_endpoints[0].push_back(end_point_0);
                trunk_lines.vertical_lines_endpoints[1].push_back(end_point_1);
            }
        }
        return true;
    }
    bool clusterTheta(const std::vector<double>& theta_vec, std::vector<double>& horizontal_theta, std::vector<double>& vertical_theta, double theta_fit_threshold){
        /**
         * cluster the theta into two groups, the horizontal and the vertical.
         * Normally, the vertical lines' theta_v hand the horizontal lines' theta_h should have this relation approximately:
         *  theta_h = theta_v + PI/2 (with normalized angle)
         * or theta_h = theta_v - PI/2
         * Here we use a RANSAC similaire algorithm in order to be more robust and detect the outliers, which is not horizontal lines, neither vertical lines
         */   
        std::vector<int> fit_lines_num_vec(theta_vec.size());
        int max_fit_try_index = 0;
        for(unsigned int try_i=0; try_i<theta_vec.size(); try_i++){
            double theta = theta_vec[try_i];
            double fit_num=0;
            for(unsigned int test_i =0 ; test_i < theta_vec.size(); test_i++){
                if(fabs(normalizeSlopeAngleCenter(theta + M_PI/2.0 - theta_vec[test_i])) < theta_fit_threshold
                    || fabs(normalizeSlopeAngleCenter(theta - theta_vec[test_i])) < theta_fit_threshold)
                    fit_num++;
            }
            
            fit_lines_num_vec[try_i] = fit_num;
            if(fit_lines_num_vec[try_i] > fit_lines_num_vec[max_fit_try_index]) max_fit_try_index = try_i;
        }
        if(fit_lines_num_vec[max_fit_try_index] <= 1) return false; // to keep correct, at least it needs two fit. 


        // find max_fit lines
        double theta[2] ;
        theta[0] = theta_vec[max_fit_try_index];
        theta[1] = theta_vec[max_fit_try_index] + M_PI/2.0;
        double theta_v, theta_h;
        if(fabs(theta[0] - M_PI/2.0) < fabs(theta[1] - M_PI/2.0)){
            theta_h = theta[0];
            theta_v = theta[1];
        }
        else{
            theta_h = theta[1];
            theta_v = theta[0];
        }
        for(unsigned int test_i=0; test_i < theta_vec.size(); test_i++){
            if(fabs(normalizeSlopeAngleCenter(theta_h - theta_vec[test_i])) < theta_fit_threshold){
                horizontal_theta.push_back(theta_vec[test_i]);
            }
            else if(fabs(normalizeSlopeAngleCenter(theta_v - theta_vec[test_i])) < theta_fit_threshold){
                vertical_theta.push_back(theta_vec[test_i]);
            }
        }

        return true;
    }
    void clusterLines(const std::vector<cv::Vec3f> lines, std::vector<cv::Vec3f>& horizontal_lines, std::vector<cv::Vec3f>& vertical_lines, double theta_fit_threshold){
        /**
         * cluster the lines into two groups, the horizontal and the vertical.
         * Normally, the vertical lines' theta_v hand the horizontal lines' theta_h should have this relation approximately:
         *  theta_h = theta_v + PI/2 (with normalized angle)
         * or theta_h = theta_v - PI/2
         * Here we use a RANSAC similaire algorithm in order to be more robust and detect the outliers, which is not horizontal lines, neither vertical lines
         */ 
        std::vector<int> fit_lines_num_vec(lines.size());
        int max_fit_try_index = 0;
        for(unsigned int try_i=0; try_i<lines.size(); try_i++){
            double theta = lines[try_i][1];
            double fit_num=0;
            for(unsigned int test_i =0 ; test_i < lines.size(); test_i++){
                if(fabs(normalizeSlopeAngleCenter(theta + M_PI/2.0 - lines[test_i][1])) < theta_fit_threshold
                    || fabs(normalizeSlopeAngleCenter(theta - lines[test_i][1])) < theta_fit_threshold)
                    fit_num++;
            }
            
            fit_lines_num_vec[try_i] = fit_num;
            if(fit_lines_num_vec[try_i] > fit_lines_num_vec[max_fit_try_index]) max_fit_try_index = try_i;
        }

        // find max_fit lines
        double theta[2] ;
        theta[0] = lines[max_fit_try_index][1];
        theta[1] = lines[max_fit_try_index][1] + M_PI/2.0;
        double theta_v, theta_h;
        if(fabs(theta[0] - M_PI/2.0) < fabs(theta[1] - M_PI/2.0)){
            theta_h = theta[0];
            theta_v = theta[1];
        }
        else{
            theta_h = theta[1];
            theta_v = theta[0];
        }
        for(unsigned int test_i=0; test_i < lines.size(); test_i++){
            if(fabs(normalizeSlopeAngleCenter(theta_h - lines[test_i][1])) < theta_fit_threshold){
                horizontal_lines.push_back(lines[test_i]);
            }
            else if(fabs(normalizeSlopeAngleCenter(theta_v - lines[test_i][1])) < theta_fit_threshold){
                vertical_lines.push_back(lines[test_i]);
            }
        }
    }
    void findLines(const cv::Mat& edge, std::vector<cv::Vec3f> filtered_lines, double theta1_min, double theta1_max, double theta2_min=-180, double theta2_max=-180){
        // thetas are presented in degree, in [0, 180]
        std::vector<cv::Vec3f> lines;
        
        cv_v3_4_5::HoughLines(edge, lines, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI / 180. * theta1_min, M_PI / 180. * theta1_max); 


        if(theta2_min >= 0 && theta2_max >= 0){
            // consider only two regions
            std::vector<cv::Vec3f> lines_2;
            cv_v3_4_5::HoughLines(edge, lines_2, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI / 180.0 * theta2_min, M_PI / 180.0 * theta2_max); 
            if(!lines_2.empty()){
                lines.insert(std::end(lines), std::begin(lines_2), std::end(lines_2));
                std::sort(lines.begin(), lines.end(), lineScoreGreater);
            }
        }
        if(!lines.empty())
            linesNMS(lines, filtered_lines,3); 

        cv::Mat debug_visualize;
        cv::cvtColor(edge, debug_visualize, CV_GRAY2BGR);
        drawLines(debug_visualize, filtered_lines, 255, 128, 0);
    }

    void maskEllipseEdge(const cv::Mat& edge, cv::RotatedRect inner_ellipse, cv::Mat& filtered_edge, double pattern){
        cv::RotatedRect dilate_ellipse = inner_ellipse;
        dilate_ellipse.center.x = edge.cols/2.0;
        dilate_ellipse.center.y = edge.rows/2.0;
        dilate_ellipse.size.width -= pattern;
        dilate_ellipse.size.height -= pattern;

        filtered_edge = edge.clone();

        Eigen::Matrix3d ellipse_quad_form;
        BaseCircleSolver::translateEllipse(dilate_ellipse, ellipse_quad_form);

        cv::Mat debug_visualize_img;
        cv::cvtColor(edge, debug_visualize_img, CV_GRAY2BGR);
        ellipse(debug_visualize_img, dilate_ellipse, CV_RGB(255,0,0));
        Eigen::Vector3d point;
        for(int i=0; i<edge.rows; i++){
            for(int j=0; j<edge.cols; j++){
                point << i, j ,1;
                if(point.transpose() * ellipse_quad_form * point >= 0){
                    // point is outside of the dilated ellipse
                    filtered_edge.at<uchar>(i,j) = 0;
                }
            }
        }
    }
    
    bool computeThetaInPlane(const Eigen::Matrix<double,3,4>& left_projection_mat, const Eigen::Matrix<double,3,4>& right_projection_mat, const Eigen::Matrix4d& T_camera_to_plane,
                             const TrunkLines& left_trunk_lines, const TrunkLines& right_trunk_lines, std::vector<double>& theta_vec){
        Eigen::Matrix3d left_M_inv, right_M_inv;
        computeMinv(left_projection_mat, T_camera_to_plane, left_M_inv);
        computeMinv(right_projection_mat, T_camera_to_plane, right_M_inv);
        
        // left image
        
        double theta;
        for(unsigned int i=0; i<left_trunk_lines.horizontal_lines_endpoints[0].size(); i++){
            computeThetaInPlane(left_M_inv, left_trunk_lines.horizontal_lines_endpoints[0][i], left_trunk_lines.horizontal_lines_endpoints[1][i], theta);
            theta_vec.push_back(theta); //  normalized into 0,pi
        }

        for(unsigned int i=0; i<left_trunk_lines.vertical_lines_endpoints[0].size(); i++){
            computeThetaInPlane(left_M_inv, left_trunk_lines.vertical_lines_endpoints[0][i], left_trunk_lines.vertical_lines_endpoints[1][i], theta);
            theta_vec.push_back(theta); //  normalized into 0,pi
        }

        for(unsigned int i=0; i<right_trunk_lines.horizontal_lines_endpoints[0].size(); i++){
            computeThetaInPlane(right_M_inv, right_trunk_lines.horizontal_lines_endpoints[0][i], right_trunk_lines.horizontal_lines_endpoints[1][i], theta);
            theta_vec.push_back(theta); //  normalized into 0,pi
        }

        for(unsigned int i=0; i<right_trunk_lines.vertical_lines_endpoints[0].size(); i++){
            computeThetaInPlane(right_M_inv, right_trunk_lines.vertical_lines_endpoints[0][i], right_trunk_lines.vertical_lines_endpoints[1][i], theta);
            theta_vec.push_back(theta); //  normalized into 0,pi
        }
        return true;

    }

    void computeThetaInPlane(const Eigen::Matrix3d& M_inv, const cv::Point2f points_in_image_0, const cv::Point2f points_in_image_1, double& theta){
        Eigen::Vector3d endpoint_0_in_plane, endpoint_1_in_plane;
        computePointsInPlane(M_inv, points_in_image_0, endpoint_0_in_plane);
        computePointsInPlane(M_inv, points_in_image_1, endpoint_1_in_plane);
        theta = std::atan2(endpoint_0_in_plane(1) - endpoint_1_in_plane(1), endpoint_0_in_plane(0) - endpoint_1_in_plane(0));
        if(theta < 0){
            theta += M_PI; //make sure theta is in 0 to pi
        }
    }
    void computePointsInPlane(const Eigen::Matrix3d& M_inv, const cv::Point2f& point_in_image, Eigen::Vector3d& point_in_plane){
        Eigen::Vector3d point_in_image_eigen;
        point_in_image_eigen << point_in_image.x, point_in_image.y, 1.0;
        point_in_plane = M_inv * point_in_image_eigen;
        normalizeVector3d(point_in_plane);
    }
    void computeMinv(const Eigen::Matrix<double,3,4>& projection_mat, const Eigen::Matrix4d& T_camera_to_plane, Eigen::Matrix3d& M_inv){
        /**
         * [u,v,w]^t = M * [x,y,1]^t, where (u/w, v/w) in image is the correspond point of (x, y, 1) (in plane coordinate)
         * M_inv = M.inverse();
         */

        Eigen::Matrix<double, 4,3> N;
        N << 1,0,0,0,1,0,0,0,0,0,0,1;
        Eigen::Matrix3d M = projection_mat * T_camera_to_plane * N;
        M_inv = M.inverse();
    }
    
    bool getTrunkPoseInPlane(const std::vector<Eigen::Matrix4d>& T_camera_to_plane_vec, 
                           const std::vector<cv::RotatedRect>& left_inner_ellipse_vec, 
                           const std::vector<cv::RotatedRect>& right_inner_ellipse_vec,
                           
                           const std::vector<cv::Mat>& left_edge_roi_vec, const std::vector<cv::Mat>& right_edge_roi_vec,
                           const std::vector<cv::Rect>& left_edge_roi_rect_vec, const std::vector<cv::Rect> right_edge_roi_rect_vec,
                           const Eigen::Matrix<double,3,4>& left_projection_mat, const Eigen::Matrix<double,3,4>& right_projection_mat,
                           const double square_trunk_length, const double circle_radius,
                           Eigen::Matrix4d& T_plane_to_trunk){
        /**
        *   The ellipse should be centered in the edge roi image
        */
        assert(left_inner_ellipse_vec.size() == right_inner_ellipse_vec.size());
        assert(left_edge_roi_vec.size() == right_edge_roi_vec.size());
        assert(left_inner_ellipse_vec.size() == left_edge_roi_vec.size());
        std::vector<double> theta_vec;
        for(unsigned int i=0; i<left_inner_ellipse_vec.size(); i++){
            TrunkLines left_trunk_lines, right_trunk_lines;
            findTrunkSquare(left_edge_roi_vec[i], left_edge_roi_rect_vec[i], left_inner_ellipse_vec[i], 
                            left_projection_mat, T_camera_to_plane_vec[i], square_trunk_length, circle_radius,
                            left_trunk_lines);
            findTrunkSquare(right_edge_roi_vec[i], right_edge_roi_rect_vec[i], right_inner_ellipse_vec[i], 
                            right_projection_mat, T_camera_to_plane_vec[i], square_trunk_length, circle_radius,
                            right_trunk_lines);
            computeThetaInPlane(left_projection_mat, right_projection_mat, T_camera_to_plane_vec[i], 
                                left_trunk_lines, right_trunk_lines, theta_vec);
        }
        std::vector<double> horizontal_theta_vec, vertical_theta_vec;
        if(!clusterTheta(theta_vec, horizontal_theta_vec, vertical_theta_vec, M_PI/180.0 * 5)) return false;

        double delta_theta_avg = 0;
        double theta_avg;
        if(horizontal_theta_vec.size() > 0)
            theta_avg = horizontal_theta_vec[0];
        else if(vertical_theta_vec.size() > 0)
            theta_avg = vertical_theta_vec[0];
        else 
            return false;


        for(unsigned int i=0; i<horizontal_theta_vec.size(); i++){
            double delta_theta = normalizeSlopeAngleCenter(horizontal_theta_vec[i] - theta_avg);
            delta_theta_avg += delta_theta;
        }
        for(unsigned int i=0; i<vertical_theta_vec.size(); i++){
            double delta_theta = normalizeSlopeAngleCenter(vertical_theta_vec[i] - theta_avg + M_PI/2.0);
            delta_theta_avg += delta_theta;
        }
        delta_theta_avg /= ((double) horizontal_theta_vec.size() + (double) vertical_theta_vec.size());
        theta_avg = theta_avg + delta_theta_avg;
        
        while(theta_avg > M_PI/2.0) theta_avg -= M_PI/2.0;
        while(theta_avg < -M_PI/2.0) theta_avg += M_PI/2.0;
        T_plane_to_trunk.setIdentity();
        T_plane_to_trunk.block<3,3>(0,0) = Eigen::AngleAxisd(theta_avg, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 

        cv::Mat visualize_roi;
        cv::cvtColor(left_edge_roi_vec[0], visualize_roi, CV_GRAY2BGR );
        visualizeTrunk(visualize_roi, left_edge_roi_rect_vec[0], left_projection_mat, T_camera_to_plane_vec[0], T_plane_to_trunk);
        return true;
    }
    void visualizeTrunk(cv::Mat& roi, cv::Rect roi_rect, const Eigen::Matrix<double,3,4> projection_mat, 
                        const Eigen::Matrix4d& T_camera_to_plane, const Eigen::Matrix4d& T_plane_to_trunk){
        Eigen::Vector4d points[4];
        double length = 20.0;
        points[0] << length/2.0, -length/2.0,0, 1.0;
        points[1] << length/2.0, length/2.0,0, 1.0;
        points[2] << -length/2.0, length/2.0,0, 1.0;
        points[3] << -length/2.0, -length/2.0,0, 1.0;

        std::vector<cv::Point2f> points_in_image(4);
        for(unsigned int i=0; i<4; i++){
            Eigen::Vector3d point_in_image_eigen = projection_mat * T_camera_to_plane * T_plane_to_trunk * points[i];
            normalizeVector3d(point_in_image_eigen);
            points_in_image[i].x = point_in_image_eigen(0) - roi_rect.x;
            points_in_image[i].y = point_in_image_eigen(1) - roi_rect.y;
        }
        drawLines(roi, points_in_image, 128,128,0);
    }
};

#endif