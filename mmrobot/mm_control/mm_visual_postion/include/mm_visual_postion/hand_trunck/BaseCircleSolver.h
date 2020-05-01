#ifndef __BASE_CIRCLE_SOLVER_H__
#define __BASE_CIRCLE_SOLVER_H__
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include <bitset>
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/hand_trunck/CircleModel.h"
class FittedEllipse{
public:
    int id;
    cv::RotatedRect box;
    std::shared_ptr<std::vector<cv::Point> > valid_contours_ptr;
    std::bitset<12> cover_area; //divide an ellipse into 12 components (30 degrees per each), 
                        // if the contour covers some components, then the corresponding cover_area will be true, otherwise false
    
    std::vector<int> possible_other_parts_id;
    double score;
    FittedEllipse(int ellipse_id, cv::RotatedRect& ellipse_box)
    :id(ellipse_id), box(ellipse_box)
    {
        valid_contours_ptr = std::make_shared<std::vector<cv::Point> >();
    }
    void cover(const std::vector<cv::Point>& points){
        for(unsigned int i=0; i<points.size(); i++){
            double angle = std::atan2(points[i].y-box.center.y, points[i].x-box.center.x) + M_PI; // angle is between 0 to 2*pi
            cover_area[ (int)(angle/ M_PI * 6)] = 1;
        }
    }
    bool hasOverlap(const FittedEllipse& another_ellipse) const{
        std::bitset<12> overlap_area = cover_area & another_ellipse.cover_area;
        if(cover_area.count() == 12 || another_ellipse.cover_area.count()==12) return true;
        if(cover_area.count() + another_ellipse.cover_area.count() == 13 ) return false;
        if(overlap_area.count() > 1) return true; // accept that both two contours have points in at most one interval of angle area.
                                                  // or two intervals if they form completely an ellipse
        return false;
    }
};
class EllipseWithScore{
public:
    cv::RotatedRect box; 
    double score;
    EllipseWithScore(){};
    EllipseWithScore(cv::RotatedRect ellipse_box, double ellipse_score){
        box = ellipse_box;
        score = ellipse_score;
    }
};
typedef std::vector<cv::Point> VecPoint;
typedef std::shared_ptr<VecPoint> VecPointPtr;

class StereoCircleValidContours{
public:
    VecPointPtr left, right;
};
class BaseCircleSolver{
protected:

    std::shared_ptr<StereoCameraArmModel> stereo_cam_ptr;

    static void inline getEllipseParams(const cv::RotatedRect& ellipse_cv_form, double& a, double&b ,double &x_c, double &y_c, double& sin_theta, double& cos_theta){
        a = ellipse_cv_form.size.width /2.0;
        b = ellipse_cv_form.size.height /2.0;
        x_c = ellipse_cv_form.center.x;
        y_c = ellipse_cv_form.center.y;
        sin_theta = std::sin(ellipse_cv_form.angle * M_PI / 180.0); // rotatedrect's angle is presented in degree
        cos_theta = std::cos(ellipse_cv_form.angle * M_PI / 180.0);
    }
    double getAlgebraDistance(const cv::RotatedRect& ellipse, double x, double y){
        double a, b, x_c, y_c, sin_theta, cos_theta;
        getEllipseParams(ellipse, a, b, x_c, y_c, sin_theta, cos_theta);
        double equation = -1.0 + std::pow(-(x-x_c)*sin_theta +(y-y_c)*cos_theta, 2)/b/b 
                            + std::pow((x-x_c)*cos_theta + (y-y_c)*sin_theta, 2)/a/a;
        return fabs(equation);
        
    }

    double getAlgebraDistance(const Eigen::Matrix3d & ellipse_quad_form, double x, double y){
        Eigen::Vector3d X;
        X << x,y,1.0;
        return fabs(X.transpose() * ellipse_quad_form * X);
    }
    template <class T>
    double getAlgebraDistance(const T & ellipse, std::vector<cv::Point>& points){
        double distance = 0;
        for(unsigned int i=0; i<points.size();i++){
            distance += pow(getAlgebraDistance(ellipse, points[i].x, points[i].y),2);
        }
        return distance;
    }

    
    double getNormalizedAlgebraDistance(const cv::RotatedRect & ellipse, std::vector<cv::Point>& points, const double good_fit_normalized_threshold,  std::vector<cv::Point>& good_fit_points){
        double total_distance = 0;
        double normalize_factor = 1.0/pow((ellipse.size.width + ellipse.size.height)/4.0 ,4); // quasi-normalized the distance
        for(unsigned int i=0; i<points.size();i++){
            double curr_distance = pow(getAlgebraDistance(ellipse, points[i].x, points[i].y),2) * normalize_factor;
            if(curr_distance < good_fit_normalized_threshold){
                good_fit_points.push_back(points[i]);
            }
            total_distance += curr_distance;

        }
        return total_distance;
    }
    

    double getNormalizedAlgebraDistance(const cv::RotatedRect & ellipse, std::vector<cv::Point>& points){
        double distance = getAlgebraDistance(ellipse, points);
        distance /= pow((ellipse.size.width + ellipse.size.height)/4.0, 4); // quasi-normalized the distance
        return distance;
    }

    
    




public:
    static void translateEllipse(const cv::RotatedRect &ellipse_cv_form, Eigen::Matrix3d &ellipse_quad_form)
    {
        double a, b, x_c, y_c, sin_theta, cos_theta;
        getEllipseParams(ellipse_cv_form, a, b, x_c, y_c, sin_theta, cos_theta);

        double A = (a*a*sin_theta*sin_theta + b*b*cos_theta*cos_theta)/(a*a*b*b);
        double B = (-2*a*a*sin_theta*cos_theta + 2*b*b*sin_theta*cos_theta)/(a*a*b*b);
        double C = (a*a*cos_theta*cos_theta + b*b*sin_theta*sin_theta)/(a*a*b*b);
        double D = (-2*a*a*x_c*sin_theta*sin_theta + 2*a*a*y_c*sin_theta*cos_theta - 2*b*b*x_c*cos_theta*cos_theta - 2*b*b*y_c*sin_theta*cos_theta)/(a*a*b*b);
        double E = (2*a*a*x_c*sin_theta*cos_theta - 2*a*a*y_c*cos_theta*cos_theta - 2*b*b*x_c*sin_theta*cos_theta - 2*b*b*y_c*sin_theta*sin_theta)/(a*a*b*b);
        double F = (-a*a*b*b + a*a*x_c*x_c*sin_theta*sin_theta - 2*a*a*x_c*y_c*sin_theta*cos_theta + a*a*y_c*y_c*cos_theta*cos_theta + b*b*x_c*x_c*cos_theta*cos_theta + 2*b*b*x_c*y_c*sin_theta*cos_theta + b*b*y_c*y_c*sin_theta*sin_theta)/(a*a*b*b);

        ellipse_quad_form << A/F, B / 2.0/F, D / 2.0/F, B / 2.0/F, C/F, E / 2.0/F, D / 2.0/F, E / 2.0/F, 1.0;
        /*
        //verify the translation
        double alpha = 0.0;

        for(; alpha < 2*M_PI; alpha += 0.1){// in rad
            double x = a * cos(alpha) * cos_theta - b * sin(alpha) * sin_theta + x_c;
            double y = a * cos(alpha) * sin_theta + b * sin(alpha) * cos_theta + y_c;
            Eigen::Vector3d X;
            X << x,y,1;
            double result = X.transpose() * ellipse_quad_form * X;
            std::cout<<"verification result: "<<result<<std::endl;
        }
        */
    }
    BaseCircleSolver(std::shared_ptr<StereoCameraArmModel>& stereo_cam_ptr_):stereo_cam_ptr(stereo_cam_ptr_){};
    void reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const Circle3D& circle, int camera_id, int sample_size, const cv::Scalar& color);
    void reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const std::vector<Circle3D>& circles_vec, int camera_id, int sample_size, const cv::Scalar& color);
    void reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const ConcentricCircles3D &concentric_circles, int camera_id, int sample_size, const cv::Scalar &color);
    void reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const std::vector<ConcentricCircles3D> &concentric_circles, int camera_id, int sample_size, const cv::Scalar &color);

};
void BaseCircleSolver::reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const Circle3D &circle, int camera_id, int sample_size, const cv::Scalar &color)
{
    /** reproject a circle in 3D into the image (left/right, indicated by the camera_id)
     *  the circle is discrete by uniformly sampled points, and the 3D points are projected into the image.
     * @ param image : the image needed to be shown, it should be in BGR, attention: this function will change this image.
     * @ param circle : the circle needed to be projected.
     * @ camera_id : it could be LEFT_CAMERA or RIGHT_CAMERA
     * @ sample_size : the number of sampling points.
     * @ color : the color of sampling points which are shown in the image.
    */
    double increment_angle = 2 * M_PI / sample_size;
    Eigen::Matrix4d transform_camera_to_circle;
    Eigen::Vector3d pixel_pos;

    Eigen::Matrix<double, 3, 4> projection_matrix;

    switch (camera_id)
    {
    case LEFT_CAMERA:
    {
        projection_matrix = stereo_cam_ptr->left_camera.projection_mat;
        break;
    }
    case RIGHT_CAMERA:
    {
        projection_matrix = stereo_cam_ptr->right_camera.projection_mat;
        break;
    }
    default:
        assert("camera id should be only one of the LEFT_CAMERA or RIGHT_CAMERA");
        break;
    }

    circle.getTransformOriginToCircle(transform_camera_to_circle);
    cv::Vec3b color_vec3b;
    color_vec3b[0] = color[0];
    color_vec3b[1] = color[1];
    color_vec3b[2] = color[2];
    
    for (int i = 0; i < sample_size; i++)
    {
        Eigen::Vector4d point_in_circle_coord; // the point on the circle, which is presented in the circle coordinate. (origin is the circle's center)
        point_in_circle_coord(0) = circle.radius * std::cos(increment_angle * i);
        point_in_circle_coord(1) = circle.radius * std::sin(increment_angle * i);
        point_in_circle_coord(2) = 0.0;
        point_in_circle_coord(3) = 1.0;

        // transform point in circle coordinate into camera's coordinate.
        Eigen::Vector4d point_in_camera_coord = transform_camera_to_circle * point_in_circle_coord;
        pixel_pos = projection_matrix * point_in_camera_coord;

        // normalize pixel pos
        pixel_pos(0) = pixel_pos(0) / pixel_pos(2);
        pixel_pos(1) = pixel_pos(1) / pixel_pos(2);

        // draw points on the image
        cv::Point point((int)pixel_pos(0) - rect_roi.x, (int)pixel_pos(1) - rect_roi.y);    
        //cv::circle(image,point,2,color,1);
        if(point.x < image_roi.cols && point.x > 0 && point.y < image_roi.rows && point.y > 0)
            image_roi.at<cv::Vec3b>(point) = color_vec3b;

    }
}

void BaseCircleSolver::reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const std::vector<Circle3D> &circles_vec, int camera_id, int sample_size, const cv::Scalar &color)
{
    for (unsigned int i = 0; i < circles_vec.size(); i++)
    {
        reprojectCircles(image_roi, rect_roi, circles_vec[i], camera_id, sample_size, color);
    }
}

void BaseCircleSolver::reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const ConcentricCircles3D &concentric_circles, int camera_id, int sample_size, const cv::Scalar &color){
    Circle3D circles[2];
    concentric_circles.splitToCircles(circles);
    reprojectCircles(image_roi, rect_roi, circles[0], camera_id, sample_size, color);
    reprojectCircles(image_roi, rect_roi, circles[1], camera_id, sample_size, color);
}
void BaseCircleSolver::reprojectCircles(cv::Mat &image_roi, cv::Rect& rect_roi, const std::vector<ConcentricCircles3D> &concentric_circles, int camera_id, int sample_size, const cv::Scalar &color)
{
    for (unsigned int i = 0; i < concentric_circles.size(); i++)
    {
        reprojectCircles(image_roi, rect_roi, concentric_circles[i], camera_id, sample_size, color);
    }
}


#endif