#ifndef __ROUGH_CIRCLE_SOLVER_H__
#define __ROUGH_CIRCLE_SOLVER_H__

/*
*   This class implements the algorithm that decribed in the paper
*    "Long Quan. Conic Reconstruction and Correspondence from Two Views",
*   which gives an analytical solution of the circle pose in space. 
*
*   Author: Chuan QIN 
*/
#include "CircleModel.h"
#include "BaseCircleSolver.h"
#include <vector>
#include <limits>
#include <tuple>
#include <utility> //std::pair
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "mm_visual_postion/hand_trunck/ransac_ellipse2d.h"
#define DEBUG

class RoughCircleSolver : public BaseCircleSolver
{
private:
public:
    RoughCircleSolver(std::shared_ptr<StereoCameraArmModel> &stereo_cam_ptr_) : BaseCircleSolver(stereo_cam_ptr_)
    {
    }

    void getPossibleCircles(const cv::Mat &left_edge_roi, const cv::Mat &right_edge_roi,
                                           const cv::Rect &left_rect_roi, const cv::Rect& right_rect_roi,
                                           std::vector<Circle3D> &circles_3d,
                                           std::vector<StereoCircleValidContours> &stereo_valid_contours_in_roi_vec,
                                           std::vector<EllipseWithScore>& valid_left_ellipses_box, 
                                           std::vector<EllipseWithScore>& valid_right_ellipses_box,
                                           const double estimate_radius,
                                           const double radius_threshold);
// main function of this class

private:
    void getPossibleEllipse(const cv::Mat &edge_input, std::vector<EllipseWithScore> &ellipses_vec,
                                           std::vector<VecPointPtr> &valid_circle_contours_vec); //helper function
    bool computeCircle3D(const cv::RotatedRect &left_ellipse_box, const cv::RotatedRect &right_ellipse_box, Circle3D &circle);                                                                     //helper function
    typedef std::tuple<int, int, double> CirclePair;                                                                                                                                               // left index, right index, error
    void computeI2I3I4(const Eigen::Matrix4d &A, const Eigen::Matrix4d &B, double &I2, double &I3, double &I4);
    void computeI2I3I4Analytic(const Eigen::Matrix4d &A, const Eigen::Matrix4d &B, double &I_2, double &I_3, double &I_4);

    void computePointsInPlane(const double &u, const double &v, const Eigen::Vector4d &plane, Eigen::Vector4d &point, int camera_id);

    bool areConcentric(const Circle3D a, const Circle3D b, double center_threshold);
    void get3DCircles(const std::vector<EllipseWithScore> &left_possible_ellipses,
                                             const std::vector<EllipseWithScore> &right_possible_ellipses,
                                            const cv::Rect &left_rect_roi, const cv::Rect& right_rect_roi,
                                             std::vector<Circle3D> &circle3d_vec,
                                             std::vector<std::pair<int, int>> & valid_paired_index_vec,
                                             const cv::Mat *left_edge_ptr,
                                             const cv::Mat *right_edge_ptr,
                                             const double estimate_radius,
                                             const double radius_threshold);
    bool equal(const cv::RotatedRect &box_0, const cv::RotatedRect &box_1, const double center_threshold, const double size_threshold, const double angle_threshold)
    {
        if (fabs(box_0.center.x - box_1.center.x) > center_threshold)
            return false;
        if (fabs(box_0.center.y - box_1.center.y) > center_threshold)
            return false;
        if (fabs(box_0.size.width - box_1.size.width) > size_threshold)
            return false;
        if (fabs(box_0.size.height - box_1.size.height) > size_threshold)
            return false;
        if (fabs(box_0.angle - box_1.angle) > angle_threshold)
            return false;
        return true;
    }
};
template <class T>
bool isIn(T num, T min, T max)
{
    if (num >= min && num <= max)
        return true;
    else
        return false;
}

#endif //__ROUGH_CIRCLE_SOLVER_H__