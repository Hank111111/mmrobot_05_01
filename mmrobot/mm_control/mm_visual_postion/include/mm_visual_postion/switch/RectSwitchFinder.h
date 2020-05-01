#ifndef __RECT_SWTICH_FINDER_H__
#define __RECT_SWTICH_FINDER_H__
#include <Eigen/Dense>
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/utils/RigidTransfomSolver.h"
#include "mm_visual_postion/utils/image_utils.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/hough_v3_4_5.h"
#include "mm_visual_postion/switch/LineRefiner.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <memory>
#include <vector>

// given a roughly plane, find all rectangle switches (on the plane) in the images.
class Switch{
public:
    Switch(){
    }
    typedef std::vector<cv::Point> LinePoints;
    double width;
    double height;
    std::string name;
    Eigen::Matrix4d T_cam_to_switch;
    std::vector<cv::Point2d> corner_points_in_roi; // presented in roi
    cv::Rect roi; // store the roi that corner_points presented
    std::vector<Eigen::Vector4d> corner_points_in_3d;
    std::vector<LinePoints> points_of_lines; // edge points of lines(left-up line, left-right line, etc...)
};

typedef std::shared_ptr<Switch> SwitchPtr;


class PairedSwitch{
public:
    SwitchPtr left, right; //switch got from left/right images
};

typedef std::vector<cv::Mat> TemplateImageVec;
class RectSwitchFinder{
private:
    cv::Size2d req_switch_size; // mm
    Eigen::Matrix4d req_rough_T_cam_to_switch;
    std::string req_switch_name;
    
    std::shared_ptr<StereoCameraArmModel> model_ptr;
    cv::Mat left_projection_cv_mat, right_projection_cv_mat;

public:
    TemplateImageVec templates_remote_switch_status_0, templates_remote_switch_status_1;

    //void setRoughSwitchPlane(Eigen::Vector4d plane_params_input); //plane_params^t * [x,y,z,1] = 0
    void setSwitchParamsToRecognize(std::string name, double width, double height, Eigen::Matrix4d& rough_T_cam_to_obj);
    

    void setCameraParams(std::shared_ptr<StereoCameraArmModel> model_ptr_input);

    bool findRectSwitchInOneImage(cv::Mat& roi_img, cv::Mat& roi_gray, cv::Mat& roi_edge, cv::Rect& roi_rect, int camera_id, std::vector<SwitchPtr>& found_switches, bool visualization = false);

    void pairRectInStereoImages(std::vector<SwitchPtr>& found_switches_in_left_img, std::vector<SwitchPtr>& found_switches_in_right_img, std::vector<PairedSwitch>& paired_switches);
    void findAllSwitches(cv::Mat& left_roi_img, cv::Mat& right_roi_img, cv::Rect& left_roi_rect, cv::Rect& right_roi_rect, std::vector<SwitchPtr>& all_switches, bool visualization = false); 
    // main function
    
    void computeRectPosByStereoImages(PairedSwitch& paired_switch, SwitchPtr& switch_in_3d);

    void refineSwitchInOneImage(const cv::Mat& roi_edge, const cv::Mat& roi_gray, cv::Rect& roi_rect, SwitchPtr& switch_ptr);
    void getStatus(const cv::Mat& roi, const Switch& the_switch, int& status, int camera_id);
    bool loadTemplateImages();
    void getCorrectedSwitch(const cv::Mat& image_roi, const Switch& the_switch, cv::Size size, cv::Mat& corrected_switch, int camera_id);
	void getEdge(cv::Mat& gray, cv::Mat& edge);

private:
    int findPossibleSwitchInROI(const cv::Mat& img, const cv::Mat& gray, const cv::Mat& edge, const cv::Rect& roi_rect,
	                                             std::vector<SwitchPtr>& rect_switches,int camera_id, bool visualization);
    bool isSimilarToRect(cv::Size2d& rect_size, std::vector<Eigen::Vector4d>& corners_in_plane);
    
    bool matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat& template_img, double& match_val, cv::Point& match_loc);

    static void readImagesInFolder(std::string folder_name, std::vector<cv::Mat>& images);

    void cvtPointInImageIntoPointInPlane(const cv::Point2f& cv_point_in_roi, const cv::Rect& roi_rect, const Eigen::Matrix4d& rough_T_cam_to_switch, Eigen::Vector4d& point_in_plane, int camera_id);
    void getRotateRect(cv::Mat& edge, std::vector<cv::RotatedRect>& rect_list, bool visualization);
    void getROI(const std::vector<cv::RotatedRect>& rect_list, int img_width, int img_height, std::vector<cv::Rect>& roi_list, bool visualization, const cv::Mat& img);
	void filterROI(std::vector<cv::Rect>& roi_list, std::vector<cv::Rect>& rect_list_filtered, bool visualization, const cv::Mat& img);
    double getRectArea(cv::Rect rect) {
	    return rect.size().width * rect.size().height;
    }
    void uv2cxyzOpencv(const std::vector<cv::Point2d> &uvLeftPoints, const std::vector<cv::Point2d> &uvRightPoints, std::vector<cv::Point3d> &xyzPoints);
};

#endif //__RECT_SWTICH_FINDER_H__