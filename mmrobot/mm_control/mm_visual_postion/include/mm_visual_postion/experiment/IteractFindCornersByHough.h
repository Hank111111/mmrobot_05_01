#ifndef __ITERACTFINDCORNERS_BYHOUGH_H__
#define __ITERACTFINDCORNERS_BYHOUGH_H__

#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include "ros/ros.h"
#include "mm_visual_postion/utils/hough_v3_4_5.h"
#include "mm_visual_postion/switch/SwitchFinderByHough.h"
class IteractFindCornersByHough{
private:
    std::vector<cv::Point2f> corners;
    bool finished;
    cv::Mat show_image;
    cv::Mat edge;
    cv::Mat roi_edge_image_merge, edge_image_merge;
    const cv::Mat origin_image;
    cv::Mat roi_image;
    std::string window_name;
    std::string roi_window_name;
    bool drag_mode; //drag mode
    cv::Point2f left_up_point, right_down_point;
    int drag_point_index;
    std::vector<cv::Vec3f> filterd_up_lines, filterd_left_lines, filterd_right_lines, filterd_down_lines;
    cv::Rect up_roi_rect, down_roi_rect, left_roi_rect, right_roi_rect;
    int selected_left_line_index = -1;
    int selected_right_line_index = -1;
    int selected_up_line_index = -1;
    int selected_down_line_index = -1;
public:
    IteractFindCornersByHough(const cv::Mat& image): origin_image(image){
        finished = false;
        drag_mode = false;
        cv::Canny(origin_image, edge, 500, 1500, 5);
        cv::cvtColor(edge, edge, CV_GRAY2BGR);
        cv::addWeighted(edge, 0.5, image, 0.5, 0, edge_image_merge);

        show_image = edge.clone();
        window_name = "manually find corners";
        roi_window_name = "roi select lines";

    }

    void start(){
        cv::namedWindow(window_name,0);
        cv::imshow(window_name,show_image);
        cv::setMouseCallback(window_name, IteractFindCornersByHough::onMouse, this);
        while(ros::ok() && !finished){
            cv::waitKey(100);
        }
        cv::destroyWindow(window_name);


        finished = false;
        cv::namedWindow(roi_window_name,0);
        cv::setMouseCallback(roi_window_name, IteractFindCornersByHough::onMouseROI, this);
        roiProcess();
        while(ros::ok() && !finished){
            cv::waitKey(100);
        }
        cv::destroyWindow(roi_window_name);
    }

    int roiProcess(){
        // return -1 if not found
        cv::Rect roi(left_up_point, right_down_point);
        roi_image = origin_image(roi).clone();
        cv::Mat roi_edge = edge(roi).clone();
        cv::addWeighted(roi_edge, 0.5, roi_image, 0.5, 0, roi_edge_image_merge);

        cv::cvtColor(roi_edge, roi_edge, CV_BGR2GRAY);

        cv::Mat roi_gray;
        cv::cvtColor(roi_image, roi_gray, COLOR_BGR2GRAY);
        // try to use haar filter to find some interesting things
        cv::Mat lb_rw(cv::Size(3,1), CV_32F); //left black right white kernel
        lb_rw.at<float>(0,0) = -0.5;
        lb_rw.at<float>(0,1) = 0.0;
        lb_rw.at<float>(0,2) = 0.5;

        cv::Mat ub_dw(cv::Size(1,3), CV_32F); // up black down white kernel
        ub_dw.at<float>(0,0) = -0.5;
        ub_dw.at<float>(1,0) = 0.0;
        ub_dw.at<float>(2,0) = 0.5;

        cv::Mat roi_edge_lb_rw, roi_edge_lw_rb, roi_edge_ub_dw, roi_edge_uw_db;
        filter2D(roi_gray, roi_edge_lb_rw, -1, lb_rw);
        filter2D(roi_gray, roi_edge_lw_rb, -1, -lb_rw);

        filter2D(roi_gray, roi_edge_ub_dw, -1, ub_dw);
        filter2D(roi_gray, roi_edge_uw_db, -1, -ub_dw);


        // firstly find roughly center
        cv::Mat threshold_img;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        adaptiveThreshold(roi_gray, threshold_img, 255, cv::THRESH_BINARY_INV, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 11, 4);
        cv::findContours(threshold_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
        cv::Mat draw_img;
        
        std::vector<cv::Point2f> center;
        std::vector<float> rect_size;
        for (unsigned int i = 0; i < contours.size(); i++) {
            cv::Mat hull;

            cv::convexHull(contours[i], hull);

            if (cv::contourArea(hull) > 1500) {
                //drawContours(draw_img, contours, i, cv::Scalar(255, 255, 128), 1, 8);

                cv::RotatedRect rect = cv::minAreaRect(hull);
                float ratio = rect.size.height / rect.size.width;
                if (ratio < 1.3 && ratio > 0.7) {
                    center.push_back(rect.center);
                    rect_size.push_back((rect.size.width + rect.size.height) / 2.0);
                    
                }
            }

        }
        float center_x = 0;
        float center_y = 0;
        float radius = 0;
        if (center.size() <= 0) return -1;
        for (unsigned int i = 0; i < center.size(); i++) {
            center_x += center[i].x;
            center_y += center[i].y;
            radius += rect_size[i];
        }
        center_x = center_x / center.size();
        center_y = center_y / center.size();
        radius = radius / center.size()/2.0;

        // filter roi_edges
        cv::Mat left_roi_edge, right_roi_edge, up_roi_edge, down_roi_edge;
        left_roi_edge = roi_edge.mul(roi_edge_lb_rw);
        right_roi_edge = roi_edge.mul(roi_edge_lw_rb);
        up_roi_edge = roi_edge.mul(roi_edge_ub_dw);
        down_roi_edge = roi_edge.mul(roi_edge_uw_db);
        
        // find left lines
        left_roi_rect = cv::Rect(0, 0, center_x, roi_edge.rows);
        cv::Mat left_roi_img = left_roi_edge(left_roi_rect);
        std::vector<cv::Vec3f> left_lines, left_lines_2;
        cv_v3_4_5::HoughLines(left_roi_img, left_lines, 0.5, M_PI / 180. * 0.1, 30, 0, 0, 0, M_PI / 180. * 15.); //0-15 degree lines
        cv_v3_4_5::HoughLines(left_roi_img, left_lines_2, 0.5, M_PI / 180. * 0.1, 30, 0, 0, M_PI - M_PI / 180.0 * 15.0, M_PI); //165-180 degree

        if (left_lines.empty() && left_lines_2.empty()) {
            return -1;
        }
        if(!left_lines_2.empty()){
            left_lines.insert(std::end(left_lines), std::begin(left_lines_2), std::end(left_lines_2));
            std::sort(left_lines.begin(), left_lines.end(), lineScoreGreater);
        }

        linesNMS(left_lines, filterd_left_lines,3);
        


        // find right lines
        right_roi_rect = cv::Rect(center_x, 0, roi_edge.cols - center_x, roi_edge.rows);
        cv::Mat right_roi_img = right_roi_edge(right_roi_rect);
        std::vector<cv::Vec3f> right_lines, right_lines_2;
        cv_v3_4_5::HoughLines(right_roi_img, right_lines, 0.5, M_PI / 180. * 0.1, 30, 0, 0, 0, M_PI / 180. * 15.); //0-15 degree lines
        cv_v3_4_5::HoughLines(right_roi_img, right_lines_2, 0.5, M_PI / 180. * 0.1, 30, 0, 0, M_PI - M_PI / 180.0 * 15.0, M_PI); //165-180 degree
        //right_lines.insert(std::end(right_lines), std::begin(right_lines_2), std::end(right_lines_2));
        if (right_lines.empty() && right_lines_2.empty() ) {
            return -1;
        }

        if(!right_lines_2.empty()){
            right_lines.insert(std::end(right_lines), std::begin(right_lines_2), std::end(right_lines_2));
            std::sort(right_lines.begin(), right_lines.end(), lineScoreGreater);
        }
        linesNMS(right_lines, filterd_right_lines,3);


        // find up lines
        up_roi_rect = cv::Rect(0,0,roi_edge.cols, center_y);
        cv::Mat up_roi_img = up_roi_edge(up_roi_rect);
        std::vector<cv::Vec3f> up_lines;
        cv_v3_4_5::HoughLines(up_roi_img, up_lines, 0.5, M_PI / 180. * 0.1, 30, 0, 0, M_PI / 2.0 - M_PI / 180. * 15., M_PI / 2.0 + M_PI / 180. * 15.); //75-105 degree
        if (up_lines.empty()) {
            return -1;
        }
        linesNMS(up_lines, filterd_up_lines,3);

        // find down lines
        down_roi_rect = cv::Rect(0,center_y,roi_edge.cols, roi_edge.rows - center_y);
        cv::Mat down_roi_img = down_roi_edge(down_roi_rect);
        std::vector<cv::Vec3f> down_lines;
        cv_v3_4_5::HoughLines(down_roi_img, down_lines, 0.5, M_PI / 180. * 0.1, 30, 0, 0, M_PI / 2.0 - M_PI / 180. * 15., M_PI / 2.0 + M_PI / 180. * 15.); //75-105 degree

        if (down_lines.empty()) {
            return -1;
        }
        linesNMS(down_lines, filterd_down_lines,3);
        cv::Mat roi_image_draw = roi_image.clone();
        drawLines(roi_image_draw(up_roi_rect), filterd_up_lines, 255, 128, 0);
        drawLines(roi_image_draw(down_roi_rect), filterd_down_lines, 255, 128, 0);
        drawLines(roi_image_draw(left_roi_rect), filterd_left_lines, 255, 128, 0);
        drawLines(roi_image_draw(right_roi_rect), filterd_right_lines, 255, 128, 0);
        cv::imshow(roi_window_name,roi_image_draw);
        return 0;
    }
    int clickOnLine(cv::Rect& roi, std::vector<cv::Vec3f>& lines, int x, int y){
        if(x >= roi.x && x <= roi.x + roi.width && y >= roi.y && y <= roi.y + roi.height){
            int x_roi = x - roi.x;
            int y_roi = y - roi.y;
            // line equation rho - x*cos(theta) - y* sin(theta) = 0
            for(unsigned int i=0; i<lines.size(); i++){
                if(fabs(lines[i][0] - x_roi * std::cos(lines[i][1]) - y_roi * std::sin(lines[i][1])) < 1.0){
                    return i;
                }
            }
        }
        return -1;
    }
    void on_Mouse_roi(int events, int x, int y){
        // select lines
        switch(events){
            case CV_EVENT_LBUTTONDBLCLK:{
                if(selected_up_line_index >=0 && selected_down_line_index >=0
                    && selected_left_line_index >=0 && selected_right_line_index >=0){
                     finished = true;
                }
                break;
            }   
            case CV_EVENT_MBUTTONDOWN:{
                int i;
                i = clickOnLine(up_roi_rect, filterd_up_lines, x, y);
                if(i>=0){
                    selected_up_line_index = i;
                    break;
                }
                i = clickOnLine(down_roi_rect, filterd_down_lines, x, y);
                if(i>=0){
                    selected_down_line_index = i;
                    break;
                }
                i = clickOnLine(right_roi_rect, filterd_right_lines, x, y);
                if(i>=0){
                    selected_right_line_index = i;
                    break;
                }
                i = clickOnLine(left_roi_rect, filterd_left_lines, x, y);
                if(i>=0){
                    selected_left_line_index = i;
                    break;
                }
                break;
            }
        }
        cv::Mat roi_image_draw = roi_edge_image_merge.clone();
        drawLines(roi_image_draw(up_roi_rect), filterd_up_lines, 255, 128, 0);
        drawLines(roi_image_draw(down_roi_rect), filterd_down_lines, 255, 128, 0);
        drawLines(roi_image_draw(left_roi_rect), filterd_left_lines, 255, 128, 0);
        drawLines(roi_image_draw(right_roi_rect), filterd_right_lines, 255, 128, 0);
        if(selected_up_line_index >=0){
            drawLine(roi_image_draw(up_roi_rect), filterd_up_lines[selected_up_line_index], 0, 0, 255);
        }
        if(selected_down_line_index >=0){
            drawLine(roi_image_draw(down_roi_rect), filterd_down_lines[selected_down_line_index], 0, 0, 255);
        }
        if(selected_left_line_index >=0){
            drawLine(roi_image_draw(left_roi_rect), filterd_left_lines[selected_left_line_index], 0, 0, 255);
        }
        if(selected_right_line_index >=0){
            drawLine(roi_image_draw(right_roi_rect), filterd_right_lines[selected_right_line_index], 0, 0, 255);
        }
        cv::imshow(roi_window_name,roi_image_draw);
    }

    void getConers(std::vector<cv::Point2f>& corners_output){
        
		cv::Point2f left_up_corner, left_down_corner, right_up_corner, right_down_corner;
		getIntersecPoint(filterd_up_lines[selected_up_line_index], up_roi_rect, 
						filterd_left_lines[selected_left_line_index], left_roi_rect,left_up_corner);
		getIntersecPoint(filterd_up_lines[selected_up_line_index], up_roi_rect, 
						filterd_right_lines[selected_right_line_index], right_roi_rect, right_up_corner);
		getIntersecPoint(filterd_down_lines[selected_down_line_index], down_roi_rect, 
						filterd_right_lines[selected_right_line_index], right_roi_rect, right_down_corner);
		getIntersecPoint(filterd_down_lines[selected_down_line_index], down_roi_rect, 
						filterd_left_lines[selected_left_line_index], left_roi_rect, left_down_corner);
        corners_output.resize(4);
        corners_output[0] = left_up_corner + left_up_point;
        corners_output[1] = right_up_corner + left_up_point;
        corners_output[2] = right_down_corner + left_up_point;
        corners_output[3] = left_down_corner + left_up_point;
    }
    void on_Mouse(int events, int x, int y)
    {
        // select ROI
        switch (events)
        {
            case CV_EVENT_LBUTTONDBLCLK:{
                finished = true;
                break;
            }
            case CV_EVENT_MBUTTONDOWN :{
                // find whether the point can be dragged
                drag_mode = true; //drag mode on
                left_up_point = cv::Point2f(x,y);
                right_down_point = cv::Point2f(x,y);

                break;
            }
            case CV_EVENT_MBUTTONUP:{
                right_down_point = cv::Point2f(x,y);
                drag_mode = false;
                break;
            }
            case CV_EVENT_MOUSEMOVE:{
                if(drag_mode)
                    right_down_point = cv::Point2f(x,y);
                break;
            }
        }
        if(drag_mode){
            // update image
            // draw lines

            show_image = edge_image_merge.clone();
            cv::Mat lines_image = edge_image_merge.clone();
            cv::Point2f roi_left_up_corner = left_up_point;
            cv::Point2f roi_right_up_corner = cv::Point2f(right_down_point.x, left_up_point.y);
            cv::Point2f roi_right_down_corner = right_down_point;
            cv::Point2f roi_left_down_corner = cv::Point2f(left_up_point.x, right_down_point.y);


            cv::line(lines_image, roi_left_up_corner, roi_right_up_corner, cv::Scalar(0, 255, 0), 1);
            cv::line(lines_image, roi_right_up_corner, roi_right_down_corner, cv::Scalar(0, 255, 0), 1);
            cv::line(lines_image, roi_right_down_corner, roi_left_down_corner, cv::Scalar(0, 255, 0), 1);
            cv::line(lines_image, roi_left_up_corner, roi_left_down_corner, cv::Scalar(0, 255, 0), 1);
            cv::addWeighted(show_image, 0.5, lines_image, 0.5 , 0.0, show_image); 
            cv::imshow("manually find corners",show_image);
        }

    }   
    static void onMouse(int events, int x, int y, int, void* userdata){
        // Check for null pointer in userdata and handle the error
	    IteractFindCornersByHough* temp = reinterpret_cast<IteractFindCornersByHough*>(userdata);
    	temp->on_Mouse(events, x, y);
    }

    static void onMouseROI(int events, int x, int y, int, void* userdata){
        // Check for null pointer in userdata and handle the error
	    IteractFindCornersByHough* temp = reinterpret_cast<IteractFindCornersByHough*>(userdata);
    	temp->on_Mouse_roi(events, x, y);
    }
};



#endif