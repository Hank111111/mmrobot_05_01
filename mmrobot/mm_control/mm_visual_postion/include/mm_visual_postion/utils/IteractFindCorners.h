#ifndef __ITERACTFINDCORNERS_H__
#define __ITERACTFINDCORNERS_H__

#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
class IteractFindCorners{
private:
    std::vector<cv::Point2f> corners;
    bool finished;
    cv::Mat show_image;
    cv::Mat edge;
    cv::Mat edge_image_merge;
    const cv::Mat origin_image;
    std::string window_name;
    bool drag_mode; //drag mode
    unsigned int drag_point_index;
public:
    IteractFindCorners(const cv::Mat& image): origin_image(image){
        finished = false;
        drag_mode = false;
        cv::Canny(origin_image, edge, 500, 1500, 5);
        cv::cvtColor(edge, edge, CV_GRAY2BGR);
        cv::addWeighted(edge, 0.5, origin_image, 0.5, 0, edge_image_merge);
        show_image = edge.clone();
        window_name = "manually find corners";
    }

    void start(){
        cv::namedWindow(window_name);
        cv::imshow(window_name,show_image);
        cv::setMouseCallback(window_name, IteractFindCorners::onMouse, this);
        while(!finished){
            cv::waitKey(200);
        }
        cv::destroyWindow(window_name);
    }

    void getCorners(std::vector<cv::Point2f>& corners_output){
        assert(corners.size() == 4);
        cv::Point2f center;
        center.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
        center.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;

        // sort the corners
        bool chosen[4] = {false};
    	int left_up_index, right_up_index, right_down_index, left_down_index;
	// find left up
        for(unsigned int i=0; i<4; i++){
            if(chosen[i])
                continue;
            if(corners[i].x < center.x && corners[i].y < center.y){
                left_up_index = i;
                chosen[i] = true; 
            }
            else if(corners[i].x < center.x && corners[i].y > center.y){
                left_down_index = i;
                chosen[i] = true; 
            }
            else if(corners[i].x > center.x && corners[i].y < center.y){
                right_up_index = i;
                chosen[i] = true; 
            }
            else if(corners[i].x > center.x && corners[i].y > center.y){
                right_down_index = i;
                chosen[i] = true; 
            }
        }
        corners_output.resize(4);
        corners_output[0] = corners[left_up_index];
        corners_output[1] = corners[right_up_index];
        corners_output[2] = corners[right_down_index];
        corners_output[3] = corners[left_down_index];
    }
    void on_Mouse(int events, int x, int y)
    {
        switch (events)
        {
            case CV_EVENT_LBUTTONDOWN:{
                if(corners.size()<4)
                    corners.push_back(cv::Point2f(x,y));
                break;
            }  
            case CV_EVENT_LBUTTONDBLCLK:{
                if(corners.size() == 4){
                    finished = true;
                    break;
                }
            }
            case CV_EVENT_MBUTTONDOWN :{
                // find whether the point can be dragged
                for(unsigned int i=0; i<corners.size(); i++){
                    if( abs(corners[i].x - x) < 5 && abs(corners[i].y - y) < 5){
                        drag_point_index = i;
                        drag_mode = true; //drag mode on
                        break;
                    }
                }
                break;
            }
            case CV_EVENT_MBUTTONUP:{
                drag_mode = false;
            }
            case CV_EVENT_MOUSEMOVE:{
                if(drag_mode && drag_point_index < corners.size() && drag_point_index >= 0 ){
                    // drag point
                    corners[drag_point_index] = cv::Point2f(x,y);
                }
                break;
            }

        }
        if(!finished){
            // update image
            // draw lines

            show_image = edge_image_merge.clone();
            if(corners.size() > 0){
                cv::Mat lines_image = edge_image_merge.clone();
                for(unsigned int i=0; i<corners.size(); i++){
                    if(i+1 < corners.size())
                        cv::line(lines_image, corners[i], corners[i+1], cv::Scalar(0, 255, 0), 1);
                    lines_image.at<cv::Vec3b>(corners[i]) = cv::Vec3b(255,0,0);
                }
                if(corners.size() == 4){
                    cv::line(lines_image, corners[corners.size()-1], corners[0], cv::Scalar(0, 255, 0), 1);
                }
                cv::addWeighted(show_image, 0.5, lines_image, 0.5 , 0.0, show_image); 
            
            }
            cv::imshow("manually find corners",show_image);
        }

    }   
    static void onMouse(int events, int x, int y, int, void* userdata){
        // Check for null pointer in userdata and handle the error
	    IteractFindCorners* temp = reinterpret_cast<IteractFindCorners*>(userdata);
    	temp->on_Mouse(events, x, y);
    }
};



#endif