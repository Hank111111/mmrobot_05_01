#include "mm_visual_postion/switch/RectSwitchFinder.h"



void RectSwitchFinder::setSwitchParamsToRecognize(std::string name, double width, double height, Eigen::Matrix4d& rough_T_cam_to_obj){
    req_switch_size = cv::Size2d(width, height);
	req_rough_T_cam_to_switch = rough_T_cam_to_obj;
	Eigen::Matrix4d T_compare = req_rough_T_cam_to_switch * req_rough_T_cam_to_switch.inverse();
	assert(isTransfromSimilaireToIdentity(T_compare, 1e-8, 1e-5) && "The transform matrix rough_T_cam_to_obj is not valid");
	req_switch_name = name;
}

void RectSwitchFinder::setCameraParams(std::shared_ptr<StereoCameraArmModel> model_ptr_input){
    model_ptr = model_ptr_input;
    eigen2cv(model_ptr->left_camera.projection_mat, left_projection_cv_mat);
    eigen2cv(model_ptr->right_camera.projection_mat, right_projection_cv_mat);
}
void RectSwitchFinder::getEdge(cv::Mat& gray, cv::Mat& edge) {
	cv::Canny(gray, edge, 500, 1500, 5);
}

void RectSwitchFinder::getRotateRect(cv::Mat& edge, std::vector<cv::RotatedRect>& rect_list, bool visualization) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<cv::Point> approx;
	std::vector<cv::Point> approx_hull;
	std::vector<cv::Point> hull;
	const double min_area = 463;
	const double approxDP_epsilon = 0.38;
	const double rect_epsilon = 0.3;
	cv::findContours(edge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	cv::Mat draw_img;
	if (visualization) {
		cv::cvtColor(edge, draw_img, cv::COLOR_GRAY2BGR);
	}
	
	for (unsigned int i = 0; i < contours.size(); i++) {
		hull.clear();
		approx.clear();
		approx_hull.clear();
		cv::convexHull(contours[i], hull);

		if (cv::contourArea(hull) > min_area) {
			//drawContours(draw_img, contours, i, cv::Scalar(255, 255, 128), 1, 8);

			cv::approxPolyDP(contours[i], approx, approxDP_epsilon, true);
			cv::convexHull(approx, approx_hull);
			cv::RotatedRect rect = cv::minAreaRect(approx_hull);
			double ratio = rect.size.width/rect.size.height;
			if (ratio < 1.0 + rect_epsilon && ratio > 1.0-rect_epsilon) {
				//verify it's roughly a square(width should be equal to height)
				rect_list.push_back(rect);
				if (visualization) {
					cv::Point2f vertices[4];
					rect.points(vertices);
					for (unsigned int j = 0; j < 4; j++)
						line(draw_img, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
				}
			}

		}

	}
	if (visualization) {
		cv::imshow("rotate rect", draw_img);
		cv::waitKey(0);
	}
}


void RectSwitchFinder::getROI(const std::vector<cv::RotatedRect>& rect_list, int img_width, int img_height, std::vector<cv::Rect>& roi_list, bool visualization, const cv::Mat& img) {
	const int padding = 10;
	cv::Mat draw_img;
	if (visualization) {
		draw_img = img.clone();
	}
	for (unsigned int i = 0; i < rect_list.size(); i++) {

		int max_length = int(fmax(rect_list[i].size.width, rect_list[i].size.height));


		//check if roi is out of boundary
		int left = int(rect_list[i].center.x) - max_length + padding;
		int right = int(rect_list[i].center.x) + max_length + padding;
		int up = int(rect_list[i].center.y) - max_length + padding;
		int down = int(rect_list[i].center.y) + max_length + padding;

		left = clip(left, 0, img_width);
		right = clip(right, 0, img_width);
		up = clip(up, 0, img_height);
		down = clip(down, 0, img_height);

		if (right - left > 0 && down - up > 0) {
			cv::Rect roi(left, up, right - left, down - up);
			roi_list.push_back(roi);
		}
		if(visualization)
			cv::rectangle(draw_img, cv::Point2d(left,up), cv::Point2d(right, down), cv::Scalar(0, 255, 0));
	}
	if (visualization) {
		cv::imshow("ROI", draw_img);
		cv::waitKey(0);
	}
}
void RectSwitchFinder::filterROI(std::vector<cv::Rect>& rect_list, std::vector<cv::Rect>& rect_list_filtered, bool visualization,const cv::Mat& img) {
	std::vector<bool> mask(rect_list.size(), true);
	cv::Mat draw_img;
	if (visualization) {
		draw_img = img.clone();
	}
	for (unsigned int i = 0; i < rect_list.size(); i++) {
		bool is_fused = false;
		for (unsigned int j = 0; j < rect_list_filtered.size(); j++) {
			cv::Rect intersect_rect = rect_list[i] & rect_list_filtered[j];
			double overlap_area = getRectArea(intersect_rect);
			if (overlap_area / getRectArea(rect_list[i]) > 0.8 || overlap_area / getRectArea(rect_list_filtered[j]) > 0.8) {
				rect_list_filtered[j] = rect_list[i] | rect_list_filtered[j];
				is_fused = true;
				break;
			}
		}
		if (!is_fused) {
			rect_list_filtered.push_back(rect_list[i]);
		}
	}
	if (visualization) {
		for (unsigned int i = 0; i < rect_list_filtered.size(); i++) {
			cv::rectangle(draw_img, rect_list_filtered[i], cv::Scalar(0, 255, 0));
		}
		cv::imshow("ROI filterd", draw_img);
		cv::waitKey(0);
	}
}
void RectSwitchFinder::cvtPointInImageIntoPointInPlane(const cv::Point2f& cv_point_in_roi, const cv::Rect& roi_rect, const Eigen::Matrix4d& rough_T_cam_to_switch, Eigen::Vector4d& point_in_plane, int camera_id){
    // solve: [u,v,1]^t = p * [x*w, y*w, z*w, w]^t
    //        0 = plane_params^t * [x, y, z, 1]^t
    Eigen::Matrix4d M;
    if(camera_id == LEFT_CAMERA)
        M.block<3,4>(0,0) = model_ptr->left_camera.projection_mat;
    else if(camera_id == RIGHT_CAMERA)
        M.block<3,4>(0,0) = model_ptr->right_camera.projection_mat;
	Eigen::Vector4d center, plane_params;
	getPlane(rough_T_cam_to_switch, center, plane_params);
    M.block<1,4>(3,0) = plane_params.transpose();
    Eigen::Matrix4d M_inv = M.inverse();

    Eigen::Vector4d point_in_image;
    point_in_image << cv_point_in_roi.x + roi_rect.x , cv_point_in_roi.y + roi_rect.y, 1, 0;
    point_in_plane = M_inv * point_in_image;
    point_in_plane(0) /= point_in_plane(3);
    point_in_plane(1) /= point_in_plane(3);
    point_in_plane(2) /= point_in_plane(3);
    point_in_plane(3) = 1.0;
}
bool RectSwitchFinder::isSimilarToRect(cv::Size2d& rect_size, std::vector<Eigen::Vector4d>& corners_in_plane){
    // check whether the corners form a rectangle (90 degree)
    assert(corners_in_plane.size() == 4);
    for(unsigned int i=0; i<4; i++){
        Eigen::Vector3d line_0 = corners_in_plane[(i)%4].head<3>() - corners_in_plane[(i-1+4)%4].head<3>();
        Eigen::Vector3d line_1 = corners_in_plane[(i+1)%4].head<3>() - corners_in_plane[(i)%4].head<3>();
        double cos_angle = line_0.dot(line_1) / (line_0.norm() * line_1.norm());
        if(fabs(cos_angle) > 0.3){
			ROS_INFO("switch candidator in image is not square, cos_angle = %f", cos_angle);
			return false; // 90 degree

		} 
    }
    // check whether the rectangle's size is correct
    double width = (corners_in_plane[1] - corners_in_plane[0]).head<3>().norm();
    double height = (corners_in_plane[2] - corners_in_plane[1]).head<3>().norm();
    if(fabs(width - rect_size.width) > 5){
		ROS_INFO("switch candidator in image's width is [%f], prior information in database indicates [%f]", width, rect_size.width);
		return false;
	} 
    if(fabs(height - rect_size.height) > 5){
		ROS_INFO("switch candidator in image's height is [%f], prior information in database indicates [%f]", height, rect_size.height);
		return false;
	}
	ROS_INFO("sucessfully find one square switch candidator in image with correct width and height");
    return true;
}


int RectSwitchFinder::findPossibleSwitchInROI(const cv::Mat& img, const cv::Mat& gray, const cv::Mat& edge, const cv::Rect& roi_rect,
	    std::vector<SwitchPtr>& rect_switches, int camera_id, bool visualization)
{
	// return -1 if not found
	
	// try to use haar filter to find some interesting things
	cv::Mat lb_rw(cv::Size(3,1), CV_32F); //left black right white kernel
	lb_rw.at<float>(0,0) = -0.5;
	lb_rw.at<float>(0,1) = 0.0;
	lb_rw.at<float>(0,2) = 0.5;

	cv::Mat ub_dw(cv::Size(1,3), CV_32F); // up black down white kernel
	ub_dw.at<float>(0,0) = -0.5;
	ub_dw.at<float>(1,0) = 0.0;
	ub_dw.at<float>(2,0) = 0.5;

	cv::Mat edge_lb_rw, edge_lw_rb, edge_ub_dw, edge_uw_db;
	filter2D(gray, edge_lb_rw, -1, lb_rw);
	filter2D(gray, edge_lw_rb, -1, -lb_rw);

	filter2D(gray, edge_ub_dw, -1, ub_dw);
	filter2D(gray, edge_uw_db, -1, -ub_dw);

	/*
	// firstly find roughly center
	cv::Mat threshold_img;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	adaptiveThreshold(gray, threshold_img, 255, cv::THRESH_BINARY_INV, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 11, 4);
	cv::findContours(threshold_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	cv::Mat draw_img;
	if (visualization) {
		cv::cvtColor(threshold_img, draw_img, cv::COLOR_GRAY2BGR);
	}

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
				if (visualization) {
					cv::Point2f vertices[4];
					rect.points(vertices);
					for (int i = 0; i < 4; i++)
						line(draw_img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
				}
			}
		}

	}
	if (visualization) {
		cv::imshow("roughly position", draw_img);
	}
	*/
	float center_x = 0;
	float center_y = 0;
  	//if (center.size() <= 0){
	center_x = gray.cols/2.0;
	center_y = gray.rows/2.0;	
	/*
	}
	else{
		for (unsigned int i = 0; i < center.size(); i++) {
			center_x += center[i].x;
			center_y += center[i].y;
		}
		center_x = center_x / center.size();
		center_y = center_y / center.size();
	}
	 */
	// filter edges
	cv::Mat left_edge, right_edge, up_edge, down_edge;
	left_edge = edge.mul(edge_lb_rw);
	right_edge = edge.mul(edge_lw_rb);
	up_edge = edge.mul(edge_ub_dw);
	down_edge = edge.mul(edge_uw_db);
	/*
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 3, 3 ));
	dilate( left_edge, left_edge, element );
	dilate( right_edge, right_edge, element );
	dilate( up_edge, up_edge, element );
	dilate( down_edge, down_edge, element );
	*/

	// find left lines
	cv::Rect left_roi_rect(0, 0, center_x, edge.rows);
	cv::Mat left_roi_img = left_edge(left_roi_rect);
	std::vector<cv::Vec3f> left_lines, left_lines_2, filterd_left_lines;
	cv_v3_4_5::HoughLines(left_roi_img, left_lines, 1, M_PI / 180. * 0.2, 60, 0, 0, 0, M_PI / 180. * 15.); //0-15 degree lines
	cv_v3_4_5::HoughLines(left_roi_img, left_lines_2, 1, M_PI / 180. * 0.2, 60, 0, 0, M_PI - M_PI / 180.0 * 15.0, M_PI); //165-180 degree

	if (left_lines.empty() && left_lines_2.empty()) {
		return -1;
	}
	if(!left_lines_2.empty()){
		left_lines.insert(std::end(left_lines), std::begin(left_lines_2), std::end(left_lines_2));
		std::sort(left_lines.begin(), left_lines.end(), lineScoreGreater);
	}

	linesNMS(left_lines, filterd_left_lines,3);
	


	// find right lines
	cv::Rect right_roi_rect(center_x, 0, edge.cols - center_x, edge.rows);
	cv::Mat right_roi_img = right_edge(right_roi_rect);
	std::vector<cv::Vec3f> right_lines, right_lines_2, filterd_right_lines;
	cv_v3_4_5::HoughLines(right_roi_img, right_lines, 1, M_PI / 180. * 0.2, 30, 0, 0, 0, M_PI / 180. * 15.); //0-15 degree lines
	cv_v3_4_5::HoughLines(right_roi_img, right_lines_2, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI - M_PI / 180.0 * 15.0, M_PI); //165-180 degree
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
	cv::Rect up_roi_rect(0,0,edge.cols, center_y);
	cv::Mat up_roi_img = up_edge(up_roi_rect);
	std::vector<cv::Vec3f> up_lines, filterd_up_lines;
	cv_v3_4_5::HoughLines(up_roi_img, up_lines, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI / 2.0 - M_PI / 180. * 15., M_PI / 2.0 + M_PI / 180. * 15.); //75-105 degree
	if (up_lines.empty()) {
		return -1;
	}
	linesNMS(up_lines, filterd_up_lines,3);

	// find down lines
	cv::Rect down_roi_rect(0,center_y,edge.cols, edge.rows - center_y);
	cv::Mat down_roi_img = down_edge(down_roi_rect);
	std::vector<cv::Vec3f> down_lines, filterd_down_lines;
	cv_v3_4_5::HoughLines(down_roi_img, down_lines, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI / 2.0 - M_PI / 180. * 15., M_PI / 2.0 + M_PI / 180. * 15.); //75-105 degree

	if (down_lines.empty()) {
		return -1;
	}
	linesNMS(down_lines, filterd_down_lines,3);



	cv::Mat show_img;
	cv::Mat show_img_img, show_img_img_origin;

	int chosen_up_line_index,  chosen_left_line_index, chosen_right_line_index, chosen_down_line_index;
	int case_id = 0;
	SquareLinesIterator line_iterator(filterd_up_lines.size(), filterd_left_lines.size(),filterd_right_lines.size(),filterd_down_lines.size());
    int found_rect_switch_num = 0;
	while(case_id < 16) //maximum 16 case
	{
		case_id ++;
		if(!line_iterator.getIndex(chosen_up_line_index, chosen_left_line_index, chosen_right_line_index, chosen_down_line_index)) break;
		if (visualization) {
			// draw the lines 
			cv::cvtColor(edge, show_img, cv::COLOR_GRAY2BGR);
			show_img_img = show_img.clone();
			show_img_img_origin = show_img.clone();

			drawLines(show_img_img_origin(up_roi_rect), up_lines, 255, 128, 0);
			drawLines(show_img_img_origin(down_roi_rect), down_lines, 255, 128, 0);
			drawLines(show_img_img_origin(left_roi_rect), left_lines, 255, 128, 0);
			drawLines(show_img_img_origin(right_roi_rect), right_lines, 255, 128, 0);
			
			drawLines(show_img_img(up_roi_rect), filterd_up_lines, 255, 128, 0);
			drawLines(show_img_img(down_roi_rect), filterd_down_lines, 255, 128, 0);
			drawLines(show_img_img(left_roi_rect), filterd_left_lines, 255, 128, 0);
			drawLines(show_img_img(right_roi_rect), filterd_right_lines, 255, 128, 0);
			
			drawLine(show_img(up_roi_rect), filterd_up_lines[chosen_up_line_index], 255, 128, 0);
			drawLine(show_img(down_roi_rect), filterd_down_lines[chosen_down_line_index], 255, 128, 0);
			drawLine(show_img(left_roi_rect), filterd_left_lines[chosen_left_line_index], 255, 128, 0);
			drawLine(show_img(right_roi_rect), filterd_right_lines[chosen_right_line_index], 255, 128, 0);
			//cv::imshow("switch position", show_img);
		}

		// find corners
		cv::Point2f left_up_corner, left_down_corner, right_up_corner, right_down_corner;
		getIntersecPoint(filterd_up_lines[chosen_up_line_index], up_roi_rect, 
						filterd_left_lines[chosen_left_line_index], left_roi_rect,left_up_corner);
		getIntersecPoint(filterd_up_lines[chosen_up_line_index], up_roi_rect, 
						filterd_right_lines[chosen_right_line_index], right_roi_rect, right_up_corner);
		getIntersecPoint(filterd_down_lines[chosen_down_line_index], down_roi_rect, 
						filterd_right_lines[chosen_right_line_index], right_roi_rect, right_down_corner);
		getIntersecPoint(filterd_down_lines[chosen_down_line_index], down_roi_rect, 
						filterd_left_lines[chosen_left_line_index], left_roi_rect, left_down_corner);

        std::vector<Eigen::Vector4d> rect_pts_in_plane(4); // left_up, right_up, right_down, left_down;

		// find the most closet switches

	
		cvtPointInImageIntoPointInPlane(left_up_corner, roi_rect, req_rough_T_cam_to_switch, rect_pts_in_plane[0], camera_id);
		cvtPointInImageIntoPointInPlane(right_up_corner, roi_rect, req_rough_T_cam_to_switch, rect_pts_in_plane[1], camera_id);
		cvtPointInImageIntoPointInPlane(right_down_corner, roi_rect, req_rough_T_cam_to_switch, rect_pts_in_plane[2], camera_id);
		cvtPointInImageIntoPointInPlane(left_down_corner, roi_rect, req_rough_T_cam_to_switch, rect_pts_in_plane[3], camera_id);
		
		if(isSimilarToRect(req_switch_size, rect_pts_in_plane)){
			auto switch_ptr = std::make_shared<Switch>();
			switch_ptr->width = req_switch_size.width;
			switch_ptr->height = req_switch_size.height;
			switch_ptr->corner_points_in_roi.resize(4);
 			switch_ptr->corner_points_in_roi[0] = left_up_corner;
			switch_ptr->corner_points_in_roi[1] = right_up_corner;
			switch_ptr->corner_points_in_roi[2] = right_down_corner;
			switch_ptr->corner_points_in_roi[3] = left_down_corner;
			switch_ptr->corner_points_in_3d = rect_pts_in_plane;
			switch_ptr->roi = roi_rect;
			switch_ptr->name = req_switch_name;
			rect_switches.push_back(switch_ptr);
			found_rect_switch_num ++;
		}

	
	}
	if(found_rect_switch_num == 0) return -1;
	return 0;
}

bool RectSwitchFinder::findRectSwitchInOneImage(cv::Mat& roi_img, cv::Mat& roi_gray, cv::Mat& roi_edge, cv::Rect& roi_rect, int camera_id, std::vector<SwitchPtr>& found_switches, bool visualization)
{
    

	//getEdgeAdapdativeThreshold(gray, edge, true);
	std::vector<cv::RotatedRect> rect_list;
	std::vector<cv::Rect> roi_list, filterd_roi_list;
	//getRotateRect(roi_edge, rect_list, visualization);
	//getROI(rect_list, roi_gray.size().width, roi_gray.size().height, roi_list, visualization, roi_img);
	//filterROI(roi_list, filterd_roi_list, visualization, roi_img);
    
           
	if (findPossibleSwitchInROI(roi_img, roi_gray, roi_edge, roi_rect,found_switches, camera_id, visualization) == 0) {
		if (visualization) {
			cv::waitKey(0);
		}
	

	}
	if (found_switches.size() <= 0) {
		return false;
	}
    return true;
}
void RectSwitchFinder::pairRectInStereoImages(std::vector<SwitchPtr>& found_switches_in_left_img, std::vector<SwitchPtr>& found_switches_in_right_img, std::vector<PairedSwitch>& paired_switches){
     std::vector<bool> paired(found_switches_in_right_img.size(), false);
    for(unsigned int i=0; i<found_switches_in_left_img.size(); i++){
        for(unsigned int j=0; j<found_switches_in_right_img.size(); j++){
            if(paired[j]) continue;
            // width and height should be exactly same (the same type of switch)
            if(fabs(found_switches_in_left_img[i]->width - found_switches_in_right_img[j]->width) > 1e-8) continue;
            if(fabs(found_switches_in_left_img[i]->height - found_switches_in_right_img[j]->height) > 1e-8) continue;
			if(fabs(found_switches_in_left_img[i]->name != found_switches_in_right_img[j]->name)) continue;
            // corners in space should be close enough
            bool continue_flag = false;
            for(unsigned int corner_i=0; corner_i < 4; corner_i ++){
                if((found_switches_in_left_img[i]->corner_points_in_3d[corner_i] - 
                    found_switches_in_right_img[j]->corner_points_in_3d[corner_i]).norm() > 5.0){ // 5mm
					continue_flag = true;
                    break;
                }
            }

            if(continue_flag) continue;
			
            PairedSwitch paired_two_switch;
            paired_two_switch.left = found_switches_in_left_img[i];
            paired_two_switch.right = found_switches_in_right_img[j];
            paired_switches.push_back(paired_two_switch);
            paired[j] = true;
        }
    }
}


void RectSwitchFinder::refineSwitchInOneImage(const cv::Mat& roi_edge, const cv::Mat& roi_gray, cv::Rect& roi_rect, SwitchPtr& switch_ptr)
{
	
	cv::Mat lb_rw(cv::Size(3,1), CV_32F); //left black right white kernel
	lb_rw.at<float>(0,0) = -0.5;
	lb_rw.at<float>(0,1) = 0.0;
	lb_rw.at<float>(0,2) = 0.5;

	cv::Mat ub_dw(cv::Size(1,3), CV_32F); // up black down white kernel
	ub_dw.at<float>(0,0) = -0.5;
	ub_dw.at<float>(1,0) = 0.0;
	ub_dw.at<float>(2,0) = 0.5;

	std::vector<cv::Mat> filter_kernals;
	filter_kernals.push_back(ub_dw);
	filter_kernals.push_back(-lb_rw);
	filter_kernals.push_back(-ub_dw);
	filter_kernals.push_back(lb_rw);

	std::vector<cv::Vec3f> refined_lines(4);
    for(unsigned int i=0; i<4; i++){
        cv::Rect roi_rect_in_edge_img = switch_ptr->roi;
        roi_rect_in_edge_img.x -=  roi_rect.x;
        roi_rect_in_edge_img.y -=  roi_rect.y;
		cv::Mat filterd_roi, mask_roi;
		filter2D(roi_gray(roi_rect_in_edge_img), mask_roi, -1, filter_kernals[i]);
		filterd_roi = mask_roi.mul(roi_edge(roi_rect_in_edge_img));
        LineRefiner line_refiner(filterd_roi, switch_ptr->corner_points_in_roi[i%4], switch_ptr->corner_points_in_roi[(i+1)%4], refined_lines[i]);

    }
    for(unsigned int i=0; i<4; i++){
        cv::Point2f corner;
        getIntersecPoint(refined_lines[(i+3)%4], refined_lines[i%4], corner);
        switch_ptr->corner_points_in_roi[i] = corner;
    }
}

void RectSwitchFinder::uv2cxyzOpencv(const std::vector<Point2d> &uvLeftPoints, const std::vector<Point2d> &uvRightPoints, std::vector<Point3d> &xyzPoints)
{
	cv::Mat resultArray;
	cv::triangulatePoints(left_projection_cv_mat, right_projection_cv_mat, uvLeftPoints, uvRightPoints, resultArray);

	xyzPoints.resize(uvRightPoints.size());
	for (unsigned int i = 0; i < uvRightPoints.size(); i++)
	{
		xyzPoints[i].x = resultArray.at<double>(0, i) / resultArray.at<double>(3, i);
		xyzPoints[i].y = resultArray.at<double>(1, i) / resultArray.at<double>(3, i);
		xyzPoints[i].z = resultArray.at<double>(2, i) / resultArray.at<double>(3, i);
	}
}

void RectSwitchFinder::computeRectPosByStereoImages(PairedSwitch& paired_switch, SwitchPtr& switch_in_3d)
{

    std::vector<cv::Point2d> pts_in_left_img(4);
    std::vector<cv::Point2d> pts_in_right_img(4);
    for(unsigned int i=0; i<4; i++){
        pts_in_left_img[i].x = paired_switch.left->corner_points_in_roi[i].x + paired_switch.left->roi.x;
        pts_in_left_img[i].y = paired_switch.left->corner_points_in_roi[i].y + paired_switch.left->roi.y;
        pts_in_right_img[i].x = paired_switch.right->corner_points_in_roi[i].x + paired_switch.right->roi.x;
        pts_in_right_img[i].y = paired_switch.right->corner_points_in_roi[i].y + paired_switch.right->roi.y;   
    }
	std::vector<Point3d> pts_in_3d;
	uv2cxyzOpencv(pts_in_left_img, pts_in_right_img, pts_in_3d);
    RigidRectTransformSolver rigid_transform_solver(paired_switch.left->width, paired_switch.left->height); // mm
    
    Eigen::Matrix4d T_cam_to_rect; double scale_ration;
    rigid_transform_solver.setPointSets(pts_in_3d);
    rigid_transform_solver.solveTransform(T_cam_to_rect,scale_ration);
    switch_in_3d->width = paired_switch.left->width;
    switch_in_3d->height = paired_switch.left->height;
    switch_in_3d->corner_points_in_3d.resize(4);
	switch_in_3d->corner_points_in_roi;
    for(unsigned int i=0; i<4; i++){
        switch_in_3d->corner_points_in_3d[i] << pts_in_3d[i].x, pts_in_3d[i].y, pts_in_3d[i].z, 1.0;
    }
	switch_in_3d->corner_points_in_roi.insert(switch_in_3d->corner_points_in_roi.end(), paired_switch.left->corner_points_in_roi.begin(), paired_switch.left->corner_points_in_roi.end());
	switch_in_3d->corner_points_in_roi.insert(switch_in_3d->corner_points_in_roi.end(), paired_switch.right->corner_points_in_roi.begin(), paired_switch.right->corner_points_in_roi.end());

    switch_in_3d->T_cam_to_switch = T_cam_to_rect;

}

void RectSwitchFinder::findAllSwitches(cv::Mat& left_roi_img, cv::Mat& right_roi_img, cv::Rect& left_roi_rect, cv::Rect& right_roi_rect, std::vector<SwitchPtr>& all_switches, bool visualization)
{
    // main function
	
    cv::Mat left_roi_gray, left_roi_edge, left_roi_blurred;
	cv::GaussianBlur(left_roi_img, left_roi_blurred, cv::Size(5, 5), 1);
	cv::cvtColor(left_roi_blurred, left_roi_gray, cv::COLOR_BGR2GRAY);
 	getEdge(left_roi_gray, left_roi_edge);

    cv::Mat right_roi_gray, right_roi_edge, right_roi_blurred;
	cv::GaussianBlur(right_roi_img, right_roi_blurred, cv::Size(5, 5), 1);
	cv::cvtColor(right_roi_blurred, right_roi_gray, cv::COLOR_BGR2GRAY);
 	getEdge(right_roi_gray, right_roi_edge);

    std::vector<SwitchPtr> switches_in_left_img, switches_in_right_img;
    findRectSwitchInOneImage(left_roi_img, left_roi_gray, left_roi_edge, left_roi_rect, LEFT_CAMERA, switches_in_left_img, visualization);
    findRectSwitchInOneImage(right_roi_img, right_roi_gray, right_roi_edge, right_roi_rect, RIGHT_CAMERA, switches_in_right_img, visualization);

    std::vector<PairedSwitch> paired_switches;
    pairRectInStereoImages(switches_in_left_img, switches_in_right_img, paired_switches);
    
     for(unsigned int i=0; i< paired_switches.size(); i++){
        refineSwitchInOneImage(left_roi_edge, left_roi_gray, left_roi_rect, paired_switches[i].left);
        refineSwitchInOneImage(right_roi_edge, right_roi_gray, right_roi_rect, paired_switches[i].right);
    }
    
    for(unsigned int i=0; i< paired_switches.size(); i++){
        auto switch_in_3d = std::make_shared<Switch>();
        computeRectPosByStereoImages(paired_switches[i], switch_in_3d);
		if(isTransfromSimilaire(switch_in_3d->T_cam_to_switch, req_rough_T_cam_to_switch, 20, 0.02)){
			all_switches.push_back(switch_in_3d);
		}
    }
}

bool RectSwitchFinder::matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat& template_img, double& match_val, cv::Point& match_loc){
	const int size_factor = 1.0; // should be bigger than 0.1, smaller than 1.1
	double min_val, max_val;
	cv::Point min_loc, max_loc;
	double match_max_val = -1.0;
	cv::Point match_max_loc;
	for (double n = size_factor - 0.1 ; n < (size_factor + 0.1); n += 0.1)
	{
		cv::Mat template_img_resized;
		resize(template_img, template_img_resized, cv::Size(template_img.cols * n, template_img.rows * n));
		cv::Mat resultx;
		//int result_cols = switch_img.cols - template_img_resized.cols + 1;
		//int result_rows = switch_img.rows - template_img_resized.rows + 1;
		matchTemplate(switch_img, template_img_resized, resultx, CV_TM_CCOEFF_NORMED);
		minMaxLoc(resultx, &min_val, &max_val, &min_loc, &max_loc);
		//cout << minVal << endl;
		//cout << maxVal << endl;
		match_max_val = match_max_val > max_val ? match_max_val : max_val;
		match_max_loc = max_loc;
	}
	match_loc = match_max_loc;
	match_val = match_max_val;
	if(match_val < 0) return false;
	else return true;	
}
void RectSwitchFinder::readImagesInFolder(std::string folder_name, std::vector<cv::Mat>& images){
	// the filename of images should be 0.bmp, 1.bmp, 2.bmp and so on.
	int id = 0;
	while(true){
		cv::Mat one_image = cv::imread(folder_name +"/" + std::to_string(id) + std::string(".png"));
		id ++;
		if(!one_image.empty()){
			images.push_back(one_image);
		}
		else{
			return;
		}
	}
}
bool RectSwitchFinder::loadTemplateImages(){
	std::string path = ros::package::getPath("mm_visual_postion");

	readImagesInFolder(path + "/template_images/remote_switch/status_0", templates_remote_switch_status_0);
	if (templates_remote_switch_status_0.empty()){
		ROS_WARN("cannot load template image for remote switch in status_0");
		return false;
	}
	else{
		ROS_INFO("loaded %d template images for remote switch in status_0", (int)(templates_remote_switch_status_0.size()));
	}

	
	readImagesInFolder(path + "/template_images/remote_switch/status_1", templates_remote_switch_status_1);
	if (templates_remote_switch_status_1.empty()){
		ROS_WARN("cannot load template image for remote switch in status_1");
		return false;
	}
	else{
		ROS_INFO("loaded %d template images for remote switch in status_1", (int)(templates_remote_switch_status_1.size()));
	}
	return true;
}
void RectSwitchFinder::getStatus(const cv::Mat& roi, const Switch& the_switch, int& status, int camera_id){
	assert(the_switch.name == REMOTE_SWITCH_NAME);
	double match_score[2] = {-1.0, -1.0};
	for(unsigned int status_i = 0; status_i <2; status_i++){
		TemplateImageVec& templates_vec = (status_i == 0? templates_remote_switch_status_0 : templates_remote_switch_status_1);

		for(unsigned int template_i = 0; template_i < templates_vec.size(); template_i++){
			// calculate match score by using template matching
			double match_score_tmp = -1.0;
			cv::Point match_loc; 
			cv::Mat corrected_switch;
			getCorrectedSwitch(roi, the_switch, templates_vec[template_i].size(), corrected_switch, camera_id);
			matchTemplateElastic(corrected_switch, templates_vec[template_i], match_score_tmp, match_loc);
			if(match_score_tmp > match_score[status_i]) match_score[status_i] = match_score_tmp;
		}
	}
	if(match_score[1] > match_score[0])
		status = 1;
	else status = 0;
}

void RectSwitchFinder::getCorrectedSwitch(const cv::Mat& image_roi, const Switch& the_switch, cv::Size size, cv::Mat& corrected_switch, int camera_id){
	// used for template match
	std::vector<cv::Point2f> template_corners(4);
	template_corners[0] = cv::Point2f(0, 0);
	template_corners[1] = cv::Point2f(size.width, 0);
	template_corners[2] = cv::Point2f(size.width, size.height);
	template_corners[3] = cv::Point2f(0, size.height);
	std::vector<cv::Point2f> corners_in_image;
	if(camera_id == LEFT_CAMERA)
		corners_in_image.insert(corners_in_image.end(), the_switch.corner_points_in_roi.begin(), the_switch.corner_points_in_roi.begin() + 4);
	else
		corners_in_image.insert(corners_in_image.end(), the_switch.corner_points_in_roi.begin()+4, the_switch.corner_points_in_roi.end());

	cv::Mat transform_matrix = cv::getPerspectiveTransform(corners_in_image, template_corners);
	cv::warpPerspective(image_roi, corrected_switch, transform_matrix, size);
			
}