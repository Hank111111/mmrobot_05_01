#include "mm_visual_postion/switch/SwitchFinderByHough.h"

using namespace std;



int SwitchFinderByHough::matchTemplateElastic(const cv::Mat &switch_img, const cv::Mat& template_img, double& match_val, cv::Point& match_loc) {
	const int size_factor = 1.0;
	double min_val, max_val;
	cv::Point min_loc, max_loc;
	double match_max_val = -1.0;
	cv::Point match_max_loc;
	for (double n = size_factor - 0.2 > 0?size_factor - 0.2 > 0:0.04 ; n < (size_factor + 0.1); n += 0.04)
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
	if(match_val < 0) return -1;
	else return 0;
}


void SwitchFinderByHough::getEdge(cv::Mat& gray, cv::Mat& edge) {
	cv::Canny(gray, edge, 500, 1500, 5);
}

void SwitchFinderByHough::getEdgeAdapdativeThreshold(cv::Mat& gray, cv::Mat& edge, bool visualization) {
	do {
		if (adapdative_treshold_init) {
			int half_block_size = 5;
			int c = 2;

			cv::namedWindow("adapdative_threshold_edge", 0);
			cv::createTrackbar("block_size", "adapdative_threshold_edge", &half_block_size, 10, onChange);
			cv::createTrackbar("c", "adapdative_threshold_edge", &c, 50, onChange);
			adapdative_treshold_init = false;
		}
		int half_block_size = clip(cv::getTrackbarPos("half_block_size", "adapdative_threshold_edge"), 1, 10);
		int c = clip(cv::getTrackbarPos("c", "adapdative_threshold_edge"), 1, 50);

		adaptiveThreshold(gray, edge, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, half_block_size * 2 + 1, c);

		if (visualization) {
			cv::imshow("adapdative_threshold_edge", edge);
			char key = (char)cv::waitKey(50);
			if (key == 27) break;
		}
	} while (visualization);
}
void SwitchFinderByHough::getRotateRect(cv::Mat& edge, vector<cv::RotatedRect>& rect_list, bool visualization) {
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	vector<cv::Point> approx;
	vector<cv::Point> approx_hull;
	vector<cv::Point> hull;
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
void SwitchFinderByHough::getROI(vector<cv::RotatedRect>& rect_list, int img_width, int img_height, vector<cv::Rect>& roi_list, bool visualization, const cv::Mat& img) {
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


double inline SwitchFinderByHough::getRectArea(cv::Rect rect) {
	return rect.size().width * rect.size().height;
}
void SwitchFinderByHough::filterROI(vector<cv::Rect>& rect_list, vector<cv::Rect>& rect_list_filtered, bool visualization,const cv::Mat& img) {
	vector<bool> mask(rect_list.size(), true);
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


void SwitchFinderByHough::getCircles(const cv::Mat& gray, vector<cv::Vec4f>& circles, bool visualization) {
	int min_dist, param1, param2;
	min_dist = gray.rows / 8;
	param1 = 100;
	param2 = 90;
	do {
		if (visualization) {
			if (hough_circle_init) {
				cv::namedWindow("circles", 0);
				cv::createTrackbar("min_dist", "circles", &min_dist, 1000, onChange);
				cv::createTrackbar("param1", "circles", &param1, 1000, onChange);
				cv::createTrackbar("param2", "circles", &param2, 1000, onChange);
				hough_circle_init = false;
			}
			min_dist = clip(cv::getTrackbarPos("min_dist", "circles"), 1, 1000);
			param1 = clip(cv::getTrackbarPos("param1", "circles"), 1, 1000);
			param2 = clip(cv::getTrackbarPos("param2", "circles"), 1, 1000);
		}


		HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2, min_dist, param1, param2);

		// normalize accumulate value
		for (unsigned int i = 0; i < circles.size(); i++) {
			if (circles[i][2] == 0) {
				circles[i][3] = 0;
			}
			else
				circles[i][3] = circles[i][3] / circles[i][2]; //accumulate value / radius
		}
		if (visualization) {
			cv::Mat show_gray = gray.clone();
			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				// draw the circle center
				cv::circle(show_gray, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
				// draw the circle outline
				cv::circle(show_gray, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
			}
			cv::imshow("circles", show_gray);
			char key = (char)cv::waitKey(50);
			if (key == 27) break;
		}
	} while (visualization);
}

int SwitchFinderByHough::findPossibleSwitchInROI(const cv::Mat& img, const cv::Mat& gray, const cv::Mat& edge, const cv::Rect& roi_rect,
	const std::vector<cv::Mat>& template_img_vec, vector<cv::Point2d>& global_corners, double& template_match_score, bool visualization)
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


	// firstly find roughly center
	cv::Mat threshold_img;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	adaptiveThreshold(gray, threshold_img, 255, cv::THRESH_BINARY_INV, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 11, 4);
	cv::findContours(threshold_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	cv::Mat draw_img;
	if (visualization) {
		cv::cvtColor(threshold_img, draw_img, cv::COLOR_GRAY2BGR);
	}

	vector<cv::Point2f> center;
	vector<float> rect_size;
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
	vector<cv::Vec3f> left_lines, left_lines_2, filterd_left_lines;
	cv_v3_4_5::HoughLines(left_roi_img, left_lines, 1, M_PI / 180. * 0.2, 30, 0, 0, 0, M_PI / 180. * 15.); //0-15 degree lines
	cv_v3_4_5::HoughLines(left_roi_img, left_lines_2, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI - M_PI / 180.0 * 15.0, M_PI); //165-180 degree

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
	vector<cv::Vec3f> right_lines, right_lines_2, filterd_right_lines;
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
	vector<cv::Vec3f> up_lines, filterd_up_lines;
	cv_v3_4_5::HoughLines(up_roi_img, up_lines, 1, M_PI / 180. * 0.2, 30, 0, 0, M_PI / 2.0 - M_PI / 180. * 15., M_PI / 2.0 + M_PI / 180. * 15.); //75-105 degree
	if (up_lines.empty()) {
		return -1;
	}
	linesNMS(up_lines, filterd_up_lines,3);

	// find down lines
	cv::Rect down_roi_rect(0,center_y,edge.cols, edge.rows - center_y);
	cv::Mat down_roi_img = down_edge(down_roi_rect);
	vector<cv::Vec3f> down_lines, filterd_down_lines;
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
	std::vector<vector<cv::Point2f> > possible_corners;
	std::vector<double> possible_match_score;
	std::vector<double> possible_square_score;
	std::vector<double> possible_score;
	
	while(case_id < 16) //maximum 16 case
	{
		case_id ++;
		if(!line_iterator.getIndex(chosen_up_line_index, chosen_left_line_index, chosen_right_line_index, chosen_down_line_index)) break;
		if (!visualization) {
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

		vector<cv::Point2f> corners;
		corners.push_back(left_up_corner);
		corners.push_back(right_up_corner);
		corners.push_back(right_down_corner);
		corners.push_back(left_down_corner);
		double square_score = computeSquareScore(left_up_corner, right_up_corner, right_down_corner, left_down_corner);

		if(square_score > 50){//it is certain that this is not a square
			continue;
		} 
		double match_score_tmp;
		for(unsigned int template_i = 0; template_i < template_img_vec.size(); template_i++){
			// calculate match score by using template matching
			vector<cv::Point2f> template_corners(4);
			template_corners[0] = cv::Point2f(0, 0);
			template_corners[1] = cv::Point2f(template_img_vec[template_i].size().width, 0);
			template_corners[2] = cv::Point2f(template_img_vec[template_i].size().width, template_img_vec[template_i].size().height);
			template_corners[3] = cv::Point2f(0, template_img_vec[template_i].size().height);

			cv::Mat transform_matrix = cv::getPerspectiveTransform(corners, template_corners);
			cv::Mat corrected_switch;
			cv::warpPerspective(img, corrected_switch, transform_matrix, cv::Size(template_img_vec[template_i].size().width , template_img_vec[template_i].size().height));
			cv::Point match_loc; 
			matchTemplateElastic(corrected_switch, template_img_vec[template_i], match_score_tmp, match_loc);
		}


		//center, left_up, right_up, right_down
		possible_corners.push_back(corners);
		possible_match_score.push_back(match_score_tmp);
		possible_square_score.push_back(square_score);
		possible_score.push_back(match_score_tmp  - square_score * 0.01);
		
	}
	if(possible_score.size() == 0) return -1;
	auto max_it = std::max_element(possible_score.begin(), possible_score.end());
	int max_index = std::distance(possible_score.begin(), max_it);
	template_match_score = possible_match_score[max_index];

	cv::Vec3f refined_up_line, refined_down_line, refined_left_line, refined_right_line;
	LineRefiner refine_up_line(edge, possible_corners[max_index][0], possible_corners[max_index][1], refined_up_line);
	LineRefiner refine_right_line(edge, possible_corners[max_index][1], possible_corners[max_index][2], refined_right_line);
	LineRefiner refine_down_line(edge, possible_corners[max_index][2], possible_corners[max_index][3], refined_down_line);
	LineRefiner refine_left_line(edge, possible_corners[max_index][0], possible_corners[max_index][3], refined_left_line);

	cv::Point2f left_up_corner, left_down_corner, right_up_corner, right_down_corner;
	getIntersecPoint(refined_up_line, refined_left_line, left_up_corner);
	getIntersecPoint(refined_up_line, refined_right_line, right_up_corner);
	getIntersecPoint(refined_down_line,refined_right_line, right_down_corner);
	getIntersecPoint(refined_down_line, refined_left_line, left_down_corner);

	global_corners.resize(4);
	global_corners[0] = cv::Point2d(left_up_corner.x + roi_rect.x, left_up_corner.y + roi_rect.y);
	global_corners[1] = cv::Point2d(right_up_corner.x + roi_rect.x, right_up_corner.y + roi_rect.y);
	global_corners[2] = cv::Point2d(right_down_corner.x + roi_rect.x, right_down_corner.y + roi_rect.y);
	global_corners[3] = cv::Point2d(left_down_corner.x + roi_rect.x, left_down_corner.y + roi_rect.y);
	return 0;
}
void SwitchFinderByHough::addCenterDistanceAndAccumValue(const vector<cv::Vec3f>& lines1, const vector<int>& lines1_chosen_indexes, const vector<float>& center_dist_1,
	const vector<cv::Vec3f>& lines2, const vector<int>& lines2_chosen_indexes, const vector<float>& center_dist_2, float weight_dist, float weight_accum, cv::Mat& score) {
	for (unsigned int i = 0; i < lines1_chosen_indexes.size(); i++) {
		for (unsigned int j = i + 1; j < lines2_chosen_indexes.size(); j++) {
			score.at<float>(i, j) += weight_dist * (center_dist_1[i] + center_dist_2[j]) + weight_accum * (lines1[lines1_chosen_indexes[i]][2] + lines2[lines2_chosen_indexes[j]][2]);
		}
	}
}

void SwitchFinderByHough::addAccumValue(const vector<cv::Vec3f>& lines1, const vector<cv::Vec3f>& lines2, float weight_accum, cv::Mat& score) {
	for (unsigned int i = 0; i < min(lines1.size(), lines2.size()); i++) {
		//std::cout<<i<<std::endl;
		for (unsigned int j = i; j < min(lines1.size(),lines2.size()); j++) {
			
			score.at<float>(i, j) +=  weight_accum * (lines1[i][2] + lines2[j][2]);
		}
	}
}
void SwitchFinderByHough::getParallelScore(const vector<cv::Vec3f>& lines1, const vector<bool>& lines1_mask,
	const vector<cv::Vec3f>& lines2, const vector<bool>& lines2_mask, float weight, cv::Mat& score) {
	score = cv::Mat::ones(cv::Size(lines1.size(), lines2.size()), CV_32FC1) * (10000.0);
	for (unsigned int i = 0; i < lines1.size(); i++) {
		for (unsigned int j = i + 1; j < lines2.size(); j++) {
			if (lines1_mask[i] && lines2_mask[j]){
				float diff_angle = fabs(lines1[i][1] - lines2[j][1]);
				if(diff_angle > M_PI) diff_angle -= M_PI;
				score.at<float>(i, j) = weight * diff_angle;
			}
			else {
				score.at<float>(i, j) = 10000;
			}
		}
	}
}

void SwitchFinderByHough::hardConstraintByDistance(const vector<float>& distance, vector<bool>& mask, float min_dist, float max_dist) {
	for (unsigned int i = 0; i < distance.size(); i++) {
		if (distance[i] < max_dist && distance[i] > min_dist && mask[i]) {
			mask[i] = true;
		}
		else {
			mask[i] = false;
		}
	}
}

void SwitchFinderByHough::hardConstraintByAccumValue(const vector<cv::Vec3f>& lines, const vector<int>& indexes, vector<bool>& mask, float prop_k) {
	float max_accum_value = 0;
	for (unsigned int i = 0; i < indexes.size(); i++) {
		if (lines[indexes[i]][2] > max_accum_value) {
			max_accum_value = lines[indexes[i]][2];
		}
	}
	//int total_num = 0;
	for (unsigned int i = 0; i < indexes.size(); i++) {
		if (mask[i]) {
			if (lines[indexes[i]][2] > prop_k * max_accum_value) {
				//	total_num += 1;
			}
			else {
				mask[i] = false;
			}
		}
	}


}


void SwitchFinderByHough::computeDistance(const vector<cv::Vec3f>& lines, const vector<int>& chosen_index, vector<float>& distance, float center_x, float center_y) {
	for (unsigned int i = 0; i < chosen_index.size(); i++) {
		// line equation rho - x*cos(theta) - y* sin(theta) = 0
		float rho = lines[chosen_index[i]][0];
		float theta = lines[chosen_index[i]][1];
		distance.push_back(fabs(rho - center_x * std::cos(theta) - center_y * std::sin(theta)));
	}
}

int SwitchFinderByHough::findSwitch(const cv::Mat& img, const std::vector<cv::Mat>& template_img_vec, vector<cv::Point2d> & switch_corners, double& match_score, bool visualization) {
	cv::Mat gray, edge;
	cv::GaussianBlur(img, img, cv::Size(5, 5), 1);

	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
 	getEdge(gray, edge);
	//getEdgeAdapdativeThreshold(gray, edge, true);
	vector<cv::RotatedRect> rect_list;
	vector<cv::Rect> roi_list, filterd_roi_list;
	getRotateRect(edge, rect_list, visualization);
	getROI(rect_list, gray.size().width, gray.size().height, roi_list, visualization, img);
	filterROI(roi_list, filterd_roi_list, visualization, img);
	vector<double> template_match_scores;
	vector< vector<cv::Point2d> > corners_list;
	for (unsigned int i = 0; i < filterd_roi_list.size(); i++) {
		cv::Mat img_roi = img(filterd_roi_list[i]);
		cv::Mat gray_roi = gray(filterd_roi_list[i]);
		cv::Mat edge_roi = edge(filterd_roi_list[i]);
		vector<cv::Point2d> corners;
		double template_score;
		if (findPossibleSwitchInROI(img_roi, gray_roi, edge_roi, filterd_roi_list[i], template_img_vec, corners, template_score, visualization) == 0) {
			corners_list.push_back(corners);
			template_match_scores.push_back(template_score);
			if (visualization) {
				cv::waitKey(0);
			}
		}

	}
	if (template_match_scores.size() <= 0) {
		return -1;
	}
	std::vector<double>::iterator max_match_score_it = std::max_element(template_match_scores.begin(), template_match_scores.end());
	if(*max_match_score_it < 0.7){
		std::cout<<"not sure the switch found is the correct switch, abort"<<std::endl;
		return -1; //treshold
	}
	switch_corners = corners_list[std::distance(template_match_scores.begin(), max_match_score_it)];
	match_score = *max_match_score_it;
	std::cout<<"max match score"<<*max_match_score_it<<std::endl;
	std::cout<<"all scores: ";
	std::sort(template_match_scores.begin(),template_match_scores.end());
	for(unsigned int i=template_match_scores.size()-1; i>=0;i--){
		std::cout<<template_match_scores[i]<<" ";
	}
	std::cout<<std::endl;

	return 0;
}