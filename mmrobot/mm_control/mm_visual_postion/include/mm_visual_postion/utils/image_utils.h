#ifndef _IMAGE_UTILS_H__
#define _IMAGE_UTILS_H__

#include <algorithm>
#include "opencv2/opencv.hpp"
#include <vector>

template <typename T>
T clip(T n, T lower, T upper) {
	return std::max(lower, std::min(n, upper));
}

struct {
	bool operator()(cv::Vec3f& line_1, cv::Vec3f& line_2) const
	{   
		return line_1[2] > line_2[2];
	}   
} lineScoreGreater;

inline void onChange(int, void*){}


inline void boundRectROI(cv::Rect& roi, const cv::Size& image_size){
	if(image_size.width < roi.width) roi.width = image_size.width;
	if(image_size.height < roi.height) roi.height = image_size.height;
	if(roi.x + roi.width > image_size.width) roi.x = image_size.width -roi.width;
	if(roi.y + roi.height > image_size.height) roi.y = image_size.height -roi.height;
	if(roi.x <0) roi.x = 0;
	if(roi.y <0) roi.y = 0;
	
}
inline double computeDistance(cv::Point2d& point_1, cv::Point2d& point_2){
	return sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y));
}
inline double computeSquareScore(cv::Point2d left_up_point, cv::Point2d right_up_point, cv::Point2d right_down_point, cv::Point2d left_down_point){
	double parallel_score = fabs((left_up_point.x - right_up_point.x) - (left_down_point.x - right_down_point.x))
		+  fabs((left_up_point.y - right_up_point.y) - (left_down_point.y - right_down_point.y))
		+  fabs((left_up_point.x - left_down_point.x) - (right_up_point.x - right_down_point.x))
		+  fabs((left_up_point.y - left_down_point.y) - (right_up_point.y - right_down_point.y));
	double up_length = computeDistance(left_up_point, right_up_point);
	double down_length = computeDistance(left_down_point, right_down_point);
	double left_length = computeDistance(left_up_point, left_down_point);
	double right_length = computeDistance(right_up_point, right_down_point);
	double avg_length = (up_length + down_length + left_length + right_length) / 4.0;
	double length_score = fabs(avg_length - up_length) + fabs(avg_length - down_length) + fabs(avg_length - left_length) + fabs(avg_length - right_length);

	return parallel_score + length_score;
}
class SquareLinesIterator{
	int lines_num[4];
	int lines_index[4];
	bool has_next;
public:
	SquareLinesIterator(int lines_0_num, int lines_1_num, int lines_2_num, int lines_3_num){
		lines_num[0] = lines_0_num;
		lines_num[1] = lines_1_num;
		lines_num[2] = lines_2_num;
		lines_num[3] = lines_3_num;

		lines_index[0] = 0;
		lines_index[1] = 0;
		lines_index[2] = 0;
		lines_index[3] = 0;
		has_next = true;
	}
	bool getIndex(int& id_lines_0, int& id_lines_1, int& id_lines_2, int& id_lines_3){
		if(!has_next) return false;
		id_lines_0 = lines_index[0];
		id_lines_1 = lines_index[1];
		id_lines_2 = lines_index[2];
		id_lines_3 = lines_index[3];
		
		lines_index[3] ++;
		if(lines_index[3] >= lines_num[3]){
			lines_index[3] = 0;
			lines_index[2] ++;
			if(lines_index[2] >= lines_num[2]){
				lines_index[2] = 0;
				lines_index[1] ++;
				if(lines_index[1] >= lines_num[1]){
					lines_index[1] = 0;
					lines_index[0] ++;
					if(lines_index[0] >= lines_num[0]){
						has_next= false;
					}
				}
			}
		}
		return true;
	}
};

inline bool similarLines(const cv::Vec3f line_1, const cv::Vec3f& line_2, float r_threshold, float theta_threshold){
	if(fabs(line_1[0] - line_2[0]) < r_threshold && fabs(line_1[1] - line_2[1]) < theta_threshold) return true;
	if(fabs(line_1[0] + line_2[0]) < r_threshold && fabs(line_1[1] + line_2[1] - M_PI) < theta_threshold) return true;
	return false;
}

inline void linesNMS(const std::vector<cv::Vec3f>& lines, std::vector<cv::Vec3f>& filterd_lines, unsigned int max_num){
	// lines should be sorted (maximum to minimum)
	filterd_lines.clear();
	filterd_lines.push_back(lines[0]);
	for(unsigned int i=0; i<lines.size(); i++){
		bool is_new_local_max = true;
		for(unsigned int j=0; j<filterd_lines.size(); j++){
			if(similarLines(lines[i], filterd_lines[j], 20.0, 0.3)) {
				is_new_local_max = false;
				break;
			}
		}
		if(is_new_local_max){
			filterd_lines.push_back(lines[i]);
			if(filterd_lines.size() >= max_num) break;
		}
	}

}
inline void drawLine(cv::Mat img, cv::Vec3f& line, int color_r, int color_g, int color_b) {
	float rho = line[0];
	float theta = line[1];
	cv::Point pt1, pt2;
	double a = cos(theta), b = sin(theta);
	double x0 = a * rho, y0 = b * rho;
	pt1.x = cvRound(x0 + 1000 * (-b));
	pt1.y = cvRound(y0 + 1000 * (a));
	pt2.x = cvRound(x0 - 1000 * (-b));
	pt2.y = cvRound(y0 - 1000 * (a));
	cv::line(img, pt1, pt2, cv::Scalar(color_r, color_g, color_b), 1, 0);
}

inline void drawLines(cv::Mat img, std::vector<cv::Vec3f>& lines, int r, int g, int b) {
	// Draw the lines
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		drawLine(img, lines[i], r, g, b);
	}
}

inline void drawLines(cv::Mat img, std::vector<cv::Point2f>& corners, int r, int g, int b) {
	cv::line(img, corners[0], corners[1], cv::Scalar(r, g, b), 1);
	cv::line(img, corners[1], corners[2], cv::Scalar(r, g, b), 1);
	cv::line(img, corners[2], corners[3], cv::Scalar(r, g, b), 1);
	cv::line(img, corners[3], corners[0], cv::Scalar(r, g, b), 1);
}


inline void getIntersecPoint(const cv::Vec3f& line1, const cv::Vec3f& line2, cv::Point2f& coner) {
	//lines: left, up, right, down
	double rho1 = line1[0];
	double theta1 = line1[1];
	double rho2 = line2[0];
	double theta2 = line2[1];

	double cos_theta1 = std::cos(theta1);
	double cos_theta2 = std::cos(theta2);
	double sin_theta1 = std::sin(theta1);
	double sin_theta2 = std::sin(theta2);

	if (fabs(cos_theta1 * sin_theta2 - cos_theta2 * sin_theta1) < 1e-5) {
		std::cout << "two lines are almost parallel" << std::endl;
	}
	double x = (rho1*sin_theta2 - rho2 * sin_theta1) / (cos_theta1 * sin_theta2 - cos_theta2 * sin_theta1);
	double y = (rho1*cos_theta2 - rho2 * cos_theta1) / (sin_theta1 * cos_theta2 - sin_theta2 * cos_theta1);
	coner.x = float(x);
	coner.y = float(y);
}


inline void getIntersecPoint(const cv::Vec3f& line1, const cv::Rect& roi_rect_1, const cv::Vec3f& line2,const cv::Rect& roi_rect_2, cv::Point2f& coner) {
	// transform each line (in roi) into whole image's coordinate
	float alpha_1 = std::atan2(roi_rect_1.y, roi_rect_1.x);
	float alpha_2 = std::atan2(roi_rect_2.y, roi_rect_2.x);
	cv::Vec3f line1_whole_coord, line2_whole_coord;
	line1_whole_coord[0] = line1[0] + std::cos(line1[1] - alpha_1) * std::sqrt(std::pow(roi_rect_1.x, 2) + std::pow(roi_rect_1.y, 2));
	line1_whole_coord[1] = line1[1];

	line2_whole_coord[0] = line2[0] + std::cos(line2[1] - alpha_2) * std::sqrt(std::pow(roi_rect_2.x, 2) + std::pow(roi_rect_2.y, 2));
	line2_whole_coord[1] = line2[1];
	getIntersecPoint(line1_whole_coord, line2_whole_coord, coner);

}
#endif //_IMAGE_UTILS_H__