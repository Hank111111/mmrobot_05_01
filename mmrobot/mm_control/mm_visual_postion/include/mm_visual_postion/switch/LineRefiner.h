#ifndef __LINE_REFINER_H__
#define __LINE_REFINER_H__

#include <opencv2/opencv.hpp>
#include "ceres/ceres.h"

class LineRefiner{
private:
    void getPointsAroundLine(const cv::Mat& edge_image, const cv::Point2f& endpoint_0_estimate, const cv::Point2f& endpoint_1_estimate, std::vector<cv::Point2f>& points, const int threshold_in_pixel){
        /**
         * Get non-zero points on the edge image around the line.
         * The line is presented by two endpoints (left and right) or (up and down)
         */ 
        assert(edge_image.type() == CV_8UC1);
        points.clear();
        cv::Point2f endpoint_0, endpoint_1;
        int x_length = fabs(endpoint_1_estimate.x - endpoint_0_estimate.x);
        int y_length = fabs(endpoint_1_estimate.y - endpoint_0_estimate.y);
        if(x_length > y_length){
            if(endpoint_1_estimate.x - endpoint_0_estimate.x >= 0){
                endpoint_0 = endpoint_0_estimate;
                endpoint_1 = endpoint_1_estimate;
            }
            else{
                endpoint_1 = endpoint_0_estimate;
                endpoint_0 = endpoint_1_estimate;
            }
            float k = (endpoint_1.y - endpoint_0.y)/(endpoint_1.x - endpoint_0.x);

            for(float delta_x = 0; delta_x <= endpoint_1.x - endpoint_0.x; delta_x += 1.0){
                const float y = endpoint_0.y + delta_x * k;
                const float x = endpoint_0.x + delta_x;
                for(float dist = - threshold_in_pixel; dist <= threshold_in_pixel; dist += 1.0){                    
                    if(y + dist>=0 && y + dist<edge_image.rows){
                        const float round_x = std::round(x);
                        const float round_y_plus_dist = std::round(y+dist);
                        if(edge_image.at<uchar>(round_y_plus_dist, round_x) > 0){
                            points.push_back(cv::Point2f(round_x,round_y_plus_dist));
                        }
                    }
                }
            }
        }
        else{
            if(endpoint_1_estimate.y - endpoint_0_estimate.y >= 0){
                endpoint_0 = endpoint_0_estimate;
                endpoint_1 = endpoint_1_estimate;
            }
            else{
                endpoint_1 = endpoint_0_estimate;
                endpoint_0 = endpoint_1_estimate;
            }
            float k_inv = (endpoint_1.x - endpoint_0.x)/(endpoint_1.y - endpoint_0.y);
            for(float delta_y = 0; delta_y <= endpoint_1.y - endpoint_0.y; delta_y += 1.0){
                const float x = endpoint_0.x + delta_y * k_inv;
                const float y = endpoint_0.y + delta_y;
                for(float dist = - threshold_in_pixel; dist <= threshold_in_pixel; dist += 1.0){                    
                    if(x + dist>=0 && x + dist<edge_image.cols){\
                        const float round_y = std::round(y);
                        const float round_x_plus_dist = std::round(x+dist);
                        if(edge_image.at<uchar>(round_y, round_x_plus_dist) > 0){
                            points.push_back(cv::Point2f(round_x_plus_dist,round_y));
                        }
                    }
                }
            }
        }

    }
    class PointLineDistance: public ceres::SizedCostFunction<1/* number of residuals */,
                                                             2/* size of first parameter */>
    {
    public:
        PointLineDistance(cv::Point2f point){
        /**
         * Get line to point square distance, point is given at init.
         */
            x = point.x;
            y = point.y;
        }
        virtual ~PointLineDistance() {}
        virtual bool Evaluate(double const* const* line_params,
                                double* residuals,
                                double** jacobians) const {
            /**
             * Get line to the given point square distance .
             * The line is presented in the opencv's convention with equation:
             *  rho - x*cos(theta) - y* sin(theta) = 0, where line_param = [rho, theta]
             * @ param line_params: line parameter (composed with 1 parameter, which has 2 dimension)
             * @ param residuals: distance from line to point.
             * @ param jacobians: jacobians matrix
             */
            double rho = line_params[0][0];
            double sin_theta = std::sin(line_params[0][1]);
            double cos_theta = std::cos(line_params[0][1]);
            double sign_dist = rho - x * cos_theta - y * sin_theta;
            residuals[0] = sign_dist * sign_dist;
            if (jacobians != NULL && jacobians[0] != NULL) {
                jacobians[0][0] = 2 * sign_dist; //d(dist^2)/d(rho)
                jacobians[0][1] = 2 * sign_dist * (x * sin_theta - y * cos_theta); //d(dist^2)/d(rho)
            }
            return true;
        }
    private:
        double x,y;
    };
    
public:
    LineRefiner(const cv::Mat& edge_image, const cv::Point2f& endpoint_0, const cv::Point2f& endpoint_1, cv::Vec3f& line){
        /**
         * This class takes edge image as input, the roughly determinated line is also inputted (which could be calculated by hough transformation), 
         * and this line will be refined by line fitting based on LM iterative method.
         * @ param edge_image: the edge image, which should be with the type CV_8U
         * @ param endpoint_0: one of the end point of the estimated line
         * @ param endpoint_1: another end point of the estimated line
         * @ param line: output the refined line
         */
        
        std::vector<cv::Point2f> possible_line_points;
        getPointsAroundLine(edge_image, endpoint_0, endpoint_1, possible_line_points, 3);

        // init line
        // rho - x*cos(theta) - y* sin(theta) = 0
        // (y0-y1)*x -(x0-x1)*y + (x0-x1)*y0 - (y0-y1)*x0 = 0
        float x0_minus_x1 = endpoint_0.x - endpoint_1.x;
        float y0_minus_y1 = endpoint_0.y - endpoint_1.y;
        float theta = std::atan2(-x0_minus_x1, y0_minus_y1);
        line[1] = fabs(theta);
        if(theta >= 0)
            line[0] = (- x0_minus_x1*endpoint_0.y + (y0_minus_y1)*endpoint_0.x) / std::sqrt(x0_minus_x1 * x0_minus_x1 + y0_minus_y1*y0_minus_y1);
        else 
            line[0] =  (x0_minus_x1*endpoint_0.y - (y0_minus_y1)*endpoint_0.x) / std::sqrt(x0_minus_x1 * x0_minus_x1 + y0_minus_y1*y0_minus_y1);

        solve(possible_line_points, line);
    }

    void solve(std::vector<cv::Point2f>& possible_line_points, cv::Vec3f& line){
        ceres::Problem problem;
        ceres::LossFunction* loss = new ceres::CauchyLoss(0.3);
        double param_line[2];
        param_line[0] = line[0];
        param_line[1] = line[1];

        for(unsigned int i=0; i<possible_line_points.size(); i++){
            ceres::CostFunction* cost_function = new PointLineDistance(possible_line_points[i]);
             problem.AddResidualBlock(cost_function, loss, param_line);
        }


        // Build and solve the problem.
        ceres::Solver::Options options;
        options.max_num_iterations = 500;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_type = ceres::TRUST_REGION;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //std::cout << summary.FullReport() << "\n";
        //std::cout << summary.BriefReport() << "\n";
        //std::cout << "rho : " << line[0]
        //    << " -> " << param_line[0] << "\n";
        //std::cout << "theta : " << line[1]
        //    << " -> " << param_line[1] << "\n";


        // Update the result
        line[0] = param_line[0];
        line[1] = param_line[1];


    }

};

#endif