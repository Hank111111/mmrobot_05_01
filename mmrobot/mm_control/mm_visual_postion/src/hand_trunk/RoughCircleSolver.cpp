#include "mm_visual_postion/hand_trunck/RoughCircleSolver.h"

void getValidContourPoints(std::vector<cv::Point>& all_contour_points, std::vector<int>& valid_indexes, std::vector<cv::Point>& valid_contour_points){
    valid_contour_points.resize(valid_indexes.size());
    for (unsigned int inliner_i = 0; inliner_i < valid_indexes.size(); inliner_i++)
    {
        valid_contour_points[inliner_i] = all_contour_points[valid_indexes[inliner_i]];
    }
}
void downSampling(std::vector<cv::Point>& points, double ratio, unsigned int min_num, unsigned int max_num){
    
    if(points.size() * ratio < min_num){
        if(points.size() < min_num) return;
        ratio = static_cast<double>(min_num)/points.size();
    }
    if(points.size() * ratio > max_num){
        ratio = static_cast<double>(max_num)/points.size();
    }
    std::vector<cv::Point> down_sampling_points;
    double step = 1.0/ratio;
    double i=0;
    while(i < (double) points.size()){
        down_sampling_points.push_back(points[static_cast<int>(i)]);
        i += step;
    }
    points = down_sampling_points;

}
void RoughCircleSolver::getPossibleEllipse(const cv::Mat &edge_input, std::vector<EllipseWithScore> &ellipses_vec,
                                           std::vector<VecPointPtr> &valid_circle_contours_vec)
{
/**
    * find ellipse from a single image, the ellipses are presented in the pixel coordinate.
    * @param edge an edge image (binary)
    * @param ellipses the possible ellipse in the image, presented in the pixel coordinate. 
    * 
    * The ellipse is presented in the form of quatratic, for example, if C present a ellipse here,
    * thus X^t * C * X =0, where X = [x, y, 1]^t , see https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections 
    */
#ifdef DEBUG
    cv::Mat show_img;
    cv::cvtColor(edge_input, show_img, CV_GRAY2BGR);
#endif
    cv::Mat edge;
    cv::dilate(edge_input, edge, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

    cv::erode(edge, edge, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edge, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    const double flattening_treshold = 2.0; // the threshold of flattening of the ellipse
    const double min_area_threshold = 2000; // the threshold of minimum area of the ellipse

    std::vector<FittedEllipse> all_ellipses;
    for (unsigned int i = 0; i < contours.size(); i++)
    {

#ifdef DEBUG
        //cv::drawContours(show_img, contours, i, cv::Scalar(0, 255, 0), 1);
#endif
        int count = contours[i].size();
        if (count < 6)
            continue; //need at least 6 points to fit an ellipse
        std::vector<int> inliners_indexes;
        cv::cvtColor(edge_input, show_img, CV_GRAY2BGR);
        for (unsigned int show_i = 0; show_i < contours[i].size(); show_i++)
        {
            show_img.at<cv::Vec3b>(contours[i][show_i]) = cv::Vec3b(255, 255, 0);
        }
        cv::RotatedRect box = RansacFitEllipse(contours[i], inliners_indexes);
        if (inliners_indexes.size() == 0)
            continue;
        if (std::isnan(box.size.width) || std::isnan(box.size.height))
        {
            continue;
        }
        if (std::max(box.size.width, box.size.height) / std::min(box.size.width, box.size.height) > flattening_treshold)
            continue;

        if (box.size.width * box.size.height < min_area_threshold)
            continue;

#ifdef DEBUG
        cv::ellipse(show_img, box, cv::Scalar(0, 0, 255), 1, CV_AA);
#endif //DEBUG

        FittedEllipse fitted_ellipse(i, box);
        double score = contours[i].size() / (double)inliners_indexes.size();

        getValidContourPoints(contours[i], inliners_indexes, *(fitted_ellipse.valid_contours_ptr));

        fitted_ellipse.score = score;
        fitted_ellipse.cover(contours[i]);
        all_ellipses.push_back(fitted_ellipse);
    }

    // try to fuse the seperated components of the same ellipse
    std::vector<bool> fused(all_ellipses.size());
    for (unsigned int i = 0; i < all_ellipses.size(); i++)
    {
        for (unsigned int j = i + 1; j < all_ellipses.size(); j++)
        {
            if (fused[j])
                continue;

            cv::cvtColor(edge_input, show_img, CV_GRAY2BGR);
            cv::drawContours(show_img, contours, all_ellipses[i].id, cv::Scalar(0, 255, 0), 1);
            cv::drawContours(show_img, contours, all_ellipses[j].id, cv::Scalar(0, 255, 255), 1);

            
            if (all_ellipses[i].hasOverlap(all_ellipses[j]))
                continue; // can't belong to the same ellipse
            if (equal(all_ellipses[i].box, all_ellipses[j].box, 15, 15, 360))
                fused[j] = true;
            std::vector<cv::Point> fused_contours;
            fused_contours = *(all_ellipses[i].valid_contours_ptr);
            fused_contours.insert(fused_contours.end(), all_ellipses[j].valid_contours_ptr->begin(), all_ellipses[j].valid_contours_ptr->end());
            std::vector<int> inliners_indexes;
            cv::RotatedRect fused_box = RansacFitEllipse(fused_contours, inliners_indexes);

#ifdef DEBUG

            cv::ellipse(show_img, fused_box, cv::Scalar(0, 255, 255), 1, CV_AA);

#endif //DEBUG
            double score = fused_contours.size() / (double)inliners_indexes.size();

            if (score > 0.9)
            {
                all_ellipses[i].score = score;
                all_ellipses[i].possible_other_parts_id.push_back(all_ellipses[j].id);
                fused[j] = true;
            }
        }
    }

    std::vector<std::shared_ptr<VecPoint> > filtered_valid_contours_ptr_vec;
    for (unsigned int i = 0; i < all_ellipses.size(); i++)
    {
        cv::ellipse(show_img, all_ellipses[i].box, cv::Scalar(255, 255, 0), 1, CV_AA);

        if (fused[i])
            continue;
        if (all_ellipses[i].possible_other_parts_id.size() == 0)
        {
            cv::ellipse(show_img, all_ellipses[i].box, cv::Scalar(255, 0, 0), 1, CV_AA);
            ellipses_vec.push_back(EllipseWithScore(all_ellipses[i].box, all_ellipses[i].score));
            valid_circle_contours_vec.push_back(all_ellipses[i].valid_contours_ptr);
        }
        else
        {
            std::vector<cv::Point> fused_contours;
            fused_contours = contours[all_ellipses[i].id];
            for (unsigned int other_idx = 0; other_idx < all_ellipses[i].possible_other_parts_id.size(); other_idx++)
            {
                int id = all_ellipses[i].possible_other_parts_id[other_idx]; // contour's index
                fused_contours.insert(fused_contours.end(), contours[id].begin(), contours[id].end());
            }
            std::vector<int> inliners_indexes;
            cv::RotatedRect fused_box = RansacFitEllipse(fused_contours, inliners_indexes);
            double score = fused_contours.size() / (double)inliners_indexes.size();

            std::shared_ptr<VecPoint> inliner_points_ptr = std::make_shared<VecPoint>(inliners_indexes.size());
            getValidContourPoints(fused_contours, inliners_indexes, *inliner_points_ptr);
            valid_circle_contours_vec.push_back(inliner_points_ptr);

            cv::ellipse(show_img, fused_box, cv::Scalar(255, 0, 0), 1, CV_AA);
            ellipses_vec.push_back(EllipseWithScore(fused_box, score));
        }
    }
}

void RoughCircleSolver::computeI2I3I4(const Eigen::Matrix4d &A, const Eigen::Matrix4d &B, double &I_2, double &I_3, double &I_4)
{
    /** compute I_2, I_3, I_4, which are mentioned in the paper : "Long Quan. Conic Reconstruction and Correspondence from Two Views"
     * to simplify the programming, we choose to calculate I_2, I_3, I_4 numerically
     * by setting lambda = 1, -1, 2
    */
    double y_positive_one = (A + B).determinant();
    double k = 1.0; //std::exp(std::log(y_positive_one) / 4.0);
    double y_negative_one = ((A - B) / k).determinant();
    double y_positive_two = ((A + B * 2) / k).determinant();
    y_positive_one = ((A + B) / k).determinant();
    I_2 = (y_positive_two - y_negative_one) / 6.0 - y_positive_one / 2.0;
    I_3 = (y_positive_one + y_negative_one) / 2.0;
    I_4 = y_positive_one - y_negative_one / 3.0 - y_positive_two / 6.0;
}

void RoughCircleSolver::computeI2I3I4Analytic(const Eigen::Matrix4d &A, const Eigen::Matrix4d &B, double &I_2, double &I_3, double &I_4)
{
    /** compute I_2, I_3, I_4, which are mentioned in the paper : "Long Quan. Conic Reconstruction and Correspondence from Two Views"
    */
    I_2 = A(0, 0) * B(1, 1) * B(2, 2) * B(3, 3) - A(0, 0) * B(1, 1) * B(2, 3) * B(3, 2) - A(0, 0) * B(1, 2) * B(2, 1) * B(3, 3) + A(0, 0) * B(1, 2) * B(2, 3) * B(3, 1) + A(0, 0) * B(1, 3) * B(2, 1) * B(3, 2) - A(0, 0) * B(1, 3) * B(2, 2) * B(3, 1) - A(0, 1) * B(1, 0) * B(2, 2) * B(3, 3) + A(0, 1) * B(1, 0) * B(2, 3) * B(3, 2) + A(0, 1) * B(1, 2) * B(2, 0) * B(3, 3) - A(0, 1) * B(1, 2) * B(2, 3) * B(3, 0) - A(0, 1) * B(1, 3) * B(2, 0) * B(3, 2) + A(0, 1) * B(1, 3) * B(2, 2) * B(3, 0) + A(0, 2) * B(1, 0) * B(2, 1) * B(3, 3) - A(0, 2) * B(1, 0) * B(2, 3) * B(3, 1) - A(0, 2) * B(1, 1) * B(2, 0) * B(3, 3) + A(0, 2) * B(1, 1) * B(2, 3) * B(3, 0) + A(0, 2) * B(1, 3) * B(2, 0) * B(3, 1) - A(0, 2) * B(1, 3) * B(2, 1) * B(3, 0) - A(0, 3) * B(1, 0) * B(2, 1) * B(3, 2) + A(0, 3) * B(1, 0) * B(2, 2) * B(3, 1) + A(0, 3) * B(1, 1) * B(2, 0) * B(3, 2) - A(0, 3) * B(1, 1) * B(2, 2) * B(3, 0) - A(0, 3) * B(1, 2) * B(2, 0) * B(3, 1) + A(0, 3) * B(1, 2) * B(2, 1) * B(3, 0) - A(1, 0) * B(0, 1) * B(2, 2) * B(3, 3) + A(1, 0) * B(0, 1) * B(2, 3) * B(3, 2) + A(1, 0) * B(0, 2) * B(2, 1) * B(3, 3) - A(1, 0) * B(0, 2) * B(2, 3) * B(3, 1) - A(1, 0) * B(0, 3) * B(2, 1) * B(3, 2) + A(1, 0) * B(0, 3) * B(2, 2) * B(3, 1) + A(1, 1) * B(0, 0) * B(2, 2) * B(3, 3) - A(1, 1) * B(0, 0) * B(2, 3) * B(3, 2) - A(1, 1) * B(0, 2) * B(2, 0) * B(3, 3) + A(1, 1) * B(0, 2) * B(2, 3) * B(3, 0) + A(1, 1) * B(0, 3) * B(2, 0) * B(3, 2) - A(1, 1) * B(0, 3) * B(2, 2) * B(3, 0) - A(1, 2) * B(0, 0) * B(2, 1) * B(3, 3) + A(1, 2) * B(0, 0) * B(2, 3) * B(3, 1) + A(1, 2) * B(0, 1) * B(2, 0) * B(3, 3) - A(1, 2) * B(0, 1) * B(2, 3) * B(3, 0) - A(1, 2) * B(0, 3) * B(2, 0) * B(3, 1) + A(1, 2) * B(0, 3) * B(2, 1) * B(3, 0) + A(1, 3) * B(0, 0) * B(2, 1) * B(3, 2) - A(1, 3) * B(0, 0) * B(2, 2) * B(3, 1) - A(1, 3) * B(0, 1) * B(2, 0) * B(3, 2) + A(1, 3) * B(0, 1) * B(2, 2) * B(3, 0) + A(1, 3) * B(0, 2) * B(2, 0) * B(3, 1) - A(1, 3) * B(0, 2) * B(2, 1) * B(3, 0) + A(2, 0) * B(0, 1) * B(1, 2) * B(3, 3) - A(2, 0) * B(0, 1) * B(1, 3) * B(3, 2) - A(2, 0) * B(0, 2) * B(1, 1) * B(3, 3) + A(2, 0) * B(0, 2) * B(1, 3) * B(3, 1) + A(2, 0) * B(0, 3) * B(1, 1) * B(3, 2) - A(2, 0) * B(0, 3) * B(1, 2) * B(3, 1) - A(2, 1) * B(0, 0) * B(1, 2) * B(3, 3) + A(2, 1) * B(0, 0) * B(1, 3) * B(3, 2) + A(2, 1) * B(0, 2) * B(1, 0) * B(3, 3) - A(2, 1) * B(0, 2) * B(1, 3) * B(3, 0) - A(2, 1) * B(0, 3) * B(1, 0) * B(3, 2) + A(2, 1) * B(0, 3) * B(1, 2) * B(3, 0) + A(2, 2) * B(0, 0) * B(1, 1) * B(3, 3) - A(2, 2) * B(0, 0) * B(1, 3) * B(3, 1) - A(2, 2) * B(0, 1) * B(1, 0) * B(3, 3) + A(2, 2) * B(0, 1) * B(1, 3) * B(3, 0) + A(2, 2) * B(0, 3) * B(1, 0) * B(3, 1) - A(2, 2) * B(0, 3) * B(1, 1) * B(3, 0) - A(2, 3) * B(0, 0) * B(1, 1) * B(3, 2) + A(2, 3) * B(0, 0) * B(1, 2) * B(3, 1) + A(2, 3) * B(0, 1) * B(1, 0) * B(3, 2) - A(2, 3) * B(0, 1) * B(1, 2) * B(3, 0) - A(2, 3) * B(0, 2) * B(1, 0) * B(3, 1) + A(2, 3) * B(0, 2) * B(1, 1) * B(3, 0) - A(3, 0) * B(0, 1) * B(1, 2) * B(2, 3) + A(3, 0) * B(0, 1) * B(1, 3) * B(2, 2) + A(3, 0) * B(0, 2) * B(1, 1) * B(2, 3) - A(3, 0) * B(0, 2) * B(1, 3) * B(2, 1) - A(3, 0) * B(0, 3) * B(1, 1) * B(2, 2) + A(3, 0) * B(0, 3) * B(1, 2) * B(2, 1) + A(3, 1) * B(0, 0) * B(1, 2) * B(2, 3) - A(3, 1) * B(0, 0) * B(1, 3) * B(2, 2) - A(3, 1) * B(0, 2) * B(1, 0) * B(2, 3) + A(3, 1) * B(0, 2) * B(1, 3) * B(2, 0) + A(3, 1) * B(0, 3) * B(1, 0) * B(2, 2) - A(3, 1) * B(0, 3) * B(1, 2) * B(2, 0) - A(3, 2) * B(0, 0) * B(1, 1) * B(2, 3) + A(3, 2) * B(0, 0) * B(1, 3) * B(2, 1) + A(3, 2) * B(0, 1) * B(1, 0) * B(2, 3) - A(3, 2) * B(0, 1) * B(1, 3) * B(2, 0) - A(3, 2) * B(0, 3) * B(1, 0) * B(2, 1) + A(3, 2) * B(0, 3) * B(1, 1) * B(2, 0) + A(3, 3) * B(0, 0) * B(1, 1) * B(2, 2) - A(3, 3) * B(0, 0) * B(1, 2) * B(2, 1) - A(3, 3) * B(0, 1) * B(1, 0) * B(2, 2) + A(3, 3) * B(0, 1) * B(1, 2) * B(2, 0) + A(3, 3) * B(0, 2) * B(1, 0) * B(2, 1) - A(3, 3) * B(0, 2) * B(1, 1) * B(2, 0);

    I_3 = A(0, 0) * A(1, 1) * B(2, 2) * B(3, 3) - A(0, 0) * A(1, 1) * B(2, 3) * B(3, 2) - A(0, 0) * A(1, 2) * B(2, 1) * B(3, 3) + A(0, 0) * A(1, 2) * B(2, 3) * B(3, 1) + A(0, 0) * A(1, 3) * B(2, 1) * B(3, 2) - A(0, 0) * A(1, 3) * B(2, 2) * B(3, 1) - A(0, 0) * A(2, 1) * B(1, 2) * B(3, 3) + A(0, 0) * A(2, 1) * B(1, 3) * B(3, 2) + A(0, 0) * A(2, 2) * B(1, 1) * B(3, 3) - A(0, 0) * A(2, 2) * B(1, 3) * B(3, 1) - A(0, 0) * A(2, 3) * B(1, 1) * B(3, 2) + A(0, 0) * A(2, 3) * B(1, 2) * B(3, 1) + A(0, 0) * A(3, 1) * B(1, 2) * B(2, 3) - A(0, 0) * A(3, 1) * B(1, 3) * B(2, 2) - A(0, 0) * A(3, 2) * B(1, 1) * B(2, 3) + A(0, 0) * A(3, 2) * B(1, 3) * B(2, 1) + A(0, 0) * A(3, 3) * B(1, 1) * B(2, 2) - A(0, 0) * A(3, 3) * B(1, 2) * B(2, 1) - A(0, 1) * A(1, 0) * B(2, 2) * B(3, 3) + A(0, 1) * A(1, 0) * B(2, 3) * B(3, 2) + A(0, 1) * A(1, 2) * B(2, 0) * B(3, 3) - A(0, 1) * A(1, 2) * B(2, 3) * B(3, 0) - A(0, 1) * A(1, 3) * B(2, 0) * B(3, 2) + A(0, 1) * A(1, 3) * B(2, 2) * B(3, 0) + A(0, 1) * A(2, 0) * B(1, 2) * B(3, 3) - A(0, 1) * A(2, 0) * B(1, 3) * B(3, 2) - A(0, 1) * A(2, 2) * B(1, 0) * B(3, 3) + A(0, 1) * A(2, 2) * B(1, 3) * B(3, 0) + A(0, 1) * A(2, 3) * B(1, 0) * B(3, 2) - A(0, 1) * A(2, 3) * B(1, 2) * B(3, 0) - A(0, 1) * A(3, 0) * B(1, 2) * B(2, 3) + A(0, 1) * A(3, 0) * B(1, 3) * B(2, 2) + A(0, 1) * A(3, 2) * B(1, 0) * B(2, 3) - A(0, 1) * A(3, 2) * B(1, 3) * B(2, 0) - A(0, 1) * A(3, 3) * B(1, 0) * B(2, 2) + A(0, 1) * A(3, 3) * B(1, 2) * B(2, 0) + A(0, 2) * A(1, 0) * B(2, 1) * B(3, 3) - A(0, 2) * A(1, 0) * B(2, 3) * B(3, 1) - A(0, 2) * A(1, 1) * B(2, 0) * B(3, 3) + A(0, 2) * A(1, 1) * B(2, 3) * B(3, 0) + A(0, 2) * A(1, 3) * B(2, 0) * B(3, 1) - A(0, 2) * A(1, 3) * B(2, 1) * B(3, 0) - A(0, 2) * A(2, 0) * B(1, 1) * B(3, 3) + A(0, 2) * A(2, 0) * B(1, 3) * B(3, 1) + A(0, 2) * A(2, 1) * B(1, 0) * B(3, 3) - A(0, 2) * A(2, 1) * B(1, 3) * B(3, 0) - A(0, 2) * A(2, 3) * B(1, 0) * B(3, 1) + A(0, 2) * A(2, 3) * B(1, 1) * B(3, 0) + A(0, 2) * A(3, 0) * B(1, 1) * B(2, 3) - A(0, 2) * A(3, 0) * B(1, 3) * B(2, 1) - A(0, 2) * A(3, 1) * B(1, 0) * B(2, 3) + A(0, 2) * A(3, 1) * B(1, 3) * B(2, 0) + A(0, 2) * A(3, 3) * B(1, 0) * B(2, 1) - A(0, 2) * A(3, 3) * B(1, 1) * B(2, 0) - A(0, 3) * A(1, 0) * B(2, 1) * B(3, 2) + A(0, 3) * A(1, 0) * B(2, 2) * B(3, 1) + A(0, 3) * A(1, 1) * B(2, 0) * B(3, 2) - A(0, 3) * A(1, 1) * B(2, 2) * B(3, 0) - A(0, 3) * A(1, 2) * B(2, 0) * B(3, 1) + A(0, 3) * A(1, 2) * B(2, 1) * B(3, 0) + A(0, 3) * A(2, 0) * B(1, 1) * B(3, 2) - A(0, 3) * A(2, 0) * B(1, 2) * B(3, 1) - A(0, 3) * A(2, 1) * B(1, 0) * B(3, 2) + A(0, 3) * A(2, 1) * B(1, 2) * B(3, 0) + A(0, 3) * A(2, 2) * B(1, 0) * B(3, 1) - A(0, 3) * A(2, 2) * B(1, 1) * B(3, 0) - A(0, 3) * A(3, 0) * B(1, 1) * B(2, 2) + A(0, 3) * A(3, 0) * B(1, 2) * B(2, 1) + A(0, 3) * A(3, 1) * B(1, 0) * B(2, 2) - A(0, 3) * A(3, 1) * B(1, 2) * B(2, 0) - A(0, 3) * A(3, 2) * B(1, 0) * B(2, 1) + A(0, 3) * A(3, 2) * B(1, 1) * B(2, 0) + A(1, 0) * A(2, 1) * B(0, 2) * B(3, 3) - A(1, 0) * A(2, 1) * B(0, 3) * B(3, 2) - A(1, 0) * A(2, 2) * B(0, 1) * B(3, 3) + A(1, 0) * A(2, 2) * B(0, 3) * B(3, 1) + A(1, 0) * A(2, 3) * B(0, 1) * B(3, 2) - A(1, 0) * A(2, 3) * B(0, 2) * B(3, 1) - A(1, 0) * A(3, 1) * B(0, 2) * B(2, 3) + A(1, 0) * A(3, 1) * B(0, 3) * B(2, 2) + A(1, 0) * A(3, 2) * B(0, 1) * B(2, 3) - A(1, 0) * A(3, 2) * B(0, 3) * B(2, 1) - A(1, 0) * A(3, 3) * B(0, 1) * B(2, 2) + A(1, 0) * A(3, 3) * B(0, 2) * B(2, 1) - A(1, 1) * A(2, 0) * B(0, 2) * B(3, 3) + A(1, 1) * A(2, 0) * B(0, 3) * B(3, 2) + A(1, 1) * A(2, 2) * B(0, 0) * B(3, 3) - A(1, 1) * A(2, 2) * B(0, 3) * B(3, 0) - A(1, 1) * A(2, 3) * B(0, 0) * B(3, 2) + A(1, 1) * A(2, 3) * B(0, 2) * B(3, 0) + A(1, 1) * A(3, 0) * B(0, 2) * B(2, 3) - A(1, 1) * A(3, 0) * B(0, 3) * B(2, 2) - A(1, 1) * A(3, 2) * B(0, 0) * B(2, 3) + A(1, 1) * A(3, 2) * B(0, 3) * B(2, 0) + A(1, 1) * A(3, 3) * B(0, 0) * B(2, 2) - A(1, 1) * A(3, 3) * B(0, 2) * B(2, 0) + A(1, 2) * A(2, 0) * B(0, 1) * B(3, 3) - A(1, 2) * A(2, 0) * B(0, 3) * B(3, 1) - A(1, 2) * A(2, 1) * B(0, 0) * B(3, 3) + A(1, 2) * A(2, 1) * B(0, 3) * B(3, 0) + A(1, 2) * A(2, 3) * B(0, 0) * B(3, 1) - A(1, 2) * A(2, 3) * B(0, 1) * B(3, 0) - A(1, 2) * A(3, 0) * B(0, 1) * B(2, 3) + A(1, 2) * A(3, 0) * B(0, 3) * B(2, 1) + A(1, 2) * A(3, 1) * B(0, 0) * B(2, 3) - A(1, 2) * A(3, 1) * B(0, 3) * B(2, 0) - A(1, 2) * A(3, 3) * B(0, 0) * B(2, 1) + A(1, 2) * A(3, 3) * B(0, 1) * B(2, 0) - A(1, 3) * A(2, 0) * B(0, 1) * B(3, 2) + A(1, 3) * A(2, 0) * B(0, 2) * B(3, 1) + A(1, 3) * A(2, 1) * B(0, 0) * B(3, 2) - A(1, 3) * A(2, 1) * B(0, 2) * B(3, 0) - A(1, 3) * A(2, 2) * B(0, 0) * B(3, 1) + A(1, 3) * A(2, 2) * B(0, 1) * B(3, 0) + A(1, 3) * A(3, 0) * B(0, 1) * B(2, 2) - A(1, 3) * A(3, 0) * B(0, 2) * B(2, 1) - A(1, 3) * A(3, 1) * B(0, 0) * B(2, 2) + A(1, 3) * A(3, 1) * B(0, 2) * B(2, 0) + A(1, 3) * A(3, 2) * B(0, 0) * B(2, 1) - A(1, 3) * A(3, 2) * B(0, 1) * B(2, 0) + A(2, 0) * A(3, 1) * B(0, 2) * B(1, 3) - A(2, 0) * A(3, 1) * B(0, 3) * B(1, 2) - A(2, 0) * A(3, 2) * B(0, 1) * B(1, 3) + A(2, 0) * A(3, 2) * B(0, 3) * B(1, 1) + A(2, 0) * A(3, 3) * B(0, 1) * B(1, 2) - A(2, 0) * A(3, 3) * B(0, 2) * B(1, 1) - A(2, 1) * A(3, 0) * B(0, 2) * B(1, 3) + A(2, 1) * A(3, 0) * B(0, 3) * B(1, 2) + A(2, 1) * A(3, 2) * B(0, 0) * B(1, 3) - A(2, 1) * A(3, 2) * B(0, 3) * B(1, 0) - A(2, 1) * A(3, 3) * B(0, 0) * B(1, 2) + A(2, 1) * A(3, 3) * B(0, 2) * B(1, 0) + A(2, 2) * A(3, 0) * B(0, 1) * B(1, 3) - A(2, 2) * A(3, 0) * B(0, 3) * B(1, 1) - A(2, 2) * A(3, 1) * B(0, 0) * B(1, 3) + A(2, 2) * A(3, 1) * B(0, 3) * B(1, 0) + A(2, 2) * A(3, 3) * B(0, 0) * B(1, 1) - A(2, 2) * A(3, 3) * B(0, 1) * B(1, 0) - A(2, 3) * A(3, 0) * B(0, 1) * B(1, 2) + A(2, 3) * A(3, 0) * B(0, 2) * B(1, 1) + A(2, 3) * A(3, 1) * B(0, 0) * B(1, 2) - A(2, 3) * A(3, 1) * B(0, 2) * B(1, 0) - A(2, 3) * A(3, 2) * B(0, 0) * B(1, 1) + A(2, 3) * A(3, 2) * B(0, 1) * B(1, 0);

    I_4 = A(0, 0) * A(1, 1) * A(2, 2) * B(3, 3) - A(0, 0) * A(1, 1) * A(2, 3) * B(3, 2) - A(0, 0) * A(1, 1) * A(3, 2) * B(2, 3) + A(0, 0) * A(1, 1) * A(3, 3) * B(2, 2) - A(0, 0) * A(1, 2) * A(2, 1) * B(3, 3) + A(0, 0) * A(1, 2) * A(2, 3) * B(3, 1) + A(0, 0) * A(1, 2) * A(3, 1) * B(2, 3) - A(0, 0) * A(1, 2) * A(3, 3) * B(2, 1) + A(0, 0) * A(1, 3) * A(2, 1) * B(3, 2) - A(0, 0) * A(1, 3) * A(2, 2) * B(3, 1) - A(0, 0) * A(1, 3) * A(3, 1) * B(2, 2) + A(0, 0) * A(1, 3) * A(3, 2) * B(2, 1) + A(0, 0) * A(2, 1) * A(3, 2) * B(1, 3) - A(0, 0) * A(2, 1) * A(3, 3) * B(1, 2) - A(0, 0) * A(2, 2) * A(3, 1) * B(1, 3) + A(0, 0) * A(2, 2) * A(3, 3) * B(1, 1) + A(0, 0) * A(2, 3) * A(3, 1) * B(1, 2) - A(0, 0) * A(2, 3) * A(3, 2) * B(1, 1) - A(0, 1) * A(1, 0) * A(2, 2) * B(3, 3) + A(0, 1) * A(1, 0) * A(2, 3) * B(3, 2) + A(0, 1) * A(1, 0) * A(3, 2) * B(2, 3) - A(0, 1) * A(1, 0) * A(3, 3) * B(2, 2) + A(0, 1) * A(1, 2) * A(2, 0) * B(3, 3) - A(0, 1) * A(1, 2) * A(2, 3) * B(3, 0) - A(0, 1) * A(1, 2) * A(3, 0) * B(2, 3) + A(0, 1) * A(1, 2) * A(3, 3) * B(2, 0) - A(0, 1) * A(1, 3) * A(2, 0) * B(3, 2) + A(0, 1) * A(1, 3) * A(2, 2) * B(3, 0) + A(0, 1) * A(1, 3) * A(3, 0) * B(2, 2) - A(0, 1) * A(1, 3) * A(3, 2) * B(2, 0) - A(0, 1) * A(2, 0) * A(3, 2) * B(1, 3) + A(0, 1) * A(2, 0) * A(3, 3) * B(1, 2) + A(0, 1) * A(2, 2) * A(3, 0) * B(1, 3) - A(0, 1) * A(2, 2) * A(3, 3) * B(1, 0) - A(0, 1) * A(2, 3) * A(3, 0) * B(1, 2) + A(0, 1) * A(2, 3) * A(3, 2) * B(1, 0) + A(0, 2) * A(1, 0) * A(2, 1) * B(3, 3) - A(0, 2) * A(1, 0) * A(2, 3) * B(3, 1) - A(0, 2) * A(1, 0) * A(3, 1) * B(2, 3) + A(0, 2) * A(1, 0) * A(3, 3) * B(2, 1) - A(0, 2) * A(1, 1) * A(2, 0) * B(3, 3) + A(0, 2) * A(1, 1) * A(2, 3) * B(3, 0) + A(0, 2) * A(1, 1) * A(3, 0) * B(2, 3) - A(0, 2) * A(1, 1) * A(3, 3) * B(2, 0) + A(0, 2) * A(1, 3) * A(2, 0) * B(3, 1) - A(0, 2) * A(1, 3) * A(2, 1) * B(3, 0) - A(0, 2) * A(1, 3) * A(3, 0) * B(2, 1) + A(0, 2) * A(1, 3) * A(3, 1) * B(2, 0) + A(0, 2) * A(2, 0) * A(3, 1) * B(1, 3) - A(0, 2) * A(2, 0) * A(3, 3) * B(1, 1) - A(0, 2) * A(2, 1) * A(3, 0) * B(1, 3) + A(0, 2) * A(2, 1) * A(3, 3) * B(1, 0) + A(0, 2) * A(2, 3) * A(3, 0) * B(1, 1) - A(0, 2) * A(2, 3) * A(3, 1) * B(1, 0) - A(0, 3) * A(1, 0) * A(2, 1) * B(3, 2) + A(0, 3) * A(1, 0) * A(2, 2) * B(3, 1) + A(0, 3) * A(1, 0) * A(3, 1) * B(2, 2) - A(0, 3) * A(1, 0) * A(3, 2) * B(2, 1) + A(0, 3) * A(1, 1) * A(2, 0) * B(3, 2) - A(0, 3) * A(1, 1) * A(2, 2) * B(3, 0) - A(0, 3) * A(1, 1) * A(3, 0) * B(2, 2) + A(0, 3) * A(1, 1) * A(3, 2) * B(2, 0) - A(0, 3) * A(1, 2) * A(2, 0) * B(3, 1) + A(0, 3) * A(1, 2) * A(2, 1) * B(3, 0) + A(0, 3) * A(1, 2) * A(3, 0) * B(2, 1) - A(0, 3) * A(1, 2) * A(3, 1) * B(2, 0) - A(0, 3) * A(2, 0) * A(3, 1) * B(1, 2) + A(0, 3) * A(2, 0) * A(3, 2) * B(1, 1) + A(0, 3) * A(2, 1) * A(3, 0) * B(1, 2) - A(0, 3) * A(2, 1) * A(3, 2) * B(1, 0) - A(0, 3) * A(2, 2) * A(3, 0) * B(1, 1) + A(0, 3) * A(2, 2) * A(3, 1) * B(1, 0) - A(1, 0) * A(2, 1) * A(3, 2) * B(0, 3) + A(1, 0) * A(2, 1) * A(3, 3) * B(0, 2) + A(1, 0) * A(2, 2) * A(3, 1) * B(0, 3) - A(1, 0) * A(2, 2) * A(3, 3) * B(0, 1) - A(1, 0) * A(2, 3) * A(3, 1) * B(0, 2) + A(1, 0) * A(2, 3) * A(3, 2) * B(0, 1) + A(1, 1) * A(2, 0) * A(3, 2) * B(0, 3) - A(1, 1) * A(2, 0) * A(3, 3) * B(0, 2) - A(1, 1) * A(2, 2) * A(3, 0) * B(0, 3) + A(1, 1) * A(2, 2) * A(3, 3) * B(0, 0) + A(1, 1) * A(2, 3) * A(3, 0) * B(0, 2) - A(1, 1) * A(2, 3) * A(3, 2) * B(0, 0) - A(1, 2) * A(2, 0) * A(3, 1) * B(0, 3) + A(1, 2) * A(2, 0) * A(3, 3) * B(0, 1) + A(1, 2) * A(2, 1) * A(3, 0) * B(0, 3) - A(1, 2) * A(2, 1) * A(3, 3) * B(0, 0) - A(1, 2) * A(2, 3) * A(3, 0) * B(0, 1) + A(1, 2) * A(2, 3) * A(3, 1) * B(0, 0) + A(1, 3) * A(2, 0) * A(3, 1) * B(0, 2) - A(1, 3) * A(2, 0) * A(3, 2) * B(0, 1) - A(1, 3) * A(2, 1) * A(3, 0) * B(0, 2) + A(1, 3) * A(2, 1) * A(3, 2) * B(0, 0) + A(1, 3) * A(2, 2) * A(3, 0) * B(0, 1) - A(1, 3) * A(2, 2) * A(3, 1) * B(0, 0);
}
void RoughCircleSolver::computePointsInPlane(const double &u, const double &v, const Eigen::Vector4d &plane, Eigen::Vector4d &point, int camera_id)
{
    /** compute the point in 3d which is intersected by a line and a plane.
     * the line is passed through the focal origin, and projected to the left image plane, with a coordinate (u, v) (in pixel)
     * 
    */
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

    Eigen::Matrix<double, 3, 4> A_full; //A_full * [x, y, z, 1]^t = 0
    A_full.row(0) = v * projection_matrix.row(2) - projection_matrix.row(1);
    A_full.row(1) = -u * projection_matrix.row(2) + projection_matrix.row(0);
    A_full.row(2) = plane;

    Eigen::Matrix3d A = A_full.block<3, 3>(0, 0);
    Eigen::Vector3d b = -A_full.col(3);

    point.head<3>() = A.inverse() * b;
    point(3) = 1;
}

bool RoughCircleSolver::computeCircle3D(const cv::RotatedRect &left_ellipse_box, const cv::RotatedRect &right_ellipse_box, Circle3D &circle)
{
    double I_2, I_3, I_4;
    Eigen::Matrix3d left_ellipse_quadratic, right_ellipse_quadratic;
    translateEllipse(left_ellipse_box, left_ellipse_quadratic);
    translateEllipse(right_ellipse_box, right_ellipse_quadratic);

    /*
    if(fabs(left_ellipse_box.angle - 65) < 0.01){
        cv::RotatedRect left_ellipse_box_changed = left_ellipse_box;
        left_ellipse_box_changed.angle = 59.2503;
        translateEllipse(left_ellipse_box_changed, left_ellipse_quadratic);
    }
    */

    //Eigen::Matrix<double, 3, 4> left_projection_matrix = stereo_cam_ptr->left_camera.projection_mat;
    //Eigen::Matrix<double, 3, 4> right_projection_matrix = stereo_cam_ptr->right_camera.projection_mat;
    //std::cout<<"\n\n\nleft_projection: \n"<<left_projection_matrix<<std::endl;
    //std::cout<<"right_projection: \n"<<right_projection_matrix<<std::endl;
    Eigen::Matrix4d A = stereo_cam_ptr->left_camera.projection_mat.transpose() * left_ellipse_quadratic * stereo_cam_ptr->left_camera.projection_mat;
    Eigen::Matrix4d B = stereo_cam_ptr->right_camera.projection_mat.transpose() * right_ellipse_quadratic * stereo_cam_ptr->right_camera.projection_mat;
    //std::cout << "left box: center: " << left_ellipse_box.center.x << " " << left_ellipse_box.center.y << " size: " << left_ellipse_box.size.width << " " << left_ellipse_box.size.height << " angle: " << left_ellipse_box.angle << std::endl;
    //std::cout << "right box: center: " << right_ellipse_box.center.x << " " << right_ellipse_box.center.y << " size: " << right_ellipse_box.size.width << " " << right_ellipse_box.size.height << " angle: " << right_ellipse_box.angle << std::endl;

    //std::cout<<"A: \n"<<A<<std::endl;
    //std::cout<<"B: \n"<<B<<std::endl;
    //std::cout<<"left ellipse: \n"<<left_ellipse_quadratic<<std::endl;
    //std::cout<<"right ellipse: \n"<<right_ellipse_quadratic<<std::endl;

    // calculate the circle's pose
    Eigen::Vector4d right_camera_origin = -stereo_cam_ptr->right_cam_transform_mat.col(3);
    right_camera_origin(3) = -right_camera_origin(3);

    computeI2I3I4Analytic(A, B, I_2, I_3, I_4);
    //std::cout<<"I2: "<<I_2<<" I3: "<<I_3<<" I4: "<<I_4<<std::endl;
    //computeI2I3I4(A, B, I_2, I_3, I_4);
    //std::cout<<"I2: "<<I_2<<" I3: "<<I_3<<" I4: "<<I_4<<std::endl;

    double lambda = -I_3 / (2 * I_2);
    Eigen::Matrix4d C = A + lambda * B;

    //std::cout<<"C: \n"<<C<<std::endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(C);

    if (eigensolver.info() != Eigen::Success)
    {
        std::cout << "failed to get the eigen values/vectors" << std::endl;
        return false;
    };
    Eigen::Vector4d eigen_values = eigensolver.eigenvalues(); //increasing order
    //std::cout<<"eigen "<<eigen_values<<std::endl;

    if (eigen_values(0) * eigen_values(3) >= 0)
        return false;

    // see if there are two zero eigen values (the eigen_value(1) and eigen_value(2) should be zero since eigen_values is presented in increasing order)
    double min_non_zero_value = std::min(fabs(eigen_values(0)), fabs(eigen_values(3)));
    if (fabs(eigen_values(1)) / min_non_zero_value > 0.5 || fabs(eigen_values(2)) / min_non_zero_value > 0.5)
        return false;

    //double res = I_3 * I_3 -4*I_2 * I_4;
    //if(fabs(res) > 0.01)
    //    return false;

    // find the plane of the circle
    Eigen::Vector4d p = std::sqrt(-eigen_values(0)) * eigensolver.eigenvectors().col(0) + std::sqrt(eigen_values(3)) * eigensolver.eigenvectors().col(3);
    if (p(3) * p.dot(right_camera_origin) < 0)
    {
        p = std::sqrt(-eigen_values(0)) * eigensolver.eigenvectors().col(0) - std::sqrt(eigen_values(3)) * eigensolver.eigenvectors().col(3);
    }
    
    // convention: p(3) > 0 , in our case, it can't be zero
    if (p(3) < 0)
    {
        p = -p;
    }

    //std::cout << "plane :" << p.transpose() << std::endl;
    // find the center of the circle
    // Here, we assume the center of the circle is also the center of the ellipse in the image.
    double center_x = left_ellipse_box.center.x;
    double center_y = left_ellipse_box.center.y;

    // x=a*cos(alpha)*cos(theta) - b*sin(alpha)*sin(theta) + x_c
    // y=a*sin(theta)*cos(alpha) + b*sin(alpha)*cos(theta) + y_c
    // where a = left_ellipse_box.size.width / 2.0
    //       b = left_ellipse_box.size.height/ 2.0

    // here, we simply choose a point with alpha=0
    double a = left_ellipse_box.size.width / 2.0;
    double p_x = a * cos(left_ellipse_box.angle) + center_x; // point on the ellipse
    double p_y = a * sin(left_ellipse_box.angle) + center_y;

    Eigen::Vector4d center_point_3d, circle_point_3d;
    computePointsInPlane(center_x, center_y, p, center_point_3d, LEFT_CAMERA);
    computePointsInPlane(p_x, p_y, p, circle_point_3d, LEFT_CAMERA);

    circle.radius = (center_point_3d - circle_point_3d).norm();
    circle.plane = p;
    circle.center = center_point_3d;
    circle.normalizePlane();
    return true;
}

bool RoughCircleSolver::areConcentric(const Circle3D a, const Circle3D b, double center_threshold)
{
    if ((a.center - b.center).norm() < center_threshold)
        return true;
    else
        return false;
}
void RoughCircleSolver::get3DCircles(const std::vector<EllipseWithScore> &left_possible_ellipses,
                                             const std::vector<EllipseWithScore> &right_possible_ellipses,
                                             const cv::Rect &left_rect_roi, const cv::Rect& right_rect_roi,
                                             std::vector<Circle3D> &circle3d_vec,
                                             std::vector<std::pair<int, int>> & valid_paired_index_vec,
                                             const cv::Mat *left_edge_ptr,
                                             const cv::Mat *right_edge_ptr,
                                             const double estimate_radius,
                                             const double radius_threshold)
{
    /**
     * pairs the ellipses from left/right camera and gives the circles in 3D coordinate.
     * 
    */
    if (left_possible_ellipses.size() == 0 || right_possible_ellipses.size() == 0)
        return;

    Eigen::Matrix4d A, B; //X^t * A * X =0, same for B

    //double error_threshold = 0.1;

    Eigen::Matrix<double, 3, 4> left_projection, right_projection;
    left_projection = stereo_cam_ptr->left_camera.projection_mat;
    right_projection = stereo_cam_ptr->right_camera.projection_mat;

    cv::Mat left_edge, right_edge;


    for (unsigned int i = 0; i < left_possible_ellipses.size(); i++)
    {
        for (unsigned int j = 0; j < right_possible_ellipses.size(); j++)
        {
            Circle3D circle_3d;
            Eigen::Matrix3d left_ellipse_quadratic, right_ellipse_quadratic;
            cv::RotatedRect left_ellipse_visualize = left_possible_ellipses[i].box;
            cv::RotatedRect right_ellipse_visualize = right_possible_ellipses[j].box;

            left_ellipse_visualize.center.x -= left_rect_roi.x;
            left_ellipse_visualize.center.y -= left_rect_roi.y;
            right_ellipse_visualize.center.x -= right_rect_roi.x;
            right_ellipse_visualize.center.y -= right_rect_roi.y;
            
            cv::cvtColor(*left_edge_ptr, left_edge, CV_GRAY2BGR);
            cv::cvtColor(*right_edge_ptr, right_edge, CV_GRAY2BGR);
            cv::ellipse(left_edge, left_ellipse_visualize, cv::Scalar(0, 0, 255), 1, CV_AA);
            cv::ellipse(right_edge,  right_ellipse_visualize, cv::Scalar(0, 0, 255), 1, CV_AA);

            if (!computeCircle3D(left_possible_ellipses[i].box, right_possible_ellipses[j].box, circle_3d))
                continue; // these two views cannot match

            if (fabs(circle_3d.radius - estimate_radius) < radius_threshold)
            {
                circle_3d.score = (left_possible_ellipses[i].box.size.width + left_possible_ellipses[i].box.size.height + 
                                    right_possible_ellipses[j].box.size.width + right_possible_ellipses[j].box.size.height                                
                                )/4.0;
                
                circle3d_vec.push_back(circle_3d);
                std::pair<int, int> paired_index(i, j);
                valid_paired_index_vec.push_back(paired_index);
            }
        }
    }

}

void RoughCircleSolver::getPossibleCircles(const cv::Mat &left_edge_roi, const cv::Mat &right_edge_roi,
                                           const cv::Rect &left_rect_roi, const cv::Rect& right_rect_roi,
                                           std::vector<Circle3D> &circles_3d,
                                           std::vector<StereoCircleValidContours> &stereo_valid_contours_in_roi_vec,
                                           std::vector<EllipseWithScore>& valid_left_ellipses_box, 
                                           std::vector<EllipseWithScore>& valid_right_ellipses_box,
                                           const double estimate_radius,
                                           const double radius_threshold)
{
    /** Main function of this class, get all the possible circles in the image.
     * @ param left_edge : the left edge image
     * @ param right_edge : the right edge image
     * @ param concentric_circles : the concentric circles in 3d space.
    */

    std::vector<EllipseWithScore> left_ellipses_box_vec, right_ellipses_box_vec; 
    std::vector<VecPointPtr> left_circle_contours_in_roi_vec, right_circle_contours_in_roi_vec;
    getPossibleEllipse(left_edge_roi, left_ellipses_box_vec, left_circle_contours_in_roi_vec);
    getPossibleEllipse(right_edge_roi, right_ellipses_box_vec, right_circle_contours_in_roi_vec);

    // convert the ellipse from roi coordinate to full image coordinate
    for(unsigned int i=0; i<left_ellipses_box_vec.size(); i++){
        left_ellipses_box_vec[i].box.center.x += left_rect_roi.x;
        left_ellipses_box_vec[i].box.center.y += left_rect_roi.y;
    }
    for(unsigned int i=0; i<right_ellipses_box_vec.size(); i++){
        right_ellipses_box_vec[i].box.center.x += right_rect_roi.x;
        right_ellipses_box_vec[i].box.center.y += right_rect_roi.y;
    }
    std::vector<std::pair<int, int> > valid_paired_index_vec;
    get3DCircles(left_ellipses_box_vec, right_ellipses_box_vec, left_rect_roi, right_rect_roi, circles_3d, valid_paired_index_vec,
                         &left_edge_roi, &right_edge_roi, estimate_radius, radius_threshold);
    for(unsigned int i=0; i<valid_paired_index_vec.size(); i++){
        int left_index = std::get<0>(valid_paired_index_vec[i]);
        int right_index = std::get<1>(valid_paired_index_vec[i]);
        StereoCircleValidContours stereo_valid_contours;
        stereo_valid_contours.left = left_circle_contours_in_roi_vec[left_index];
        stereo_valid_contours.right = right_circle_contours_in_roi_vec[right_index];
        stereo_valid_contours_in_roi_vec.push_back(stereo_valid_contours);

        valid_left_ellipses_box.push_back(left_ellipses_box_vec[left_index]);
        valid_right_ellipses_box.push_back(right_ellipses_box_vec[right_index]);
    }

}
