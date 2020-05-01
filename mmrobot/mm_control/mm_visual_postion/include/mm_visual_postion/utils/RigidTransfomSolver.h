#ifndef __RIGID_TRANSFORM_SOLVER_H__
#define __RIGID_TRANSFORM_SOLVER_H__
/*
    Input: two corresponding points' sets in two coordinate
    Ouput: the transformation matrix within these two coordinate
    This class uses an algorithm based on SVD, which can be found in the paper  
        “Least-Squares Fitting of Two 3-D Point Sets” by K.S Arun et. alii (1987 ),
    see also https://en.wikipedia.org/wiki/Kabsch_algorithm 
    Author: Chuan QIN
*/
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <opencv2/opencv.hpp>
class RigidTransformSolver{
private:
    Eigen::Matrix<double, 3, 3> H; //H = sum(qi * qi'^t) in the paper, which is the matrix that we will do SVD later
    Eigen::Vector3d translate;
    double scale;
    Eigen::Vector3d center_a, center_b;
public:
    RigidTransformSolver(){
        /*
        solve the transformation matrix from set_a to set_b
        */
    };
    RigidTransformSolver(const std::vector<Eigen::Vector3d>& set_a, const std::vector<Eigen::Vector3d>& set_b){
        setPointSets(set_a, set_b);
    }
    void setPointSets(const std::vector<Eigen::Vector3d>& set_a, const std::vector<Eigen::Vector3d>& set_b){
        std::cout<<"set_a: "<<std::endl;
        for(unsigned int i=0;i<set_a.size();i++){
            std::cout<<set_a[i].transpose()<<std::endl;
        }
        std::cout<<"set_b: "<<std::endl;
        for(unsigned int i=0;i<set_b.size();i++){
            std::cout<<set_b[i].transpose()<<std::endl;
        }

        computeCenter(set_a, center_a);
        computeCenter(set_b, center_b);
        H.setZero();
        double std_a = 0;
        double std_b = 0; //standard deviation 
        for(unsigned int i=0; i<set_a.size(); i++){
            Eigen::Vector3d set_a_zero_mean, set_b_zero_mean;
            set_a_zero_mean = set_a[i] - center_a;
            set_b_zero_mean = set_b[i] - center_b;
        
            // compute scale ratio
            std_a += set_a_zero_mean(0) * set_a_zero_mean(0)
                        + set_a_zero_mean(1) * set_a_zero_mean(1)
                        + set_a_zero_mean(2) * set_a_zero_mean(2);
            std_b += set_b_zero_mean(0) * set_b_zero_mean(0)
                        + set_b_zero_mean(1) * set_b_zero_mean(1)
                        + set_b_zero_mean(2) * set_b_zero_mean(2);
        }
        std_a = std::sqrt(std_a/set_a.size());
        std_b = std::sqrt(std_b/set_a.size());
        scale = std_a / std_b;


        for(unsigned int i=0; i<set_a.size(); i++){
            Eigen::Vector3d set_a_zero_mean, set_b_zero_mean;
            set_a_zero_mean = set_a[i] - center_a;
            set_b_zero_mean = set_b[i] - center_b;
            std::cout<<"set_a_zero_mean: "<<set_a_zero_mean.transpose()<<std::endl;
            std::cout<<"set_b_zero_mean: "<<set_b_zero_mean.transpose()<<std::endl;

            H = H + (scale*set_b_zero_mean) * set_a_zero_mean.transpose();
        }
    }

    void solveTransform(Eigen::Matrix4d& T_a_to_b, double& scale_ratio){
        // transform is from a to b.
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU|Eigen::ComputeThinV);
        //std::cout<<"singularValues: "<<svd.singularValues()<<std::endl;
        Eigen::Matrix3d rotation = svd.matrixV() * (svd.matrixU().transpose()); // do not count the scaling 
        // to count the scale situation, see algo: https://en.wikipedia.org/wiki/Kabsch_algorithm

        double det = rotation.determinant();
        Eigen::Matrix3d d;
        d.setIdentity();
        T_a_to_b.setIdentity();
        if(det < 0) // reflection case
        {
            d(2,2) = det;
            rotation = svd.matrixV() * d * (svd.matrixU().transpose());        
        }

        translate = center_a - rotation * center_b;
        T_a_to_b.block<3,3>(0,0) = rotation;
        T_a_to_b.block<3,1>(0,3) = translate;

        scale_ratio = scale;
    }

    void computeCenter(const std::vector<Eigen::Vector3d>& set, Eigen::Vector3d& center){
        center.setZero();
        for(unsigned int i=0; i<set.size(); i++){
            center = center + set[i];
        }
        center = center / set.size();
    }

};

class RigidRectTransformSolver:public RigidTransformSolver{
private:
    double width, height;
public:
    RigidRectTransformSolver(double rect_width, double rect_height)
        :width(rect_width), height(rect_height){};
    void setPointSets(const std::vector<Eigen::Vector3d>& set_real){
        std::vector<Eigen::Vector3d> set_rect(4);

        // same convention as in camera coordinate
        Eigen::Vector3d left_up_in_rect( -width / 2.0,  -height / 2.0, 0);
        Eigen::Vector3d right_up_in_rect( width / 2.0, -height / 2.0, 0);
        Eigen::Vector3d right_down_in_rect( width / 2.0,  height / 2.0, 0);
        Eigen::Vector3d left_down_in_rect( -width / 2.0, height / 2.0, 0);

        set_rect[0] = left_up_in_rect;
        set_rect[1] = right_up_in_rect;
        set_rect[2] = right_down_in_rect;
        set_rect[3] = left_down_in_rect;

        RigidTransformSolver::setPointSets(set_real, set_rect);
    }

    void setPointSets(const std::vector<cv::Point3d> & set_real){
        std::vector<Eigen::Vector3d> set_real_eigen(set_real.size());
        for(unsigned int i=0; i<set_real.size(); i++){
            set_real_eigen[i](0) = set_real[i].x;
            set_real_eigen[i](1) = set_real[i].y;
            set_real_eigen[i](2) = set_real[i].z;
        }
        setPointSets(set_real_eigen);
    }
    void solveTransform(Eigen::Matrix4d& T_real_to_rect, double& scale_ratio){
        RigidTransformSolver::solveTransform(T_real_to_rect, scale_ratio);
    }
    void solveTransform(Eigen::Vector3d& translate, Eigen::Quaterniond& q, double scale_ratio){
        Eigen::Matrix4d transform;
        solveTransform(transform, scale_ratio);
        translate = transform.block<3,1>(0,3);
        q = Eigen::Quaterniond(transform.block<3,3>(0,0));
    }

    void solveTransform(std::vector<double>& xyzabc, double& scale_ratio){
        Eigen::Matrix4d transform;
        solveTransform(transform, scale_ratio);
        Eigen::Vector3d angles = transform.block<3,3>(0,0).eulerAngles(0, 1, 2); 
        std::cout<<angles<<std::endl;
        std::cout<<transform.block<3,3>(0,0)<<std::endl;
        std::cout<<Eigen::Quaternionf(transform.block<3,3>(0,0).cast<float>()).coeffs()<<std::endl;
        xyzabc.resize(6);
        xyzabc[0] = transform(0,3);
        xyzabc[1] = transform(1,3);
        xyzabc[2] = transform(2,3);
        xyzabc[3] = angles(0);
        xyzabc[4] = angles(1);
        xyzabc[5] = angles(2);
    }
};



#endif //__SOLVE_RIGID_TRANSFORM_H__