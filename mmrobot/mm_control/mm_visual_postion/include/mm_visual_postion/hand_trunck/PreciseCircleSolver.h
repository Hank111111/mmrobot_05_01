#ifndef __PRECISE_CIRCLE_SOLVER_H__
#define __PRECISE_CIRCLE_SOLVER_H__
#include "BaseCircleSolver.h"
#include "CircleModel.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "mm_visual_postion/utils/TrackBar.h"


#define INNER_CIRCLE 0
#define OUTER_CIRCLE 1
class CircleParams{
public:
    double quaternion[4]; //q_x,q_y,q_z,q_w
    double rest[4];//x,y,z,radius
};
void normalize(Eigen::Vector3d &v)
{
    v(0) /= v(2);
    v(1) /= v(2);
    v(2) = 1.0;
}
void translateParam(const EigenVector8d &params, CircleParams& circle_params){
    for (int i = 3; i < 7; i++)
    {
        circle_params.quaternion[i-3] = params(i);
    }
    for (int i = 0; i < 3; i++)
    {
        circle_params.rest[i] = params(i); //x,y,z
    }
    circle_params.rest[3] = params(7); //radius
}

void translateParam(const CircleParams& circle_params, EigenVector8d &params){
    for (int i = 3; i < 7; i++)
    {
        params(i) = circle_params.quaternion[i-3];
    }
    for (int i = 0; i < 3; i++)
    {
        params(i) = circle_params.rest[i]; //x,y,z
    }
    params(7) = circle_params.rest[3]; //radius
}
void translateParam(const EigenVector8d &param, Eigen::Matrix4d &transform)
{
    transform.setIdentity();
    Eigen::Quaterniond quaternion(param(6), param(3), param(4), param(5)); // qw,qx,qy,qz
    Eigen::Matrix3d rot = quaternion.toRotationMatrix();
    transform.block<3, 3>(0, 0) = rot;
    transform.block<3, 1>(0, 3) = param.segment(0, 3);
}

void translateParam(const double a, const double b, const double c, Eigen::Matrix3d &rot_mat)
{
    rot_mat = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
void translateParam(const Eigen::Matrix3d &rot_mat, double &a, double &b, double &c)
{
    Eigen::Vector3d euler_angle = rot_mat.eulerAngles(0, 1, 2);
    a = euler_angle(0);
    b = euler_angle(1);
    c = euler_angle(2);
}

inline static void getReprojectPoint(const Eigen::Matrix<double, 3, 4> &projection_mat, const EigenVector8d &param,
                                     const Eigen::Vector4d &point_in_plane, Eigen::Vector3d &project_point)
{
    Eigen::Matrix4d transform;
    translateParam(param, transform);

    project_point = projection_mat * transform * point_in_plane;
    normalize(project_point);
}

class PointToEllipseDistance: public ceres::SizedCostFunction<1/* number of residuals */,
                                                             4/* size of first parameter(quaternion) */,
                                                             5/* size of second parameter(the rest) */>
{
public:
    PointToEllipseDistance( const cv::Rect& rect_roi_input,
                            const VecPoint& points_input, int fit_circle_id, 
                            const Eigen::Matrix<double, 3, 4> &projection_mat_input)
        : circle_id(fit_circle_id), projection_mat(projection_mat_input), points(points_input), 
        rect_roi(rect_roi_input)
    {

    };
    virtual ~PointToEllipseDistance() {}
    class Func_0{
        // euler to transformation matrix
    private:
        double q_x, q_y, q_z, q_w;
    public:
        void forward(const EigenVector8d& param, Eigen::Matrix4d& transform);
        void backward(const Eigen::Matrix4d& deriv_transform, EigenVector8d& deriv_param);
    };
    class Func_1{
    private:
        const Eigen::Matrix<double,3,4>* A_ptr;
        // Y=A*X
    public:
        void forward(const Eigen::Matrix<double,3,4>* A_ptr_input, Eigen::Matrix4d& X, Eigen::Matrix<double, 3,4>& Y);
        void backward(const Eigen::Matrix<double, 3,4>& deriv_Y, Eigen::Matrix4d& deriv_X);
    };
    class Func_2{
        // Y = X*B
    private:
        const Eigen::Matrix<double, 4, 3>* B_ptr;
    public:
        void forward(const Eigen::Matrix<double, 4, 3>* B_ptr_input, Eigen::Matrix<double,3,4>& X, Eigen::Matrix<double,3,3>& Y);
        void backward(const Eigen::Matrix<double,3,3>& deriv_Y, Eigen::Matrix<double, 3, 4>& deriv_X);
    };

    class Func_3{
        // Y = X.inverse()
        // https://math.stackexchange.com/questions/1471825/derivative-of-the-inverse-of-a-matrix
    private:
        Eigen::Matrix3d Y_local;
    public:
        void forward(const Eigen::Matrix3d& X, Eigen::Matrix3d& Y);
        void backward(const Eigen::Matrix3d& deriv_Y, Eigen::Matrix3d& deriv_X);
        void testDerivate(const Eigen::Matrix3d& X);
    };

    class Func_4{
        //Y=X*V, where Y, V are vectors, X is a matrix
    private:
        Eigen::Vector3d V;
    public:
        void forward(const Eigen::Vector3d& V_input, const Eigen::Matrix3d X, Eigen::Vector3d& Y);
        void backward(const Eigen::Vector3d& deriv_Y, Eigen::Matrix3d& deriv_X);
    };


    class Func_5{
        // normalize [u*w, v*w, w] to [u, v, 1]
    private:
        double u,v,w;
    public:
        void forward(const Eigen::Vector3d& X, Eigen::Vector3d& X_normalized);
        void backward(const Eigen::Vector3d& deriv_X_normalized, Eigen::Vector3d& deriv_X);
        void testDerivate(const Eigen::Vector3d& X);
    };

    class Func_6{
        // res = x^2 / r^2 + y^2/r^2 -1
    private:
        double x, y, r, rr;
        
    public:
        void forward(double x_input, double y_input, double r_input, double& res);
        void backward(double deriv_res, double& deriv_x, double& deriv_y, double& deriv_r);
        void testDerivate(double x_input, double y_input, double r_input);
    };
    
    virtual bool Evaluate(double const* const* x,
                                double* residuals,
                                double** jacobians) const ;

private:
    int circle_id;
    const Eigen::Matrix<double, 3, 4> projection_mat;
    const VecPoint& points;

    const cv::Rect& rect_roi;
};

class PreciseCircleSolverImpl
{
private:
    int n_point;
    cv::Mat * left_img_ptr;
    cv::Mat * right_img_ptr;
    cv::Rect left_rect_roi, right_rect_roi;
public:
    Eigen::Matrix<double, 3, 4> left_projection, right_projection;
    StereoCircleValidContours stereo_valid_contours;
    PreciseCircleSolverImpl(){};

    void init(const cv::Rect& left_rect_roi_input, const cv::Rect& right_rect_roi_input,
              const StereoCircleValidContours& stereo_valid_contours_input,
              const Eigen::Matrix<double, 3, 4> &left_projection_input, const Eigen::Matrix<double, 3, 4> &right_projection_input,
              cv::Mat* left_img_ptr_input=nullptr, cv::Mat* right_img_ptr_input=nullptr);
    void getInitCircleParams(const Circle3D &init_circle, CircleParams *init_params);
    void buildBlocks(ceres::Problem& problem, ceres::LossFunction* loss, const VecPoint& contours_point, int circle_id, 
                    const cv::Rect& rect_roi, const Eigen::Matrix<double,3,4>& projection_mat, CircleParams *circle_params);
    void buildProblem(ceres::Problem &problem, CircleParams *circle_params, double robust_threshold);
    void solveProblem(ceres::Problem &problem, CircleParams *circle_params_ptr, Circle3D &result, double& final_cost_per_point);
};


class TestLoss:public BaseCircleSolver{
private:
    cv::Mat left_image, right_image;
    double x_origin, y_origin, z_origin, a_origin, b_origin, c_origin, r_i_origin, r_o_origin;
    double xyz_ratio, abc_ratio, r_ratio;
    Eigen::Matrix3d init_rot;
    std::vector<std::shared_ptr<PointToEllipseDistance> > distance_ptr;
    ceres::Problem* problem_ptr;
    CircleParams* circle_params_ptr;
public:
    TrackBar x_bar, y_bar, z_bar, a_bar, b_bar, c_bar, r_inner_bar, r_outer_bar;
    TestLoss(cv::Mat left_image_input, cv::Mat right_image_input,
            StereoCircleValidContours& stereo_valid_contours,
            double x_default, double y_default, double z_default,
            double a_default, double b_default, double c_default,
            double r_i_default, double r_o_default,
            std::shared_ptr<StereoCameraArmModel>& stereo_cam_ptr):BaseCircleSolver(stereo_cam_ptr){
        left_image = left_image_input.clone();
        right_image = right_image_input.clone();
        cv::namedWindow("left", 0);
        cv::namedWindow("right", 0);
        cv::namedWindow("params", 0);

        translateParam(a_default, b_default, c_default, init_rot);

        x_origin = x_default - 10;
        y_origin = y_default - 10;
        z_origin = z_default - 10;
        a_origin = -1.0;
        b_origin = -1.0;
        c_origin = -1.0;
        r_i_origin = r_i_default - 5;
        r_o_origin = r_o_default - 5;
        
        r_ratio = 0.01;
        xyz_ratio = 0.01;
        abc_ratio = 0.001;
        x_bar.setTrackBar("params", "x", 10.0/xyz_ratio, 20.0/xyz_ratio, onTrackBar, reinterpret_cast<void*>(this));
        y_bar.setTrackBar("params", "y", 10.0/xyz_ratio, 20.0/xyz_ratio, onTrackBar, reinterpret_cast<void*>(this));
        z_bar.setTrackBar("params", "z", 10.0/xyz_ratio, 20.0/xyz_ratio, onTrackBar, reinterpret_cast<void*>(this));
        a_bar.setTrackBar("params", "a", 1.0/abc_ratio, 2.0/abc_ratio, onTrackBar, reinterpret_cast<void*>(this));
        b_bar.setTrackBar("params", "b", 1.0/abc_ratio, 2.0/abc_ratio, onTrackBar, reinterpret_cast<void*>(this));
        c_bar.setTrackBar("params", "c", 1.0/abc_ratio, 2.0/abc_ratio, onTrackBar, reinterpret_cast<void*>(this));
        r_inner_bar.setTrackBar("params", "r_i", 5.0/r_ratio, 10.0/r_ratio, onTrackBar, reinterpret_cast<void*>(this));
        r_outer_bar.setTrackBar("params", "r_o", 5.0/r_ratio, 10.0/r_ratio, onTrackBar, reinterpret_cast<void*>(this));
     
    }
    void setProblem(ceres::Problem* problem_ptr_input, CircleParams* ceres_circle_params_ptr){
        problem_ptr = problem_ptr_input;
        circle_params_ptr = ceres_circle_params_ptr;
        visualize();
        cv::waitKey(0);   
    }
    
    void visualize();
    void trackbarCallback(int v){
        visualize();
    }
    static void onTrackBar(int v, void* ptr){
        reinterpret_cast<TestLoss*>(ptr)->trackbarCallback(v);
    }
};

class PreciseCirclesSolver : public BaseCircleSolver
{
private:
    PreciseCircleSolverImpl solver_impl;
    CircleParams circle_params;
    std::shared_ptr<TestLoss> test_loss;
public:
    PreciseCirclesSolver(std::shared_ptr<StereoCameraArmModel> &stereo_cam_ptr_) : BaseCircleSolver(stereo_cam_ptr_){};
    void init(  const cv::Rect& left_rect_roi_input, const cv::Rect& right_rect_roi_input,
                const Circle3D &init_circle, StereoCircleValidContours& stereo_valid_contours_input,
                cv::Mat * left_img_ptr=nullptr, cv::Mat * right_img_ptr=nullptr)
    {
        solver_impl.init(left_rect_roi_input, right_rect_roi_input, stereo_valid_contours_input, stereo_cam_ptr->left_camera.projection_mat, stereo_cam_ptr->right_camera.projection_mat, left_img_ptr, right_img_ptr);
        solver_impl.getInitCircleParams(init_circle, &circle_params);
        //test_loss = std::make_shared<TestLoss>(*left_img_ptr, *right_img_ptr, stereo_valid_contours_input, circle_params[0], circle_params[1], circle_params[2],
        //        circle_params[3],circle_params[4],circle_params[5],circle_params[6],circle_params[7],stereo_cam_ptr);

    }
    void solve(Circle3D &result, double robust_threshold, double& final_cost_per_point)
    {
        
        ceres::Problem problem;
        solver_impl.buildProblem(problem, &circle_params, robust_threshold);
        //test_loss->setProblem(&problem, circle_params);
        solver_impl.solveProblem(problem, &circle_params, result, final_cost_per_point);
        //std::cout<<"final loss: "<<test_loss->computeLoss(circle_params)<<std::endl;
    }
};


#endif