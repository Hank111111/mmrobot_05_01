/*
 * particle_filter.h
 *
 * 2D particle filter class.
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include <vector>
#include <string>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "mm_visual_postion/utils/CoordinateTransformer.h"
class Particle {
public:
	int id;
	double x;
	double y;
	double z;
	double q_x; // Quaternion
    double q_y;
    double q_z;
	double q_w;
    double l; //length of the square
    double weight;
	bool need_reset;
    Particle():id(0), x(0), y(0), z(0), q_x(0), q_y(0), q_z(0), q_w(0), l(0), weight(0),need_reset(false){};
    Particle(int id, double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double l, double weight)
        :id(id), x(x), y(y), z(z), q_x(q_x), q_y(q_y), q_w(q_w), l(l), weight(weight),need_reset(false){};
	void setTranslation(double x_, double y_, double z_){
		x = x_;
		y = y_;
		z = z_;
	}
	void fromEulerAngles(double rot_x, double rot_y, double rot_z){
		Eigen::Quaterniond q;
		q = Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX())
    		* Eigen::AngleAxisd(rot_y, Eigen::Vector3d::UnitY())
    		* Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ());
		q_x = q.x();
		q_y = q.y();
		q_z = q.z();
		q_w = q.w();
	}
	void toEulerAngles(double& rot_x, double& rot_y, double& rot_z){
		Eigen::Quaterniond q(q_x, q_y, q_z, q_w);
		Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		rot_x = euler(0);
		rot_y = euler(1);
		rot_z = euler(2);

	}
	void normalizeQuaternion(){
		double norm = sqrt(q_x * q_x + q_y * q_y + q_z * q_z + q_w * q_w);
		q_x = q_x / norm;
		q_y = q_y / norm;
		q_z = q_z / norm;
		q_w = q_w / norm;
	}

};

class StdPos { //standard deviation 
public:
    double std_x;
    double std_y;
    double std_z;
    double std_q;
    double std_l;
    
    StdPos(double std_x, double std_y, double std_z, double std_q, double std_l)
        :std_x(std_x), std_y(std_y), std_z(std_z), std_q(std_q), std_l(std_l){}
    StdPos():std_x(0), std_y(0), std_z(0), std_q(1), std_l(0){}


};

class StdPixel{
public:
    double std_x;
    double std_y;
    StdPixel(double std_x, double std_y):std_x(std_x), std_y(std_y){};
    StdPixel():std_x(0), std_y(0){};
};

class ParticleFilter {
	
	// Number of particles to draw
	int num_particles; 
	
	
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;

	std::shared_ptr<CoordinateTransformer> left_transformer_ptr, right_transformer_ptr;
	std::random_device rd;
    std::default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param M Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false), gen(rd()) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
    void init(int total_num_particules, Particle init_particle, StdPos std_pos, 
				std::shared_ptr<CoordinateTransformer> left_transformer_ptr_, std::shared_ptr<CoordinateTransformer> right_transformer_ptr_);
	void reset();
	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
    void prediction(StdPos std_pos);	

    void computeCornersPosInBaseCoord(Particle* p,  std::vector<Eigen::Vector4d>& corners_homo_pos_in_base);
	void computeCornersPosInCamPixel(Particle* p, std::vector<cv::Point2d>& corners_pos_in_pixel);
	void computeCornersPosInCamPixel(Particle* p, std::vector<cv::Point2d>& corners_pos_in_left_img,  std::vector<cv::Point2d>& corners_pos_in_right_img);
	void computeCornersPosInBase(Particle* p, std::vector<Eigen::Vector4d>& corners_homo_pos_in_base);
	void getCornersPosInObject(const double l, std::vector<Eigen::Vector4d>& corners_homo_pos_in_obj);

	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(StdPixel std_pixel, std::vector<cv::Point2d> pixel_pos);
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample(const Particle& possible_particle, const StdPos& std_pos);	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}
	void getResult(Particle& result_particle);

};



#endif /* PARTICLE_FILTER_H_ */