/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "mm_visual_postion/switch/particularFilter.h"

using namespace std;

void ParticleFilter::init(int total_num_particules, Particle init_particle, StdPos std_pos,
                        std::shared_ptr<CoordinateTransformer> left_transformer_ptr_, std::shared_ptr<CoordinateTransformer> right_transformer_ptr_){
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).


    num_particles = total_num_particules; 
    left_transformer_ptr = left_transformer_ptr_;
    right_transformer_ptr = right_transformer_ptr_;
    weights.clear();
    particles.clear();
    weights.resize(num_particles);
    particles.resize(num_particles);

    // Normal distribution for x, y and theta
    normal_distribution<double> dist_x(init_particle.x, std_pos.std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(init_particle.y, std_pos.std_y);
    normal_distribution<double> dist_z(init_particle.z, std_pos.std_z);
    normal_distribution<double> dist_q_x(init_particle.q_x, std_pos.std_q);
    normal_distribution<double> dist_q_y(init_particle.q_y, std_pos.std_q);
    normal_distribution<double> dist_q_z(init_particle.q_z, std_pos.std_q);
    normal_distribution<double> dist_q_w(init_particle.q_w, std_pos.std_q);
    normal_distribution<double> dist_l(init_particle.l, std_pos.std_l);


    // create particles and set their values
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.z = dist_z(gen);
        do{
            p.q_x = dist_q_x(gen);
            p.q_y = dist_q_y(gen);
            p.q_z = dist_q_z(gen);
            p.q_w = dist_q_w(gen);
        }while(p.q_x * p.q_x + p.q_y * p.q_y + p.q_z * p.q_z + p.q_w * p.q_w > 1.0);
        p.normalizeQuaternion();
        p.l = dist_l(gen);
        p.weight = 1.0/num_particles;
        particles[i] = p;
        weights[i] = p.weight;
    }
    is_initialized = true;
}
void ParticleFilter::reset(){
	// reset the particular filter, and is_initialized will be false
    is_initialized = false;
}
void ParticleFilter::getCornersPosInObject(const double l, std::vector<Eigen::Vector4d>& corners_homo_pos_in_obj){
    
    
    Eigen::Vector4d left_up_in_square( -l / 2.0,  l / 2.0, 0, 1);
    Eigen::Vector4d right_up_in_square( l / 2.0,  l / 2.0, 0, 1);
    Eigen::Vector4d right_down_in_square( l / 2.0, - l / 2.0, 0, 1);
    Eigen::Vector4d left_down_in_square( -l / 2.0, - l / 2.0, 0, 1);

    corners_homo_pos_in_obj.resize(4);
    corners_homo_pos_in_obj[0] = left_up_in_square;
    corners_homo_pos_in_obj[1] = right_up_in_square;
    corners_homo_pos_in_obj[2] = right_down_in_square;
    corners_homo_pos_in_obj[3] = left_down_in_square;
}

void ParticleFilter::computeCornersPosInBase(Particle* p, std::vector<Eigen::Vector4d>& corners_homo_pos_in_base){
    std::vector<Eigen::Vector4d> corners_homo_pos_in_obj;
    getCornersPosInObject(p->l, corners_homo_pos_in_obj);
    left_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);
    right_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);
    corners_homo_pos_in_base.resize(4);
    for(int i=0; i<4; i++){
        left_transformer_ptr->getPosInBaseFromPoseInObject(corners_homo_pos_in_obj[i], corners_homo_pos_in_base[i]);
    }
}

void ParticleFilter::computeCornersPosInCamPixel(Particle* p, std::vector<cv::Point2d>& corners_pos_in_pixel){    
    std::vector<Eigen::Vector4d> corners_homo_pos_in_obj;
    getCornersPosInObject(p->l, corners_homo_pos_in_obj);
    left_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);
    right_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);
    
    corners_pos_in_pixel.resize(8);
    for(int i=0; i<4; i++){
        left_transformer_ptr->getPixelPosFromPosInObject(corners_homo_pos_in_obj[i], corners_pos_in_pixel[i*2]);
        right_transformer_ptr->getPixelPosFromPosInObject(corners_homo_pos_in_obj[i], corners_pos_in_pixel[i*2 + 1]);
    }
}

void ParticleFilter::computeCornersPosInCamPixel(Particle* p, std::vector<cv::Point2d>& corners_pos_in_left_img,  std::vector<cv::Point2d>& corners_pos_in_right_img){
    std::vector<Eigen::Vector4d> corners_homo_pos_in_obj;
    getCornersPosInObject(p->l, corners_homo_pos_in_obj);
    left_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);
    right_transformer_ptr->setTransBaseToObject(p->x, p->y, p->z, p->q_x, p->q_y, p->q_z, p->q_w);

    corners_pos_in_left_img.resize(4);
    corners_pos_in_right_img.resize(4);
    for(int i=0; i<4; i++){
        left_transformer_ptr->getPixelPosFromPosInObject(corners_homo_pos_in_obj[i], corners_pos_in_left_img[i]);
        right_transformer_ptr->getPixelPosFromPosInObject(corners_homo_pos_in_obj[i], corners_pos_in_right_img[i]);
    }
}
void ParticleFilter::prediction(StdPos std_pos) {
    //http://docs.ros.org/hydro/api/tf_conversions/html/c++/tf__eigen_8h.html
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i]; // get address of particle to update
        normal_distribution<double> dist_x(p->x, std_pos.std_x); // mean is centered around the new measurement
        normal_distribution<double> dist_y(p->y, std_pos.std_y);
        normal_distribution<double> dist_z(p->z, std_pos.std_z);
        normal_distribution<double> dist_q_x(p->q_x, std_pos.std_q);
        normal_distribution<double> dist_q_y(p->q_y, std_pos.std_q);
        normal_distribution<double> dist_q_z(p->q_z, std_pos.std_q);
        normal_distribution<double> dist_q_w(p->q_w, std_pos.std_q);
        normal_distribution<double> dist_l(p->l, std_pos.std_l);

        p->x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p->y = dist_y(gen);
        p->z = dist_z(gen);

        do{
            p->q_x = dist_q_x(gen);
            p->q_y = dist_q_y(gen);
            p->q_z = dist_q_z(gen);
            p->q_w = dist_q_w(gen);
        }
        while(p->q_x * p->q_x + p->q_y * p->q_y
                    + p->q_z * p->q_z + p->q_w * p->q_w > 1.0);
        p->normalizeQuaternion();
        p->l = dist_l(gen);
    }
}


void ParticleFilter::updateWeights(StdPixel std_pixel, std::vector<cv::Point2d> pixel_pos) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html


    double weights_sum = 0;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i];
        std::vector<cv::Point2d> corners_homo_pos_in_pixel;
        computeCornersPosInCamPixel(p, corners_homo_pos_in_pixel);

        double wt = 1.0;

        // convert observation from vehicle's to map's coordinate system
        double tmp = 0.0;
        for(unsigned int j=0; j<corners_homo_pos_in_pixel.size(); ++j){
            
            // update weights using Multivariate Gaussian Distribution
            // equation given in Transformations and Associations Quiz
            tmp += pow((corners_homo_pos_in_pixel[j].x - pixel_pos[j].x), 2) / pow(std_pixel.std_x, 2) 
                                    + pow((corners_homo_pos_in_pixel[j].y - pixel_pos[j].y), 2) / pow(std_pixel.std_y, 2);
        }
        wt = exp(-0.5 * tmp);
        weights_sum += wt + 0.000000001;
        p->weight = wt + 0.000000001;
    }
    std::cout<<"weights_sum: "<<weights_sum<<std::endl;
    // normalize weights to bring them in (0, 1]
        for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        if(p->weight <= 0.000000002) p->need_reset = true;
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

void ParticleFilter::resample(const Particle& possible_particle, const StdPos& std_pos) {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles(particles.size());


    normal_distribution<double> dist_x(possible_particle.x, std_pos.std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(possible_particle.y, std_pos.std_y);
    normal_distribution<double> dist_z(possible_particle.z, std_pos.std_z);
    normal_distribution<double> dist_q_x(possible_particle.q_x, std_pos.std_q);
    normal_distribution<double> dist_q_y(possible_particle.q_y, std_pos.std_q);
    normal_distribution<double> dist_q_z(possible_particle.q_z, std_pos.std_q);
    normal_distribution<double> dist_q_w(possible_particle.q_w, std_pos.std_q);
    normal_distribution<double> dist_l(possible_particle.l, std_pos.std_l);

    for (int i = 0; i < num_particles; i++){
        resampled_particles[i] = particles[distribution(gen)];
        
        if(resampled_particles[i].need_reset){
            resampled_particles[i].x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
            resampled_particles[i].y = dist_y(gen);
            resampled_particles[i].z = dist_z(gen);
            do{
                resampled_particles[i].q_x = dist_q_x(gen);
                resampled_particles[i].q_y = dist_q_y(gen);
                resampled_particles[i].q_z = dist_q_z(gen);
                resampled_particles[i].q_w = dist_q_w(gen);
            }
            while(resampled_particles[i].q_x * resampled_particles[i].q_x + resampled_particles[i].q_y * resampled_particles[i].q_y
                     + resampled_particles[i].q_z * resampled_particles[i].q_z + resampled_particles[i].q_w * resampled_particles[i].q_w > 1.0);
            resampled_particles[i].normalizeQuaternion();
            resampled_particles[i].l = dist_l(gen);
        
        }
    }

    particles = resampled_particles;
}

void ParticleFilter::getResult(Particle& result_particle) {
    result_particle.x = 0;
    result_particle.y = 0;
    result_particle.z = 0;
    result_particle.q_x = 0;
    result_particle.q_y = 0;
    result_particle.q_z = 0;
    result_particle.q_w = 0;
    result_particle.l = 0;
    for(unsigned int i=0; i< particles.size(); i++){
        result_particle.x += particles[i].x * particles[i].weight;
        result_particle.y += particles[i].y * particles[i].weight;
        result_particle.z += particles[i].z * particles[i].weight;
        result_particle.q_x += particles[i].q_x * particles[i].weight;
        result_particle.q_y += particles[i].q_y * particles[i].weight;
        result_particle.q_z += particles[i].q_z * particles[i].weight;
        result_particle.q_w += particles[i].q_w * particles[i].weight;
        result_particle.l += particles[i].l * particles[i].weight;
    } 
    result_particle.normalizeQuaternion();

    double rot_x, rot_y, rot_z;
    result_particle.toEulerAngles(rot_x, rot_y, rot_z);
    
    left_transformer_ptr->setTransBaseToObject(result_particle.x, result_particle.y, result_particle.z, rot_x, rot_y, rot_z);
    right_transformer_ptr->setTransBaseToObject(result_particle.x, result_particle.y, result_particle.z, rot_x, rot_y, rot_z);
    left_transformer_ptr->pubCamToObjTransform();
}
