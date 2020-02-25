//
//  RBPF.h
//  FastSLAM
//
//  Created by Mats Steinweg on 20.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef RBPF_h
#define RBPF_h

#include <list>
#include <Eigen/Dense>

#include "Particle.h"
#include "Sensor.h"
#include "ScanMatcher.h"
#include "Map.h"

class Robot;

using namespace std;

class RBPF {
    
    public:
        // Constructor and destructor
        RBPF();
        RBPF(int n_particles, Eigen::Vector3f R, int max_iterations, float tolerance, float discard_fraction);
        ~RBPF(){};
        
        // Summary of RBPF
        void summary();
        
        // Functionality of the particle filter
        void predict(const float& v, const float& omega, const float& current_timestamp);
        void sweep_estimate(Sensor& sensor);
        void mapping(Sensor& sensor);
        void run(Robot &robot, Eigen::Vector2f odometry_signal, const int simulation_mode);
        int inverse_sensor_model(const int& x, const int& y, Eigen::Vector3f& image_pose, Sensor& sensor);
        void improved_proposal(Sensor& sensor, Eigen::Vector2f odometry_signal, float current_timestamp);
        void scan_matching(const Eigen::Vector3f &pose, Sensor& sensor);
        void weight(Sensor& sensor);
        void resample();
        double motion_model_velocity(Eigen::Vector3f particle_pose, Eigen::Vector3f sample_pose, Eigen::Vector2f odometry_signal, float current_timestamp);
        
        // Getter functions
        Map& getMap();
        list<Particle>& getParticles(){ return this->particles; };
        const int getN(){ return this->n_particles; };
        const Eigen::Vector3f getR(){ return this->R; };
        ScanMatcher& getScanMatcher(){ return this->scan_matcher; };
        float& getLastTimestamp(){ return this->last_timestamp; };
        
        // Setter functions
        void setScanMatcher(ScanMatcher& scan_matcher){ this->scan_matcher = scan_matcher; };
        
    private:
        float last_timestamp;
        list<Particle> particles;
        int n_particles;
        Eigen::Vector3f R;
        ScanMatcher scan_matcher;
    
};


#endif /* RBPF_h */
