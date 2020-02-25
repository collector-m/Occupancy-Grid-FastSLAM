//
//  Particle.h
//  FastSLAM
//
//  Created by Mats Steinweg on 07.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Particle_h
#define Particle_h

#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "Map.h"

using namespace std;


class Particle{
    
    public:
        // Constructor and destructor
        Particle();
        Particle(float weight, Eigen::Vector3f pose);
        ~Particle(){};
    
        // Print particle summary
        void summary();
    
        // Getter functions
        Eigen::Vector3f& getPose(){ return this->pose; };
        Eigen::Vector3f& getLastPose(){ return this->last_pose; };
        Eigen::Vector3f getPoseCopy(){ return this->pose; };
        Map& getMap(){ return this->map; };
        double& getWeight(){ return this->weight; };
        Eigen::MatrixX2f& getMeasurementEstimate(){ return this->measurement_estimate; };
        vector<Eigen::Vector3f>& getSamples(){ return this->samples; };
        vector<Eigen::MatrixX2f>& getSampleMeasurementEstimates(){ return this->sample_measurement_estimates; };
    
        void setLastPose(Eigen::Vector3f pose){ this->last_pose = pose; };

    private:
        double weight; // particle's current weight
        Eigen::Vector3f pose; // particle's pose (x, y, thetha)
        Eigen::Vector3f last_pose;
        Map map; // particle's estimated grid map of the environment
        Eigen::MatrixX2f measurement_estimate; // array of measurement values
        vector<Eigen::Vector3f> samples; // samples around the reported scan-matching pose
        vector<Eigen::MatrixX2f> sample_measurement_estimates; // measurement estimates of the samples
    
};

#endif /* Particle_h */
