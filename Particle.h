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
        Map& getMap(){ return this->map; };
        float& getWeight(){ return this->weight; };
        Eigen::MatrixX2f& getMeasurementEstimate(){ return this->measurement_estimate; };

    private:
        float weight; // particle's current weight
        Eigen::Vector3f pose; // particle's pose (x, y, thetha)
        Map map; // particle's estimated grid map of the environment
        Eigen::MatrixX2f measurement_estimate; // array of measurement values
    
};

#endif /* Particle_h */
