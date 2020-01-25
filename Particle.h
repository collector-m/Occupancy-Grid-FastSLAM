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

// function which generates unique ID for each particle
int generateID();

class Particle{
    
    public:
        // constructor and destructor
        Particle();
        Particle(float weight, Eigen::Array3f pose);
        ~Particle(){};
    
        // print summary of particle
        void summary();
    
        // getter functions
        Eigen::Array3f& getPose(){ return this->pose; };
        const int& getID(){ return this->ID; };
        Map& getMap(){ return this->map; };
        float& getWeight(){ return this->weight; };
        Eigen::MatrixX2f& getMeasurementEstimate(){ return this->measurement_estimate; };


    private:
        int ID; // unique particle ID
        float weight; // particle's current weight
        Eigen::Array3f pose; // particle's pose (x, y, thetha)
        Map map; // particle's estimated grid map of the environment
        Eigen::MatrixX2f measurement_estimate; // array of measurement values
    
};

#endif /* Particle_h */
