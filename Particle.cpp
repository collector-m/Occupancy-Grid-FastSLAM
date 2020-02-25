//
//  Particle.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 16.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include "Particle.h"
#include "Map.h"

using namespace std;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Particle::Particle() {
    
    int n_samples = 20;
    this->weight = 1.0;
    this->pose = Eigen::Array3f::Zero();
    this->last_pose = Eigen::Array3f::Zero();
    
    for (int i = 0; i < n_samples; i++){
        this->samples.push_back(Eigen::Vector3f::Zero());
        this->sample_measurement_estimates.push_back(Eigen::MatrixX2f::Zero(91, 2));
    }
    
}

Particle::Particle(float weight, Eigen::Vector3f pose) {
    
    this->weight = weight;
    this->pose = pose;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++ Print Summary ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Particle::summary(){
    
    cout << "Particle:" << endl;
    cout << "---------" << endl;
    cout << "Pose: x: " << this->pose(0) << "m | y: " << this->pose(1) << "m | theta: " << this->pose(2) << "rad" << endl;
    cout << "Weight: " << this->weight << endl;
    
}
