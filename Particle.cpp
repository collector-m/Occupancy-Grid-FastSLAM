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
    
    this->weight = 0.;
    this->pose = Eigen::Array3f::Zero();
    
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
