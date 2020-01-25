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

// generate unique particle ID
int generateID(){
    static int id = 0;
    return ++id;
}


// constructor
Particle::Particle() {
    
    // set member variables
    this->ID = generateID();
    this->weight = 0.2;
    this->pose = Eigen::Array3f::Zero();
    
}


Particle::Particle(float weight, Eigen::Array3f pose) {
    
    // set member variables
    this->ID = generateID();
    this->weight = weight;
    this->pose = pose;
}


// print summary of particle
void Particle::summary(){
    
    cout << "+++++++++++++++++++++++++++++++ \n";
    cout << "Particle Summary \n";
    cout << "ID: " << this->ID << "\n";
    cout << "Pose: x: " << this->pose(0) << " | y: " << this->pose(1) << " | theta: " << this->pose(2) <<"\n";
    cout << "Weight: " << this->weight << "\n";
    cout << "Map Dimension: x: " << this->map.getWidth() << "m (" << this->map.getData().cols << "px) | y: " << this->map.getHeight() << "m (" << this->map.getData().rows << "px) \n";
    cout << "+++++++++++++++++++++++++++++++ \n";
    
}
