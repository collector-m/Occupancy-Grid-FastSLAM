//
//  Robot.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include <math.h>

#include "Robot.h"

using namespace std;
using namespace cv;

#define PI 3.14159265

///////////////////
// constructors //
//////////////////

// standard constructor
Robot::Robot() {
    
    // set member variables
    this->pose = Eigen::Vector3f::Zero();
    this->last_timestamp = 0.0;
    this->radius = 0.5;
}


// constructor with initial position
Robot::Robot(Eigen::Vector3f initial_pose) {
    
    // set member variables
    this->pose = initial_pose;
    this->last_timestamp = 0.0;
    this->radius = 0.5;
}


//////////////
// summary //
/////////////

void Robot::summary(){
    
    cout << "+++++++++++++++++++++++++++++++ \n";
    cout << "Robot Summary \n";
    cout << "Radius: " << this->radius << "m \n";
    cout << "Pose: x: " << this->pose(0) << " | y: " << this->pose(1) << " | theta: " << this->pose(2) <<"\n";
    cout << "Sensor: \n";
    this->sensor.summary();
    cout << "RBPF: \n";
    this->filter.summary();
    cout << "Last Update: " << this->last_timestamp << "s" << endl;
    cout << "+++++++++++++++++++++++++++++++ \n";
    
}


////////////
// drive //
///////////

void Robot::drive(const float &delta_t){
        
    // pose difference due to constant control signal applied over period of delta_t
    Eigen::Vector3f pose_dif;
    pose_dif(0) = delta_t * this->v * cos((float)this->pose(2));
    pose_dif(1) = delta_t * this->v * sin((float)this->pose(2));
    pose_dif(2) = delta_t * this->omega;
    pose_dif(2) = fmod((float)pose_dif(2)+PI, 2*PI) - PI;

    // update pose
    this->pose += pose_dif;
        
    // encode movement
    this->getWheelEncoder().encode_movement(this->v, this->omega, delta_t);
        
    // set last update to current timestamp
    this->last_timestamp += delta_t;

}

