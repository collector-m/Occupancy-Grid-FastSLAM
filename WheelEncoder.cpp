//
//  WheelEncoder.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 09.09.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include <random>

#include "WheelEncoder.h"

using namespace std;

#define PI 3.14159265

default_random_engine generator;
normal_distribution<float> distribution(0.0, 1.0);


// constructor
WheelEncoder::WheelEncoder(){
    
    this->E_T = 2048; // number of ticks per wheel rotation
    this->B = 0.35; // distance between wheels in m
    this->R_L = 0.1; // radius of the left wheel
    this->R_R = 0.1; // radius of the right wheel
    this->noise = Eigen::Vector2f::Ones() * 0.01; // standard deviation of the measurement noise
    this->last_timestamp = 0.0;
    this->ticks_left = 0;
    this->ticks_right = 0;
    this->ticks_left_prev = 0;
    this->ticks_right_prev = 0;
    
}


// inverse odometry model
void WheelEncoder::encode_movement(float v, float omega, const float& delta_t){
        
    // apply uncorrelated noise to v and omega to model sensor inaccuracies due to friction and wind
    v += (this->noise(0) * distribution(generator));
    omega += (this->noise(1) * distribution(generator));
    
    float omega_l = (2*v - omega*this->B) / (2*this->R_L);
    float omega_r = (2*v - omega_l*this->R_L) / this->R_R;
    
    this->ticks_right += (int)((omega_r * this->E_T * delta_t) / (2*PI));
    this->ticks_left += (int)((omega_l * this->E_T * delta_t) / (2*PI));
    
}

// get odometry information from wheel encoder ticks
Eigen::Vector2f WheelEncoder::getOdometry(float current_timestamp){
    
    float delta_t = current_timestamp - this->last_timestamp;
    
    Eigen::Vector2f control_signal;
    
    int delta_ticks_left = this->ticks_left - this->ticks_left_prev;
    int delta_ticks_right = this->ticks_right - this->ticks_right_prev;
    
    float omega_r = 2 * PI * delta_ticks_left / (this->E_T * delta_t);
    float omega_l = 2 * PI * delta_ticks_right / (this->E_T * delta_t);
    float omega = (omega_r * this->R_R - omega_l * this->R_L) / this->B;
    float v = (omega_r * R_R + omega_l * this->R_L) / 2;
    
    control_signal(0) = v;
    control_signal(1) = omega;
    
    this->last_timestamp = current_timestamp;
    this->ticks_left_prev = this->ticks_left;
    this->ticks_right_prev = this->ticks_right;
    
    return control_signal;
    
}
