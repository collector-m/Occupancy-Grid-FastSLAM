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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++ Encode Motion ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Inverse odometry model
void WheelEncoder::encode_motion(float v, float omega, const float& delta_t){
        
    // Apply uncorrelated noise to v and omega to model sensor inaccuracies due to friction and wind
    v += (this->noise(0) * distribution(generator));
    omega += (this->noise(1) * distribution(generator));
    
    // Compute angular velocity of left and right wheel
    float omega_l = (2*v - omega*this->B) / (2*this->R_L);
    float omega_r = (2*v - omega_l*this->R_L) / this->R_R;
    
    // Accumulate encoder ticks
    this->ticks_right += (int)((omega_r * this->E_T * delta_t) / (2*PI));
    this->ticks_left += (int)((omega_l * this->E_T * delta_t) / (2*PI));
    
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++ Get Odometry +++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Compute odometry information from wheel encoder ticks
Eigen::Vector2f WheelEncoder::getOdometry(float current_timestamp){
    
    // Get Sampling time
    float delta_t = current_timestamp - this->last_timestamp;
    
    // Instantiate control signal container
    Eigen::Vector2f control_signal;
    
    // Get ticks for current timestep
    int delta_ticks_left = this->ticks_left - this->ticks_left_prev;
    int delta_ticks_right = this->ticks_right - this->ticks_right_prev;
    
    // Compute v and omega from encoder ticks
    float omega_r = 2 * PI * delta_ticks_left / (this->E_T * delta_t);
    float omega_l = 2 * PI * delta_ticks_right / (this->E_T * delta_t);
    float omega = (omega_r * this->R_R - omega_l * this->R_L) / this->B;
    float v = (omega_r * R_R + omega_l * this->R_L) / 2;
    
    // Set computed control signals
    control_signal(0) = v;
    control_signal(1) = omega;
    
    // Update timestamp and encoder ticks
    this->last_timestamp = current_timestamp;
    this->ticks_left_prev = this->ticks_left;
    this->ticks_right_prev = this->ticks_right;
    
    return control_signal;
    
}
