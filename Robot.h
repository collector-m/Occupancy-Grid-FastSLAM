//
//  Robot.h
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Robot_h
#define Robot_h

#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "Sensor.h"
#include "RBPF.h"
#include "WheelEncoder.h"

class Robot {
    
    public:
        // Constructors and destructor
        Robot();
        Robot(Eigen::Vector3f initial_pose);
        ~Robot(){};
    
        // Print summary of robot
        void summary();
    
        // Drive robot
        Eigen::Vector2f drive(const float& delta_t);
    
        // Getter functions
        Eigen::Vector3f& getPose(){ return this->pose; };
        Sensor& getSensor(){ return this->sensor; };
        RBPF& getFilter(){ return this->filter; };
        WheelEncoder& getWheelEncoder(){ return this->wheel_encoder; };
        const float& getRadius(){ return this->radius; };
        const float& getV(){ return this->v; };
        const float& getOmega(){ return this->omega; };
        const float& getTimestamp(){ return this->last_timestamp; };

        // Setter functions
        void setV(const float& v){ this->v = v; };
        void setOmega(const float& omega){ this->omega = omega; };
        void setSensor(Sensor& sensor){ this->sensor = sensor; };
        void setFilter(RBPF& filter){ this->filter = filter; };

    private:
        Eigen::Vector3f pose; // robot pose (x, y, theta)
        Sensor sensor; // robot's sensor
        RBPF filter; // robot's particle filter
        WheelEncoder wheel_encoder; // wheel encoder
        float last_timestamp; // timestamp of last pose update
        float radius; // radius of circular robot in m
        float v;  // translational velocity of the robot
        float omega; // angular velocity of the robot
};

#endif /* Robot_h */
