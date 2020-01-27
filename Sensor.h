//
//  Sensor.hpp
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Sensor_hpp
#define Sensor_hpp

#include <stdio.h>
#include <Eigen/Dense>

using namespace std;

class Sensor {
    
    public:
        // Constructor and destructor
        Sensor();
        Sensor(int FoV, int range, float resolution, Eigen::Vector2f Q);
        ~Sensor(){};
    
        // Print sensor summary
        void summary();

        // Getter functions
        Eigen::MatrixX2f& getMeasurements(){ return this->measurements; };
        const int& getRange(){ return this->range; };
        const int& getN(){ return this->n_measurements; };
        const int& getFoV(){ return this->FoV; };
        const Eigen::Vector2f& getQ(){ return this->Q; };

        // Compute sensor sweep
        void sweep(const vector<vector<float>>& map_coordinates, const Eigen::Vector3f& pose);
    
    private:
        int FoV; // sensor's field of view in degree
        int range; // sensor's maximum range in m
        float resolution; // sensor's resolution in beams/degree
        int n_measurements; // number of measurements obtained per sweep
        Eigen::MatrixX2f measurements; // array of measurement values
        Eigen::Vector2f Q; // standard deviation of gaussian measurement noise
    
};

#endif /* Sensor_hpp */
