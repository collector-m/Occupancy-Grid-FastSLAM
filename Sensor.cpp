//
//  Sensor.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <iostream>
#include <vector>
#include <math.h>

#include "Sensor.h"

#define PI 3.14159265

using namespace std;


// standard constructor
Sensor::Sensor(){
    
    // set member variables
    this->FoV = 90;
    this->range = 10;
    this->resolution = 1;
    this->n_measurements = (int)((FoV/resolution) + 1);
    this->Q(0) = 0.1;
    this->Q(1) = 0.1;
    this->measurements = Eigen::MatrixX2f::Zero(this->n_measurements, 2);
    
}

// constructor
Sensor::Sensor(int FoV, int range, float resolution){
    
    // set member variables
    this->FoV = FoV;
    this->range = range;
    this->resolution = resolution;
    this->n_measurements = (int)((FoV/resolution) + 1);
    this->Q(0) = 0.1;
    this->Q(1) = 0.1;
    this->measurements = Eigen::MatrixX2f::Zero(this->n_measurements, 2);
    
}


// summary of sensor
void Sensor::summary(){
    
    cout << "Sensor: \n";
    cout << "-------" << endl;
    cout << "FoV: " << this->FoV << "° \n";
    cout << "Range: " << this->range << "m \n";
    cout << "Resolution: " << this->resolution << "° \n";
    cout << "Number of Measurements: " << this->n_measurements << endl;
    cout << "Sensor Uncertainty: " << this->Q(0) << "m, " << this->Q(1) << "rad" << endl;
}


// compute complete sensor sweep
void Sensor::sweep(const vector<vector<float>> &map_coordinates, const Eigen::Array3f &pose){
    
    // sensor resolution in radians
    float resol_rad = this->resolution * PI / 180;

    // vector containing wall coordinates relative to robot's position
    vector<float> trans_x_start;
    vector<float> trans_y_start;
    vector<float> trans_x_end;
    vector<float> trans_y_end;
    
    // line equation parameters
    vector<float> a;
    vector<float> b;
    vector<float> c;
    
    // current robot pose
    float x = pose(0);
    float y = pose(1);
    float theta = pose(2);

    // iterate over each wall in the map
    for (vector<vector<float>>::const_iterator i = map_coordinates.begin(); i != map_coordinates.end(); i++) {
        
        // start and end coordinates of the line segment
        float x_start = (*i)[0];
        float y_start = (*i)[1];
        float x_end = (*i)[2];
        float y_end = (*i)[3];
        
        // line coordintes relative to robot pose
        trans_x_start.push_back((x_start-x) * cos(theta) + (y_start-y)*sin(theta));
        trans_y_start.push_back((y_start-y)*cos(theta) - (x_start-x)*sin(theta));
        trans_x_end.push_back((x_end-x)*cos(theta) + (y_end-y)*sin(theta));
        trans_y_end.push_back((y_end-y)*cos(theta) - (x_end-x)*sin(theta));
        
        // check if start and end point are in correct order
        if (trans_x_start.back() > trans_x_end.back()) {
            float trans_x_temp;
            float trans_y_temp;

            trans_x_temp = (trans_x_start.back());
            trans_x_start.back() = (trans_x_end.back());
            trans_x_end.back() = trans_x_temp;
            trans_y_temp = (trans_y_start.back());
            (trans_y_start.back()) = (trans_y_end.back());
            (trans_y_end.back()) = trans_y_temp;
            
        }
        
        // line equation parameters for currently inspected wall
        a.push_back((trans_y_start.back()) - (trans_y_end.back()));
        b.push_back((trans_x_end.back()) - (trans_x_start.back()));
        float l = sqrt(pow((a.back()), 2) + pow((b.back()), 2));
        (a.back()) = (a.back()) / l;
        (b.back()) = (b.back()) / l;
        c.push_back((a.back()) * (trans_x_start.back()) + (b.back()) * (trans_y_start.back()));
        
    }
    
    // iterate over all laser beams
    for (int beam_id = 0; beam_id < this->n_measurements; beam_id++) {
        
        // set the initial measurement to maximum sensor range
        float min_dist = (float) this->range;
        
        // get angle of currently inspected beam relative to robot pose
        float phi = beam_id * resol_rad - (this->FoV / 2.0) * PI / 180.0;
        
        // get number of walls
        int n_walls = (int) map_coordinates.size();
        
        // iterate over each wall in the map
        for (int wall_id = 0; wall_id < n_walls; wall_id++) {
            
            // check if laser beam and wall intersect
            float norm_coeff = a[wall_id] * cos(phi) + b[wall_id] * sin(phi);
            
            // if intersection
            if (abs(norm_coeff) > 1e-6) {
                
                // distance from origin of coordinate system to point on line
                float d = c[wall_id] / norm_coeff;
                
                // ensure that point lies on specified line segment
                if (d > 0 && d < min_dist) {
                    if (abs(trans_x_end[wall_id] - trans_x_start[wall_id]) > 1e-2) {
                        if(d * cos(phi) < trans_x_end[wall_id] && d * cos(phi) > trans_x_start[wall_id]) {
                            min_dist = d;
                        }
                    }
                    else {
                        if (abs(trans_y_end[wall_id] - trans_y_start[wall_id]) > 1e-2) {
                            if(d * sin(phi) < trans_y_end[wall_id] && d * sin(phi) > trans_y_start[wall_id]) {
                                    min_dist = d;
                            }
                        }
                        else {
                            if(d * sin(phi) > trans_y_end[wall_id] && d * sin(phi) < trans_y_start[wall_id]) {
                                        min_dist = d;
                            }
                        }
                    }
                }
            }
        }
    
    // fill array with angle of laser beam and measured distance to wall
    this->measurements(beam_id, 0) = phi;
    this->measurements(beam_id, 1) = min_dist;
    
    }
}
