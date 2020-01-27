//
//  Map.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 16.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "eigen2cv.h"
#include "Map.h"

using namespace std;


// Initialize static member variables
// Default values will be overwritten by specified simulation parameters
int Map::map_type = 0;
string Map::gt_map_file_path = "Data/gt_map.jpg";

float Map::width = 30;
float Map::height = 30;
float Map::resolution = 0.1;
float Map::x_min = -5; // min x coordinate of map in m
float Map::x_max = 25; // max x coordinate of map in m
float Map::y_min = -5; // min y coordinate of map in m
float Map::y_max = 25; // max y coordinate of map in m

int Map::occupancy_value_min = 0;
int Map::occupancy_value_max = 255;
int Map::occupancy_value_step = 25;
int Map::occupancy_threshold = 127;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Map::Map(){
    
    // Initialize raw map if map_type is 0 (mapping or SLAM mode)
    if (Map::map_type == 0){
        this->data = cv::Mat((int)width/resolution, (int)height/resolution, CV_8UC1, cv::Scalar::all(Map::occupancy_threshold));
    }
    // Use ground truth map if map_type is 1 (localization mode)
    else {
        
        bool gt_map_exists = std::__fs::filesystem::exists(Map::gt_map_file_path);
        if (gt_map_exists == false){
            cout << "No ground truth map available!" << endl;
            exit(1);
        }
        else {
            // Read-in ground truth map
            this->data = cv::imread(gt_map_file_path, CV_8UC1);
            
            // Get width and height of ground truth map
            int gt_height = this->data.size().height;
            int gt_width = this->data.size().width;
            
            // Get specified width and height from simulation parameters
            int map_height = (int)height/resolution;
            int map_width = (int)width/resolution;
            
            // Change width and height to dimensions of ground truth in case of mismatch
            if (gt_width != map_width || gt_height != map_height){
                Map::resolution = (Map::x_max - Map::x_min) / gt_width;
                cout << "Size of ground truth map doesn't match specified map dimension." << endl;
                cout << "New map dimensions: " << endl;
                cout << "Width: " << gt_width << "px | Height: " << gt_height << "px" << endl;
                cout << "Resolution: " << Map::resolution << "m/px" << endl;
            }
        }
    }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++ Print Summary ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Map::summary(){
    
    cout << "Map:" << endl;
    cout << "----" << endl;
    cout << "Resolution: " << Map::resolution << "m/px" << endl;
    cout << "Width: " << (int)width/resolution << "px | Height: " << (int)height/resolution << "px" << endl;
    cout << "value_min: " << Map::occupancy_value_min << endl;
    cout << "value_max: " << Map::occupancy_value_max << endl;
    cout << "value_step: " << Map::occupancy_value_step << endl;
    
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++ Set Parameters ++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Set static map parameters from simulation parameters
void Map::setParameters(float x_min, float x_max, float y_min, float y_max, float map_resolution, int occupancy_value_min, int occupancy_value_max, int occpuancy_value_step, int occupancy_threshold, int simulation_mode, string data_dir){
    
    Map::resolution = map_resolution;
    Map::x_min = x_min;
    Map::x_max = x_max;
    Map::y_min = y_min;
    Map::y_max = y_max;
    Map::width = x_max - x_min;
    Map::height = y_max - y_min;
    Map::occupancy_value_min = occupancy_value_min;
    Map::occupancy_value_max = occupancy_value_max;
    Map::occupancy_value_step = occpuancy_value_step;
    Map::occupancy_threshold = occupancy_threshold;
    
    // Set required map type
    if (simulation_mode == 1 || simulation_mode == 2){
        Map::map_type = 0; // use empty map for mapping and SLAM mode
    }
    else {
        Map::map_type = 1; // use ground truth map for localization mode
        Map::gt_map_file_path = data_dir + "/" + "gt_map.jpg";
    }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++ Draw Map ++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Map::draw(){
    
    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Map", this->data);
    cv::waitKey(1);
        
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++ Coordinate Transforms +++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Transform continuous world coordinates into discrete map coordinates (m -> px)
Eigen::Array3f Map::world2map(const Eigen::Array3f& world_pose){
    
    // Container for pose in image coordinates
    Eigen::Array3f image_pose;
    
    // Transform coordinates
    image_pose(0) = (int)((world_pose(0) - Map::x_min) / (Map::x_max - Map::x_min) * Map::width / Map::resolution);
    image_pose(1) = (int)((world_pose(1) - Map::y_min) / (Map::y_max - Map::y_min) * Map::height / Map::resolution);
    image_pose(2) = (float) world_pose(2);
    
    return image_pose;
    
}


// Scale scalar variable to map coordinate system
int Map::world2map(const float &world_variable){
    
    int map_variable = (int) (world_variable / Map::resolution);
    return map_variable;
}


// Scale scalar variable from map to world coordinate system
float Map::map2world(const int &map_variable){
    
    float world_variable = (float) (map_variable * Map::resolution);
    return world_variable;
}
