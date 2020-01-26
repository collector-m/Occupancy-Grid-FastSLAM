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

// initialize static member variables
const float Map::width = 30;
const float Map::height = 30;
const float Map::resolution = 0.1;
const float Map::x_min = -5; // min x coordinate of map in m
const float Map::x_max = 25; // max x coordinate of map in m
const float Map::y_min = -5; // min y coordinate of map in m
const float Map::y_max = 25; // max y coordinate of map in m

const int Map::occupancy_value_min = 0;
const int Map::occupancy_value_max = 255;
const int Map::occupancy_value_step = 25;
const int Map::occupancy_value_init = 127;


// constructor
Map::Map(){
    
    // set member variables
    //this->data = cv::imread("Data/map.jpg", CV_8UC1);
    this->data = cv::Mat((int)width/resolution, (int)height/resolution, CV_8UC1, cv::Scalar::all(127));
    
}


// summary function
void Map::summary(){
    
    cout << "Map: \n";
    cout << "----" << endl;
    cout << "Resolution: " << Map::resolution << "m/px \n";
    cout << "value_min: " << Map::occupancy_value_min << endl;
    cout << "value_max: " << Map::occupancy_value_max << endl;
    cout << "value_step: " << Map::occupancy_value_step << endl;
    
}


// display map using cv::imshow
cv::Mat Map::draw(){
    
    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Map", this->data);
    cv::waitKey(1);
    
    return this->data;
    
}


// coordinate transformation
Eigen::Array3f Map::world2map(const Eigen::Array3f& world_pose){
    
    Eigen::Array3f image_pose; // pose in image coordinates
    
    image_pose(0) = (int)((world_pose(0) - Map::x_min) / (Map::x_max - Map::x_min) * Map::width / Map::resolution);
    image_pose(1) = (int)((world_pose(1) - Map::y_min) / (Map::y_max - Map::y_min) * Map::height / Map::resolution);
    image_pose(2) = (float) world_pose(2);
    
    return image_pose;
    
}


// scale transform
int Map::world2map(const float &world_variable){
    
    int map_variable = (int) (world_variable / Map::resolution);
    return map_variable;
}


// scale transform
float Map::map2world(const int &map_variable){
    
    float world_variable = (float) (map_variable * Map::resolution);
    return world_variable;
}
