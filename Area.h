//
//  Area.h
//  FastSLAM
//
//  Created by Mats Steinweg on 04.09.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Area_h
#define Area_h

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Robot.h"

class Area{
    
    public:
    // constructor and destructor
    Area();
    Area(vector<vector<float>> wall_coordinates, float x_min, float x_max, float y_min, float y_max, float resolution);
    ~Area(){};
    
    // drawing functions
    void drawInitialScene();
    cv::Mat drawScene(int verbose);
    void drawPath();
    void drawRobot();
    void drawWalls();
    void drawScan();
    void drawScanEstimate();
    void drawParticles();
    
    // transforms from continuous world coordinate system to discrete area coordinate system
    Eigen::Vector2i discretize_world_location(const Eigen::Vector2f& world_location);
    int scale_world_variable(const float& world_coordinate);
    
    // getter functions
    Robot& getRobot(){ return this->robot; };
    
    // setter functions
    void setRobot(Robot robot){ this->robot = robot; };
    
    // print summary
    void summary();

    private:
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float resolution;
    int data_dim_x;
    int data_dim_y;
    
    cv::Mat data;
    Robot robot;
    
    vector<vector<float>> wall_coordinates;
    
    // colors
    cv::Scalar robot_color = {130, 130, 130};
    cv::Scalar particle_color = {60, 60, 60};
    cv::Scalar scan_color = {80, 127, 255};
    cv::Scalar scan_estimate_color = {255, 127, 80};
    cv::Scalar path_ground_truth_color = {80, 127, 255};
    cv::Scalar path_estimate_color = {255, 127, 80};

};

#endif /* Area_h */
