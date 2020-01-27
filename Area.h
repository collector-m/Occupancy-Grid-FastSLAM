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
    // Constructor and destructor
    Area();
    Area(vector<vector<float>> wall_coordinates, float x_min, float x_max, float y_min, float y_max, float resolution);
    ~Area(){};
    
    // Drawing functions
    void drawScene(int verbose);
    void drawPath();
    void drawRobot();
    void drawWalls();
    void drawScan();
    void drawScanEstimate();
    void drawParticles();
    
    // Transforms from continuous world coordinate system to discrete area coordinate system
    Eigen::Vector2i discretize_world_location(const Eigen::Vector2f& world_location);
    int scale_world_variable(const float& world_coordinate);
    
    // Getter functions
    Robot& getRobot(){ return this->robot; };
    cv::Mat& getData(){ return this->scene_data; };
    
    // Setter functions
    void setRobot(Robot& robot){ this->robot = robot; };
    
    // Print summary
    void summary();

    private:
        float x_min; // minimum x-coordinate of area
        float x_max; // maximum x-coordinate of area
        float y_min; // minimum y-coordinate of area
        float y_max; // maximum y-coordinate of area
        float resolution; // resolution of scene image in m/px
        int data_dim_x; // width of the scene image in px
        int data_dim_y; // height of the scene image in px
        
        cv::Mat data; // image on which scene is drawn
        cv::Mat scene_data; // image which contains currently active scene (for result saving)
        Robot robot; // robot object
        
        // Vector containing all wall coordiantes in area
        vector<vector<float>> wall_coordinates;
        
        // Colors
        cv::Scalar robot_color = {130, 130, 130};
        cv::Scalar particle_color = {60, 60, 60};
        cv::Scalar scan_color = {80, 127, 255};
        cv::Scalar scan_estimate_color = {255, 127, 80};
        cv::Scalar path_ground_truth_color = {80, 127, 255};
        cv::Scalar path_estimate_color = {255, 127, 80};

};

#endif /* Area_h */
