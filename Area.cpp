
//
//  Area.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 04.09.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include "Area.h"

class Map;

#define PI 3.14159265


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Standard constructor
Area::Area(){
    
    this->x_min = -5.0;
    this->x_max = 25.0;
    this->y_min = -5.0;
    this->y_max = 25.0;
    this->resolution = 0.1;
    this->data_dim_x = (int) ((x_max - x_min) / resolution);
    this->data_dim_y = (int) ((y_max - y_min) / resolution);
    this->data = cv::Mat(this->data_dim_x, this->data_dim_y, CV_8UC3, cv::Scalar::all(255));
    this->data.copyTo(scene_data);

}


// Constructor
Area::Area(vector<vector<float>> wall_coordinates, float x_min, float x_max, float y_min, float y_max, float resolution){
    
    // Set wall coordinates and boundaries of the area
    this->wall_coordinates = wall_coordinates;
    this->x_min = x_min;
    this->x_max = x_max;
    this->y_min = y_min;
    this->y_max = y_max;
    
    // Verify that all walls are located inside the specified area
    for (vector<vector<float>>::iterator it = wall_coordinates.begin(); it != wall_coordinates.end(); it++){
        const float x_min_wall = min((*it)[0], (*it)[2]);
        const float x_max_wall = max((*it)[0], (*it)[2]);
        const float y_min_wall = min((*it)[1], (*it)[3]);
        const float y_max_wall = max((*it)[1], (*it)[3]);
        if(x_min_wall < x_min || x_max_wall > x_max || y_min_wall < y_min || y_max_wall > y_max) {
            cout << " At least one wall is located outside of the specified area!" << endl;
            exit(1);
        }
    }
    
    // Specify resolution and create image for drawing area
    this->resolution = resolution;
    this->data_dim_x = (int) ((x_max - x_min) / resolution);
    this->data_dim_y = (int) ((y_max - y_min) / resolution);
    this->data = cv::Mat(this->data_dim_x, this->data_dim_y, CV_8UC3, cv::Scalar::all(255));
    this->data.copyTo(scene_data);
    
    // Draw layout of the area
    this->drawWalls();
    
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++ Print Summary +++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Area::summary(){
    
    cout << "Area: " << endl;
    cout << "-----" << endl;
    cout << "Width: " << this->x_max - this->x_min << "m" << endl;
    cout << "Height: " << this->y_max - this->y_min << "m" << endl;
    // Print map summary
    Map::summary();
    // Print robot summary
    this->getRobot().summary();

}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++ Drawing Functions ++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Draw entire scene content according to specified verbosity level
void Area::drawScene(int verbose){
        
        // ++++++++++++++++++++++++++++++++++++ Draw Static Content ++++++++++++++++++++++++++++++++++++++++++

        // Draw robot path. Path is drawn before copying data to display all past locations of robot.
        this->drawPath();
            
    
        // +++++++++++++++++++++++++++++++++++++++ Create Copy +++++++++++++++++++++++++++++++++++++++++++++++

        // Create a copy to restore static scene at every time step
        cv::Mat data_copy;
        this->data.copyTo(data_copy);
    
    
        // +++++++++++++++++++++++++++++++++++ Draw Dynamic Content ++++++++++++++++++++++++++++++++++++++++++

        // Draw sensor readings
        this->drawScan();
        
        // Draw scan estimate if specified
        if (verbose == 2){
            this->drawScanEstimate();
        }
            
        // Draw robot on top of scans
        this->drawRobot();
        
        // Draw particles if specified
        if (verbose >= 1){
            this->drawParticles();
        }
    
        // Assign complete scene data for current time step to variable for result saving
        this->data.copyTo(this->scene_data);

        // Display current scene
        cv::imshow("Area", this->data);
        cv::waitKey(1);
    
        // Restore static scene data
        this->data = data_copy;
    
}


// Draw walls
void Area::drawWalls(){
    
    // Iterate over all walls in area
    for (vector<vector<float>>::iterator i = this->wall_coordinates.begin(); i != this->wall_coordinates.end(); i++) {
        
        // Extract start and end point of the line segment and transform them to discrete area coordinates
        Eigen::Vector2f startpoint;
        Eigen::Vector2f endpoint;
        startpoint << (*i)[0], (*i)[1];
        endpoint << (*i)[2], (*i)[3];
        Eigen:: Vector2i area_startpoint = this->discretize_world_location(startpoint);
        Eigen:: Vector2i area_endpoint = this->discretize_world_location(endpoint);
        
        // Draw wall as black line
        line(this->data, {area_startpoint(0), area_startpoint(1)}, {area_endpoint(0), area_endpoint(1)}, {0, 0, 0}, 2);
        
    }
}


// Draw robot path
void Area::drawPath(){
    
    // Get robot pose in continuous world coordinates
    Eigen::Vector2f robot_location = this->robot.getPose().block<2, 1>(0, 0);
    
    // Get robot pose in discrete area coordinates
    Eigen::Vector2i area_location = this->discretize_world_location(robot_location);
    
    // Draw path as dots
    cv::circle(this->data, {area_location(0), area_location(1)}, 4, this->path_ground_truth_color, -1);
    
}


// Draw laser scan
void Area::drawScan(){
    
    // Get robot location in world coordinates
    const Eigen::Vector2f robot_location = this->robot.getPose().block<2,1>(0,0);
    
    // Get robot heading
    const float robot_heading = this->robot.getPose()(2);
    
    // Get robot location in discrete area coordinates
    const Eigen::Vector2i area_robot_location = this->discretize_world_location(robot_location);
    
    // Get reference to measurements
    const Eigen::MatrixX2f measurement_ref = this->robot.getSensor().getMeasurements();
    
    // Get number of measurements
    const int n_beams = this->robot.getSensor().getN();
    
    // Iterate over all measurements
    for (int beam_id = 0; beam_id < n_beams; beam_id++) {
        
        // Get range and angle information for currently inspected measurement
        float phi = measurement_ref(beam_id, 0);
        float area_range = this->scale_world_variable(measurement_ref(beam_id, 1));
        
        // Get final angle as sum of robot's heading and measurement angle relative to robot
        float alpha = phi + robot_heading;
        alpha = fmod((alpha+PI), (2.0*PI)) - PI; // Keep angle within range of [-pi, pi)
        
        // get Endpoint of the obtained measurement
        int line_endpoint_x = (int)(area_robot_location(0) + area_range * cos(alpha));
        int line_endpoint_y = (int)(area_robot_location(1) + area_range * sin(alpha));
        
        // Draw laser beam as line
        cv::line(this->data, {area_robot_location(0), area_robot_location(1)}, {line_endpoint_x, line_endpoint_y}, this->scan_color, 1);
    }
}


// Draw laser scan estimate for each particle
void Area::drawScanEstimate(){
    
    for (list<Particle>::iterator it = this->robot.getFilter().getParticles().begin(); it != this->robot.getFilter().getParticles().end(); it++) {
        
        // Get particle location in continuous world coordinates
        const Eigen::Vector2f particle_location = (*it).getPose().block<2,1>(0,0);
        
        // Get particle heading
        const float particle_heading = (*it).getPose()(2);
        
        // Get particle location in discrete area coordinates
        const Eigen::Vector2i area_particle_location = this->discretize_world_location(particle_location);
        
        // Get number of measurements
        const int n_beams = this->robot.getSensor().getN();
        
        // Get reference to estimated measurements
        const Eigen::MatrixX2f measurement_estimate_ref = (*it).getMeasurementEstimate();
        
        // Iterate over all estimated measurements
        for (int beam_id = 0; beam_id < n_beams; beam_id++) {
            
            // Get range and angle information for currently inspected measurement estimate
            float phi = measurement_estimate_ref(beam_id, 0);
            float area_range = this->scale_world_variable(measurement_estimate_ref(beam_id, 1));
            
            // Get final angle as sum of robot's heading and measurement angle relative to robot
            float alpha = phi + particle_heading;
            alpha = fmod((alpha+PI), (2.0*PI)) - PI; // Keep angle within range of [-pi, pi)
            
            // Get endpoint of the obtained measurement
            int line_endpoint_x = (int)(area_particle_location(0) + area_range * cos(alpha));
            int line_endpoint_y = (int)(area_particle_location(1) + area_range * sin(alpha));
            
            // Draw laser beam estimate as line
            cv::line(this->data, {area_particle_location(0), area_particle_location(1)}, {line_endpoint_x, line_endpoint_y}, this->scan_estimate_color, 1);
        }
    }
}


// Draw robot
void Area::drawRobot(){
    
    // Get robot location in world coordinates
    const Eigen::Vector2f robot_location = this->robot.getPose().block<2,1>(0,0);
    
    // Get robot heading
    const float robot_heading = this->robot.getPose()(2);
    
    // Get robot location in discrete area coordinates
    const Eigen::Vector2i area_robot_location = this->discretize_world_location(robot_location);
    
    // Get robot radius in image coordinates
    int area_radius = this->scale_world_variable(this->robot.getRadius());
    
    // Draw robot as a circle
    cv::circle(this->data, {area_robot_location(0), area_robot_location(1)}, area_radius, this->robot_color, -1);
    
    // Indicate orientation of the robot by drawing a line from the origin
    float line_scale = 1.0; // * robot radius
    int line_endpoint_x = (int)(area_robot_location(0) + line_scale * area_radius * cos(robot_heading));
    int line_endpoint_y = (int)(area_robot_location(1) + line_scale * area_radius * sin(robot_heading));
    cv::line(this->data, {area_robot_location(0), area_robot_location(1)}, {line_endpoint_x, line_endpoint_y}, {255, 255, 255}, 2);
    
    // Draw black border around robot
    cv::circle(this->data, {area_robot_location(0), area_robot_location(1)}, area_radius, {0, 0, 0}, 2);

}


// Draw Particles
void Area::drawParticles(){
    
    // Iterate over all particles
    for (list<Particle>::iterator it = this->robot.getFilter().getParticles().begin(); it != this->robot.getFilter().getParticles().end(); it++) {
        
        // Get particle location in continuous world coordinates
        const Eigen::Vector2f particle_location = (*it).getPose().block<2,1>(0,0);
        
        // Get particle location in discrete area coordinates
        const Eigen::Vector2i area_particle_location = this->discretize_world_location(particle_location);
        
        // Draw particle as dot
        cv::circle(this->data, {area_particle_location(0), area_particle_location(1)}, 6, this->particle_color, -1);
        
    }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++ Coordinate Transforms +++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Transform continuous world coordinates into discrete area coordinates (m -> px)
Eigen::Vector2i Area::discretize_world_location(const Eigen::Vector2f& world_location){
    
    // Container for pose in discrete area coordinates
    Eigen::Vector2i area_location;
    
    // Discretize x and y coordinates
    area_location(0) = (int)((world_location(0) - x_min) / (x_max - x_min) * data_dim_x);
    area_location(1) = (int)((world_location(1) - y_min) / (y_max - y_min) * data_dim_y);
    
    return area_location;
    
}


// Scale scalar variables from world coordinate system to area coordinate system (m -> px)
int Area::scale_world_variable(const float& world_variable){
    
    // Container for scaled area variable
    int area_variable;
    
    // Scale variable
    area_variable = (int)(world_variable / this->resolution);
    
    return area_variable;
    
}
