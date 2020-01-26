
//
//  Area.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 04.09.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include "Area.h"

#define PI 3.14159265

class Map;

// standard constructor
Area::Area(){
    
    // set member variables with default values
    this->x_min = -5.0;
    this->x_max = 25.0;
    this->y_min = -5.0;
    this->y_max = 25.0;
    this->resolution = 0.1;
    this->data_dim_x = (int) ((x_max - x_min) / resolution);
    this->data_dim_y = (int) ((y_max - y_min) / resolution);
    this->data = cv::Mat(this->data_dim_x, this->data_dim_y, CV_8UC3, cv::Scalar::all(255));

}


// constructor
Area::Area(vector<vector<float>> wall_coordinates, float x_min, float x_max, float y_min, float y_max, float resolution){
    
    // set wall coordinates and boundaries of the area
    this->wall_coordinates = wall_coordinates;
    this->x_min = x_min;
    this->x_max = x_max;
    this->y_min = y_min;
    this->y_max = y_max;
    
    // verify that all walls are located inside the specified area
    for (vector<vector<float>>::iterator it = wall_coordinates.begin(); it != wall_coordinates.end(); it++){
        const float x_min_wall = min((*it)[0], (*it)[2]);
        const float x_max_wall = max((*it)[0], (*it)[2]);
        const float y_min_wall = min((*it)[1], (*it)[3]);
        const float y_max_wall = max((*it)[1], (*it)[3]);
        if(x_min_wall < x_min || x_max_wall > x_max || y_min_wall < y_min || y_max_wall > y_max) {
            cout << " At least one wall is located outside the specified area!" << endl;
            exit(1);
        }
    }
    
    // specify resolution and create image for drawing the area
    this->resolution = resolution;
    this->data_dim_x = (int) ((x_max - x_min) / resolution);
    this->data_dim_y = (int) ((y_max - y_min) / resolution);
    this->data = cv::Mat(this->data_dim_x, this->data_dim_y, CV_8UC3, cv::Scalar::all(255));
    
    // draw layout of the area
    this->drawWalls();
    
}


// *****************
// drawing functions
// *****************

void Area::drawInitialScene(){
    
        // create a copy to restore original image
        cv::Mat data_copy;
        this->data.copyTo(data_copy);
        
        // draw robot
        this->drawRobot();
        
        cv::imshow("Area", this->data);
        cv::waitKey();
        
        // restore original image
        this->data = data_copy;
    
}


cv::Mat Area::drawScene(int verbose){
        
        // draw robot path
        this->drawPath();
            
        // create a copy to restore original image
        cv::Mat data_copy;
        this->data.copyTo(data_copy);
            
        // draw sensor readings
        this->drawScan();
        
        // draw scan estimate
        if (verbose == 2){
        this->drawScanEstimate();
        }
            
        // draw robot
        this->drawRobot();
        
        // draw particles
        if (verbose >= 1){
        this->drawParticles();
        }
    
        // create copy of scene for saving
        cv::Mat data_save;
        this->data.copyTo(data_save);

        cv::imshow("Area", this->data);
        cv::waitKey(1);
    
        // restore original image
        this->data = data_copy;
    
        return data_save;

}


void Area::drawPath(){
    
    // get robot pose in continuous world coordinates
    Eigen::Vector2f robot_location = this->robot.getPose().block<2, 1>(0, 0);
    
    // get robot pose in discrete area coordinates
    Eigen::Vector2i area_location = this->discretize_world_location(robot_location);
    
    // draw robot as a black circle
    cv::circle(this->data, {area_location(0), area_location(1)}, 4, this->path_ground_truth_color, -1);
    
}


void Area::drawWalls(){
    
    // iterate over each wall
    for (vector<vector<float>>::iterator i = this->wall_coordinates.begin(); i != this->wall_coordinates.end(); i++) {
        
        // extract start and end point of the line segment and transform them to discrete area coordinates
        Eigen::Vector2f startpoint;
        Eigen::Vector2f endpoint;
        startpoint << (*i)[0], (*i)[1];
        endpoint << (*i)[2], (*i)[3];
        Eigen:: Vector2i area_startpoint = this->discretize_world_location(startpoint);
        Eigen:: Vector2i area_endpoint = this->discretize_world_location(endpoint);
        
        // draw a black line
        line(this->data, {area_startpoint(0), area_startpoint(1)}, {area_endpoint(0), area_endpoint(1)}, {0, 0, 0}, 2);
        
    }
    
    cv::namedWindow("Area", cv::WINDOW_AUTOSIZE);
    cv::imshow("Area", this->data);
    
}


void Area::drawScan(){
    
    // get robot location in world coordinates
    const Eigen::Vector2f robot_location = this->robot.getPose().block<2,1>(0,0);
    
    // get robot heading
    const float robot_heading = this->robot.getPose()(2);
    
    // get robot location in discrete area coordinates
    const Eigen::Vector2i area_robot_location = this->discretize_world_location(robot_location);
    
    // get reference to measurements
    const Eigen::MatrixX2f measurement_ref = this->robot.getSensor().getMeasurements();
    
    // get number of measurements
    const int n_beams = this->robot.getSensor().getN();
    
    // iterate over all measurements
    for (int beam_id = 0; beam_id < n_beams; beam_id++) {
        
        // get range and angle information for currently inspected measurement
        float phi = measurement_ref(beam_id, 0);
        float area_range = this->scale_world_variable(measurement_ref(beam_id, 1));
        
        // get final angle as sum of robot's heading and measurement angle relative to robot
        float alpha = phi + robot_heading;
        alpha = fmod((alpha+PI), (2.0*PI)) - PI; // keep angle within range of (-PI/2, PI/2]
        
        // get endpoint of the obtained measurement
        int line_endpoint_x = (int)(area_robot_location(0) + area_range * cos(alpha));
        int line_endpoint_y = (int)(area_robot_location(1) + area_range * sin(alpha));
        
        // draw line
        cv::line(this->data, {area_robot_location(0), area_robot_location(1)}, {line_endpoint_x, line_endpoint_y}, this->scan_color, 1);
    }
    
}


void Area::drawScanEstimate(){
    
    for (list<Particle>::iterator it = this->robot.getFilter().getParticles().begin(); it != this->robot.getFilter().getParticles().end(); it++) {
        
        // get particle location in continuous world coordinates
        const Eigen::Vector2f particle_location = (*it).getPose().block<2,1>(0,0);
        
        // get particle heading
        const float particle_heading = (*it).getPose()(2);
        
        // get particle location in discrete area coordinates
        const Eigen::Vector2i area_particle_location = this->discretize_world_location(particle_location);
        
        // get number of measurements
        const int n_beams = this->robot.getSensor().getN();
        
        // get reference to measurements
        const Eigen::MatrixX2f measurement_estimate_ref = (*it).getMeasurementEstimate();
        
        // iterate over all measurements
        for (int beam_id = 0; beam_id < n_beams; beam_id++) {
            
            // get range and angle information for currently inspected measurement
            float phi = measurement_estimate_ref(beam_id, 0);
            float area_range = this->scale_world_variable(measurement_estimate_ref(beam_id, 1));
            
            // get final angle as sum of robot's heading and measurement angle relative to robot
            float alpha = phi + particle_heading;
            alpha = fmod((alpha+PI), (2.0*PI)) - PI; // keep angle within range of (-PI/2, PI/2]
            
            // get endpoint of the obtained measurement
            int line_endpoint_x = (int)(area_particle_location(0) + area_range * cos(alpha));
            int line_endpoint_y = (int)(area_particle_location(1) + area_range * sin(alpha));
            
            // draw line
            cv::line(this->data, {area_particle_location(0), area_particle_location(1)}, {line_endpoint_x, line_endpoint_y}, this->scan_estimate_color, 1);
        }
    }
}


void Area::drawRobot(){
    
    // get robot location in world coordinates
    const Eigen::Vector2f robot_location = this->robot.getPose().block<2,1>(0,0);
    
    // get robot heading
    const float robot_heading = this->robot.getPose()(2);
    
    // get robot location in discrete area coordinates
    const Eigen::Vector2i area_robot_location = this->discretize_world_location(robot_location);
    
    // get robot radius in image coordinates
    int area_radius = this->scale_world_variable(this->robot.getRadius());
    
    // draw robot as a circle
    cv::circle(this->data, {area_robot_location(0), area_robot_location(1)}, area_radius, this->robot_color, -1);
    
    // indicate orientation of the robot by drawing a line from the origin
    float line_scale = 1.0; // * robot radius
    int line_endpoint_x = (int)(area_robot_location(0) + line_scale * area_radius * cos(robot_heading));
    int line_endpoint_y = (int)(area_robot_location(1) + line_scale * area_radius * sin(robot_heading));
    cv::line(this->data, {area_robot_location(0), area_robot_location(1)}, {line_endpoint_x, line_endpoint_y}, {255, 255, 255}, 2);
    
    // draw black circle around robot
    cv::circle(this->data, {area_robot_location(0), area_robot_location(1)}, area_radius, {0, 0, 0}, 2);

    
}


// draw Particles
void Area::drawParticles(){
    
    for (list<Particle>::iterator it = this->robot.getFilter().getParticles().begin(); it != this->robot.getFilter().getParticles().end(); it++) {
        
        // get particle location in continuous world coordinates
        const Eigen::Vector2f particle_location = (*it).getPose().block<2,1>(0,0);
        
        // get particle location in discrete area coordinates
        const Eigen::Vector2i area_particle_location = this->discretize_world_location(particle_location);
        
        // draw robot as a black circle
        cv::circle(this->data, {area_particle_location(0), area_particle_location(1)}, 6, this->particle_color, -1);
        
    }
    
}


// *********************
// Coordinate Transforms
// *********************

// coordinate transformation
Eigen::Vector2i Area::discretize_world_location(const Eigen::Vector2f& world_location){
    
    Eigen::Vector2i area_location; // pose in discrete area coordinates
    
    area_location(0) = (int)((world_location(0) - x_min) / (x_max - x_min) * data_dim_x);
    area_location(1) = (int)((world_location(1) - y_min) / (y_max - y_min) * data_dim_y);
    
    return area_location;
    
}


int Area::scale_world_variable(const float& world_variable){
    
    int area_variable;
    area_variable = (int)(world_variable / this->resolution);
    return area_variable;
    
}

void Area::summary(){
    
    cout << "Area: " << endl;
    cout << "-----" << endl;
    cout << "Width: " << this->x_max - this->x_min << "m" << endl;
    cout << "Height: " << this->y_max - this->y_min << "m" << endl;
    Map::summary();
    this->getRobot().summary();

}
