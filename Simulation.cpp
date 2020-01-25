//
//  Simulation.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <fstream>

#include "Simulation.h"
#include "Sensor.h"

class Map;

using namespace std;
using namespace Eigen;

#define PI 3.14159265


////////////////////////
// Read-In Functions //
////////////////////////

// helper function for reading in map file. convert line into separate float values.
void Simulation::read_wall_file(){
    
    std::ifstream wall_file(this->wall_filename);
    std::string wall_line;
    
    if (wall_file.is_open()) {
        while (getline (wall_file, wall_line)) {
            if (not wall_line.empty()) {
                
                // output vector of float coordinates
                vector<float> line_vector;
                
                // transform string to stringstream to use split method
                stringstream ss(wall_line);
                
                // container for each item in line string
                string item;
                
                // separate line into single values
                while (getline (ss, item, ' ')) {
                    
                    // convert string item to float value
                    float value;
                    value = strtof((item).c_str(), 0);
                    
                    // append to line vector
                    line_vector.push_back (value);
                }
            
            // append coordinates for current line to map coordinates
            this->wall_coordinates.push_back(line_vector);
            
            }
        }
    }
    
    // close file
    wall_file.close();
}


// helper function for reading in parameter file. convert line into name and float value.
void Simulation::read_parameter_file() {
    
    std::ifstream parameter_file (this->parameter_filename);
    std::string parameter_line;
    
    vector<Parameter> parameters;
    if (parameter_file.is_open()) {
        while (getline (parameter_file, parameter_line)) {
            if (not parameter_line.empty()) {
                if (parameter_line.at(0) != '#') {
                    
                    // transform string to stringstream to use split method
                    stringstream ss(parameter_line);
                    
                    // container for each item in line string
                    string item;
                    
                    // boolean values to ensure correct structure of output
                    bool name_read = false;
                    bool value_read = false;
                    
                    // container for output
                    Parameter parameter;
                    
                    // separate line into single values
                    while (getline (ss, item, '=')) {
                        // name not yet read
                        if (not name_read){
                            parameter.name = item.substr(0, item.size()-1);
                            name_read = true;
                        }
                        // name already read in but value not
                        else if (name_read && not value_read){
                            float value;
                            value = strtof((item).c_str(), 0);
                            parameter.value = value;
                            value_read = true;
                        }
                        else {
                            cout << "Parameter File Format Error!" << endl;
                            exit(1);
                        }
                    }
                            
                    // append parameter to vector of simulation parameters
                    this->parameters.push_back(parameter);
                }
            }
        }
    }
    
    // close file
    parameter_file.close();
    
    // set read-in parameters
    this->setParamters();
}


// helper function to get parameter from enum given the name
parameter_id get_id(const string& parameter_name){
    
    int n_parameters = 21;
    
    int match_id = n_parameters;
    for (int i = 0; i < n_parameters; i++){
        
        int different = parameter_name.compare(parameter_names[i]);
        if (not different){
            match_id = i;
            break;
        }
    }
    return (parameter_id) match_id;
    
}


// set parameters to values read-in from text file
void Simulation::setParamters(){
    
    // set parameters from text file
    for (vector<Parameter>::iterator it = this->parameters.begin(); it != this->parameters.end(); it++){
        
        parameter_id parameter_identifier = get_id((*it).name);
        switch (parameter_identifier){
                // area parameters
            case xmin: this->x_min = (*it).value;
                break;
            case xmax: this->x_max = (*it).value;
                break;
            case ymin: this->y_min = (*it).value;
                break;
            case ymax: this->y_max = (*it).value;
                break;
            case AreaResolution: this->resolution = (float)(*it).value;
                break;
                // Sensor Parameters
            case FOV: this->FoV = (int)(*it).value;
                break;
            case Range: this->range = (*it).value;
                break;
            case SensorResolution: this->sensor_resolution = (*it).value;
                break;
                // Filter Parameters
            case nParticles: this->n_particles = (int)(*it).value;
                break;
            case Rx: this->R(0) = (*it).value;
                break;
            case Ry: this->R(1) = (*it).value;
                break;
            case Rt: this->R(2) = (*it).value;
                break;
                // Scan Matcher
            case MaxIterations: this->max_iterations = (int)(*it).value;
                break;
            case Tolerance: this->tolerance = (*it).value;
                break;
            case DiscardFraction: this->discard_fraction = (*it).value / 100;
                break;
                // Error Case
            case Error: cout << "No corresponding variable for parameter " << (*it).name << endl;
                break;
            default: break;
        }
    }
}


// helper function for reading in control signal file. convert line into translational and angular velocity.
void Simulation::read_control_signal_file() {
    
    std::ifstream control_signal_file (this->control_signal_filename);
    std::string control_signal_line;
    
    // check if sampling time already read
    bool sampling_time_read = false;
    
    if (control_signal_file.is_open()) {
        while (getline (control_signal_file, control_signal_line)) {
            if (not control_signal_line.empty()) {
                if (control_signal_line.at(0) != '#') {
                    
                    // set sampling time
                    if (not control_signal_line.substr(0, 2).compare("Ts") && not sampling_time_read){
                        this->sampling_time = strtof((control_signal_line.substr(5, control_signal_line.size())).c_str(), 0);
                        sampling_time_read = true;
                        continue;
                    }
                    
                    // transform string to stringstream to use split method
                    stringstream ss(control_signal_line);
                    
                    // container for each item in line string
                    string item;
                    
                    // boolean values to ensure correct structure of output
                    bool v_read = false;
                    bool omega_read = false;

                    // store values in vector
                    Eigen::Vector2f control_signal;

                    // separate line into single values
                    while (getline (ss, item, ',')) {
                        // name not yet read
                        if (not v_read){
                            control_signal(0) = strtof((item).c_str(), 0);
                            v_read = true;
                        }
                        // name already read in but value not
                        else if (v_read && not omega_read){
                            control_signal(1) = strtof((item).c_str(), 0) / 180 * PI;
                            omega_read = true;
                        }
                        else {
                            cout << "Parameter File Format Error!" << endl;
                            exit(1);
                        }
                    }
                    
                    // append parameter to vector of control signals
                    this->control_signals.push_back(control_signal);
                }
            }
        }
    }
    
    // close file
    control_signal_file.close();
}


//////////////////
// Constructor //
/////////////////

Simulation::Simulation(const string& wall_filename, const string& parameter_filename, const string& control_signal_filename){
    
    //////////////////////////
    // read-in information //
    /////////////////////////
    
    // set filename variables
    this->wall_filename = wall_filename;
    this->parameter_filename = parameter_filename;
    this->control_signal_filename = control_signal_filename;
    
    // read in wall coordinates
    this->read_wall_file();
    
    // read in simulation parameters
    this->read_parameter_file();
    
    // read in control signals
    this->read_control_signal_file();
    
    ////////////////////////////////
    // create simulation objects //
    ///////////////////////////////
    
    // create area
    this->area = Area(wall_coordinates, x_min, x_max, y_min, y_max, resolution);

    // create robot and place robot in area
    Robot robot = Robot();
    this->area.setRobot(robot);
    
    ////////////////////////////
    // initialize simulation //
    ///////////////////////////
    
    // set time variables
    this->start_time = 0.0; // start time of the simulation
    this->end_time = this->control_signals.size() * this->sampling_time + this->start_time; // end time of the simulation computed from number of read-in control signals and specified sampling time
    
    // draw initial scene
    this->area.drawInitialScene();
}


/////////////////////
// run simulation //
////////////////////

void Simulation::run(){
    
    // set simulation time to specified start time
    this->simulation_time = this->start_time;
    
    // get reference to robot
    Robot& robot = this->area.getRobot();
    
    // iterate over provided control signals (length and sampling time specify duration of the simulation)
    for (vector<Eigen::Vector2f>::iterator it = this->control_signals.begin(); it != this->control_signals.end(); it++){
        
        // set robot's velocities to current control signal
        robot.setV((*it)(0));
        robot.setOmega((*it)(1));
        
        // drive robot
        robot.drive(this->sampling_time);

        // draw simulation
        this->area.drawScene();
        
        // set current simulation time
        this->simulation_time += this->sampling_time;
        
    }
}
