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
#include <filesystem>

#include "Simulation.h"
#include "Sensor.h"

class Map;

using namespace std;
using namespace Eigen;

#define PI 3.14159265


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Simulation::Simulation(const string& wall_filename, const string& parameter_filename, const string& control_signal_filename, const int simulation_mode, int verbose, SaveOptions save_options){
    
    
    // +++++++++++++++++++++++ Set simulation parameters and read-in files +++++++++++++++++++++++++++++++++++
    
    // Set simulation mode
    this->simulation_mode = simulation_mode;
    
    // Set verbosity level
    if (simulation_mode == 1 && verbose > 0){
        verbose = 0;
        cout << "Verbosity level set to 0 in Mapping mode. Particles and scan estimates won't be displayed." << endl;
    } // Set verbose to 0 in mapping mode
    this->verbose = verbose;
    
    // Set save options
    this->save_options = save_options;
    // Check if specified result directory exists and create if not
    bool result_dir_exists = std::__fs::filesystem::exists(save_options.result_dir);
     if (save_options.save == true && result_dir_exists == false){
         std::__fs::filesystem::create_directory(save_options.result_dir);
     }
    
    // Set filename variables
    this->wall_filename = wall_filename;
    this->parameter_filename = parameter_filename;
    this->control_signal_filename = control_signal_filename;
    
    // Read in wall coordinates
    this->read_wall_file();
    
    // Read in simulation parameters
    this->read_parameter_file();
    
    // Read in control signals
    this->read_control_signal_file();
    

    // +++++++++++++++++++++++ Create and initialize simulation components +++++++++++++++++++++++++++++++++++

    // Create area object
    this->area = Area(wall_coordinates, x_min, x_max, y_min, y_max, resolution);

    // Create all required robot components (Sensor and Rao-Blackwellized Particle Filter)
    if (this->simulation_mode == 1){ this->n_particles = 1; }  // set n_particles to 1 in mapping mode
    RBPF filter = RBPF(n_particles, R, max_iterations, tolerance, discard_fraction);
    Sensor sensor = Sensor(FoV, range, sensor_resolution, Q);
    
    // Create robot object
    Robot robot = Robot();
    
    // Set robot components
    robot.setFilter(filter);
    robot.setSensor(sensor);
    
    // Place robot in area
    this->area.setRobot(robot);
    
    // Perform initial sensor sweep and mapping
    this->area.getRobot().getSensor().sweep(this->wall_coordinates, this->area.getRobot().getPose());
    this->area.getRobot().getFilter().mapping(this->area.getRobot().getSensor());
    this->area.getRobot().getFilter().sweep_estimate(this->area.getRobot().getSensor());
    
    // Set time variables
    this->start_time = 0.0; // Start time of the simulation
    this->end_time = this->control_signals.size() * this->sampling_time + this->start_time; // End time of the simulation computed from number of provided control signals and specified sampling time
    
    // Print simulation summary
    this->summary();
    
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ Read-in functions ++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Helper function for reading in map file. Convert each line into separate float values.
// Each wall consists of four float values: [x_start, y_start, x_end, y_end].
void Simulation::read_wall_file(){
    
    std::ifstream wall_file(this->wall_filename);
    std::string wall_line;
    
    if (wall_file.is_open()) {
        while (getline (wall_file, wall_line)) {
            if (not wall_line.empty()) {
                
                // Output vector of float coordinates
                vector<float> line_vector;
                
                // Transform string to stringstream to use split method
                stringstream ss(wall_line);
                
                // Container for each item in line string
                string item;
                
                // Separate line into single values
                while (getline (ss, item, ' ')) {
                    
                    // Convert string item to float value
                    float value;
                    value = strtof((item).c_str(), 0);
                    
                    // Append value to line vector
                    line_vector.push_back (value);
                }
            
            // Append coordinates for current line to wall coordinates
            this->wall_coordinates.push_back(line_vector);
            
            }
        }
    }
    
    // Close file
    wall_file.close();
}


// Helper function for reading in parameter file. Convert each line into name and float value.
// Each parameter is specified as follows: "parameter_name" = parameter_value, comments start with "#"
void Simulation::read_parameter_file() {
    
    std::ifstream parameter_file (this->parameter_filename);
    std::string parameter_line;
    
    vector<Parameter> parameters;
    if (parameter_file.is_open()) {
        while (getline (parameter_file, parameter_line)) {
            if (not parameter_line.empty()) {
                if (parameter_line.at(0) != '#') {
                    
                    // Transform string to stringstream to use split method
                    stringstream ss(parameter_line);
                    
                    // Container for each item in line string
                    string item;
                    
                    // Boolean flags to ensure correct structure of output
                    bool name_read = false;
                    bool value_read = false;
                    
                    // Container for output
                    Parameter parameter;
                    
                    // Separate line into single values
                    while (getline (ss, item, '=')) {
                        // Name not yet read
                        if (not name_read){
                            parameter.name = item.substr(0, item.size()-1);
                            name_read = true;
                        }
                        // Name already read-in but value not
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
                            
                    // Append parameter to vector of simulation parameters
                    this->parameters.push_back(parameter);
                }
            }
        }
    }
    
    // Close file
    parameter_file.close();
    
    // Set read-in parameters
    this->setParamters();
}


// Helper function to get parameter from enum given the name
parameter_id get_id(const string& parameter_name){
    
    // Get total number of parameters
    int n_parameters = sizeof(parameter_names)/sizeof(parameter_names[0]);;
    
    // Match input string with corresponding parameter ID
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


// Set parameters to values read-in from text file
void Simulation::setParamters(){
    
    // Iterate over all read-in parameters
    for (vector<Parameter>::iterator it = this->parameters.begin(); it != this->parameters.end(); it++){
        
        // Get parameter ID
        parameter_id parameter_identifier = get_id((*it).name);
        
        // Assign value to corresponding member variable
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
            case Qr: this->Q(0) = (*it).value;
                break;
            case Qt: this->Q(1) = (*it).value;
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


// Helper function for reading in control signal file. Convert line into translational and angular velocity.
// Each control signal is specified as follows: translation velocity in m/s, angular velocity in °/s,
// comments start with "#"
void Simulation::read_control_signal_file() {
    
    std::ifstream control_signal_file (this->control_signal_filename);
    std::string control_signal_line;
    
    // Check if sampling time already read
    bool sampling_time_read = false;
    
    if (control_signal_file.is_open()) {
        while (getline (control_signal_file, control_signal_line)) {
            if (not control_signal_line.empty()) {
                if (control_signal_line.at(0) != '#') {
                    
                    // Set sampling time
                    if (not control_signal_line.substr(0, 2).compare("Ts") && not sampling_time_read){
                        this->sampling_time = strtof((control_signal_line.substr(5, control_signal_line.size())).c_str(), 0);
                        sampling_time_read = true;
                        continue;
                    }
                    
                    // Transform string to stringstream to use split method
                    stringstream ss(control_signal_line);
                    
                    // Container for each item in line string
                    string item;
                    
                    // Boolean values to ensure correct structure of output
                    bool v_read = false;
                    bool omega_read = false;

                    // Store values in vector
                    Eigen::Vector2f control_signal;

                    // Separate line into single values
                    while (getline (ss, item, ',')) {
                        // translational velocity not yet read
                        if (not v_read){
                            control_signal(0) = strtof((item).c_str(), 0);
                            v_read = true;
                        }
                        // translational velocity already read-in but angular velocity not
                        else if (v_read && not omega_read){
                            control_signal(1) = strtof((item).c_str(), 0) / 180 * PI;
                            omega_read = true;
                        }
                        else {
                            cout << "Control Signal File Format Error!" << endl;
                            exit(1);
                        }
                    }
                    
                    // Append control signal to vector of control signals
                    this->control_signals.push_back(control_signal);
                }
            }
        }
    }
    
    // Close file
    control_signal_file.close();
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++ Print Summary ++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Print simulation summary
void Simulation::summary(){
    
    cout << "Simulation Summary" << endl;
    cout << "++++++++++++++++++" << endl;
    cout << "Mode: " << simulation_modes[this->simulation_mode] << endl;
    cout << "Verbosity: " << this->verbose << endl;
    // Print area summary
    this->getArea().summary();
    cout << "++++++++++++++++++" << endl;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++ Run simulation ++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Run simulation main loop
void Simulation::run(){
    
    // Set simulation time to specified start time
    this->simulation_time = this->start_time;
    
    // Draw initial scene, draw initial map in mapping and SLAM mode
    this->area.drawScene(this->verbose);
    if (this->simulation_mode == 1 || this->simulation_mode == 2){
        this->getArea().getRobot().getFilter().getMap().draw();}
    
    // Wait for keypress to start the simulation
    cout << "Press key to start simulation!" << endl;
    cv::waitKey();
    
    // Get reference to robot
    Robot& robot = this->area.getRobot();
    
    // Iterate over provided control signals (number of specified control signals and sampling time specify
    // duration of the simulation)
    for (vector<Eigen::Vector2f>::iterator it = this->control_signals.begin(); it != this->control_signals.end(); it++){
        
        // Set robot's velocities to current control signal
        robot.setV((*it)(0));
        robot.setOmega((*it)(1));
        
        // Drive robot
        robot.drive(this->sampling_time);
        
        // Sensor sweep
        robot.getSensor().sweep(this->wall_coordinates, robot.getPose());
        
        // Run particle filter
        robot.getFilter().run(robot, this->simulation_mode);
        
        // Set current simulation time
        this->simulation_time += this->sampling_time;
        
        // Draw simulation
        this->area.drawScene(this->verbose);
        // Draw map only for simulation mode mapping and SLAM
        if (this->simulation_mode == 1 || this->simulation_mode == 2){
            this->getArea().getRobot().getFilter().getMap().draw();}
        
        // Save results
        this->save_results(it);
        
    }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++ Save results ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Save results
void Simulation::save_results(vector<Eigen::Vector2f>::iterator it){
    
    // Set frequency trigger based on current iteration
    bool frequency_trigger;
    int current_iteration = (int) ((this->simulation_time +
                                    this->sampling_time/2) / this->sampling_time);
    // If save frequency is 0, only save results after last iteration
    if (save_options.save_frequency == 0){
        if (next(it) == this->control_signals.end()){
            frequency_trigger = true;}
        else {
            frequency_trigger = false;}
    }
    // If frequency > 0, trigger result saving according to specified frequency
    else {
        frequency_trigger = (bool) ((current_iteration % save_options.save_frequency) == 0);}

    // Save scene image in all modes and map in mapping and SLAM mode
    if (save_options.save == true && frequency_trigger == true){
        cv::Mat scene_data = this->getArea().getData();
        this->save_image(scene_data, "Scenes", "scene");
        if (this->simulation_mode == 1 || this->simulation_mode == 2){
            cv::Mat map_data = this->getArea().getRobot().getFilter().getMap().getData();
            this->save_image(map_data, "Maps", "map");
        }
    }
}

// Save image to file
void Simulation::save_image(cv::Mat data, string save_dir, string name_prefix){
    
    // Check if "Maps" directory exists and create if not
    bool scene_dir_exists = std::__fs::filesystem::exists(save_options.result_dir + "/" + save_dir);
    if (scene_dir_exists == false){
        std::__fs::filesystem::create_directory(save_options.result_dir + "/" + save_dir);
    }
    
    // Compute current iteration as image index
    int current_iteration = (int) ((this->simulation_time + this->sampling_time/2) / this->sampling_time);
    
    // Save image in specified directory
    string file_name = name_prefix + "_" + to_string(current_iteration) + ".png";
    string file_path = this->save_options.result_dir + "/" + save_dir + "/" + file_name;
    cv::imwrite(file_path, data);
}


