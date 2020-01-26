//
//  Simulation.h
//  FastSLAM
//
//  Created by Mats Steinweg on 17.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Simulation_h
#define Simulation_h

#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Robot.h"
#include "Area.h"

using namespace std;

// Struct to store parameter values from text file
typedef struct {
    string name;
    float value;
} Parameter;

// Struct to store save options
typedef struct {
    bool save;
    int save_frequency;
    string result_dir;
} SaveOptions;

// Enum for reading in relevant simulation parameters
enum parameter_id{
    xmin,
    xmax,
    ymin,
    ymax,
    AreaResolution,
    MapResolution,
    OccupancyValueMin,
    OccupancyValueMax,
    OccupancyValueStep,
    nParticles,
    Rx,
    Ry,
    Rt,
    FOV,
    Range,
    SensorResolution,
    Qr,
    Qt,
    MaxIterations,
    Tolerance,
    DiscardFraction,
    Error
};

// String names of relevant simulation parameters
const string parameter_names []{
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "area_resolution",
    "map_resolution",
    "occupancy_value_min",
    "occupancy_value_max",
    "occupancy_value_step",
    "n_particles",
    "R_x",
    "R_y",
    "R_t",
    "FoV",
    "range",
    "sensor_resolution",
    "Q_r",
    "Q_t",
    "max_iterations",
    "tolerance",
    "discard_fraction",
};

// String names of simulation modes
const string simulation_modes [] {
    "Localization",
    "Mapping",
    "SLAM",
};


class Simulation {
    
    public:
        // Constructor and destructor
        Simulation(const string& wall_filename, const string& parameter_filename, const string& control_signal_filename, const int simulation_mode, int verbose, SaveOptions save_options);
        ~Simulation(){};
    
        // Read-in functions
        void read_wall_file();
        void read_parameter_file();
        void read_control_signal_file();
    
        // Set simulation parameters
        void setParamters();

        // Getter functions
        Area& getArea(){ return this->area; };
        const vector<vector<float>>& getWallCoordinates(){ return this->wall_coordinates; };
    
        // Run simulation
        void run();
    
        // Print summary
        void summary();
    
        // Save results
        void save_image(cv::Mat data, string save_dir, string name_prefix);
        void save_results(const vector<Eigen::Vector2f>::iterator it);

    private:
    
        // Simulation mode
        int simulation_mode;
    
        // Verbosity level
        int verbose;
    
        // Save options
        SaveOptions save_options;
    
        // Sensor parameters
        int FoV;
        float range;
        float sensor_resolution;
        Eigen::Vector2f Q;
    
        // Filter parameters
        int n_particles;
        Eigen::Vector3f R;
    
        // ScanMatcher parameters
        int max_iterations;
        float tolerance;
        float discard_fraction;
    
        // Area parameters
        float x_min;
        float x_max;
        float y_min;
        float y_max;
        float resolution;
    
        // Simulation time
        float start_time;
        float sampling_time;
        float end_time;
        float simulation_time;
    
        // Area
        Area area;
    
        // Read-in filenames and contents
        string wall_filename;  // txt file specifying wall coordinates
        string parameter_filename;  // txt file specifying all simulation parameters
        string control_signal_filename; // txt file specifying all control signals
        vector<vector<float>> wall_coordinates; // vector containing wall coordinates
        vector<Parameter> parameters; // vector containing all simulation parameters
        vector<Eigen::Vector2f> control_signals; // vector containing all control signals

};

#endif /* Simulation_h */
