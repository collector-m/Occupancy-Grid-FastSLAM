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

// struct to store parameter values from text file
typedef struct {
    string name;
    float value;
} Parameter;

// struct to store save options
typedef struct {
    bool save;
    int save_frequency;
    string result_dir;
} SaveOptions;

// enum for reading in relevant simulation parameters
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

// string names of relevant simulation parameters
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

// simulation modes
const string simulation_modes [] {
    "Localization",
    "Mapping",
    "SLAM",
};


class Simulation {
    
    public:
        // constructor and destructor
        Simulation(const string& wall_filename, const string& parameter_filename, const string& control_signal_filename, int simulation_mode, int verbose, SaveOptions save_options);
        ~Simulation(){};
    
        // read-in functions
        void read_wall_file();
        void read_parameter_file();
        void read_control_signal_file();
    
        // set simulation parameters
        void setParamters();

        // getter functions
        Area& getArea(){ return this->area; };
        const vector<vector<float>>& getWallCoordinates(){ return this->wall_coordinates; };
    
        // run simulation
        void run();
    
        // print summary
        void summary();
    
        // save results
        void save_scene(cv::Mat scene_data);
        void save_map(cv::Mat map_data);

    private:
        ////////////////////////////
        // simulation parameters //
        ///////////////////////////
    
        // Simulation mode
        int simulation_mode;
    
        // Verbose level
        int verbose;
    
        // Save options
        SaveOptions save_options;
    
        // Sensor parameters
        int FoV;
        float range;
        float sensor_resolution;
    
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
    
        // simulation time
        float start_time;
        float sampling_time;
        float end_time;
        float simulation_time;
    
        // area
        Area area;
    
        // read-in filenames and contents
        string wall_filename;  // txt file specifying wall coordinates
        string parameter_filename;  // txt file specifying all simulation parameters
        string control_signal_filename; // txt file specifying all control signals
        vector<vector<float>> wall_coordinates; // vector containing wall coordinates
        vector<Parameter> parameters; // vector containing all simulation parameters
        vector<Eigen::Vector2f> control_signals; // vector containing all control signals

};

#endif /* Simulation_h */
