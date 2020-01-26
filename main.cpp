//
//  main.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 07.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include "Particle.h"
#include "Map.h"
#include <list>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "Robot.h"
#include "ScanMatcher.h"
#include "Simulation.h"
#include <typeinfo>

using namespace std;
using namespace cv;

int main(int argc, const char * argv[]) {
    
    
    // Set simulation mode | 0 = Localization, 1 = Mapping, 2 = SLAM
    int simulation_mode = 2;
    
    // Set verbosity level
    int verbose = 2;
    
    // Set save options
    SaveOptions save_options;
    save_options.save = true;
    save_options.save_frequency = 0;  // frequency = 0 -> only save results at last timestep
    save_options.result_dir = "Results";
    
    // Create simulation
    string data_dir = "Data";
    string walls_file_path = data_dir + "/" + "walls.txt";
    string parameters_file_path = data_dir + "/" + "parameters.txt";
    string control_signals_file_path = data_dir + "/" + "control_signals.txt";
    Simulation *simulation = new Simulation(walls_file_path, parameters_file_path,
                                            control_signals_file_path, simulation_mode, verbose, save_options);
    
    // Run simulation
    simulation->run();
    
    // Wait for keypress to close window
    cv::waitKey();
    
    return 0;
}
