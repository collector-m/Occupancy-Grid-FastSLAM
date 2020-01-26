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
    
    
    // set simulation mode | 0 = Localization, 1 = Mapping, 2 = SLAM
    int simulation_mode = 1;
    
    // set verbosity level
    int verbose = 2;
    
    // set save options
    SaveOptions save_options;
    save_options.save = true;
    save_options.save_frequency = 10;
    save_options.result_dir = "Results";
    
    // create simulation
    Simulation *simulation = new Simulation("Data/walls.txt", "Data/parameters.txt",
                                            "Data/control_signals.txt", simulation_mode, verbose, save_options);
    
    // run simulation
    simulation->run();
    
    cv::waitKey();

    return 0;
}
