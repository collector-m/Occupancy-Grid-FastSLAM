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
    
    // create simulation
    Simulation *simulation = new Simulation("walls.txt", "parameters.txt", "control_signals.txt");
    
    // run simulation
    simulation->run();
    
    cv::waitKey();

    return 0;
}
