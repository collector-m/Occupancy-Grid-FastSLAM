//
//  ScanMatcher.h
//  FastSLAM
//
//  Created by Mats Steinweg on 30.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef ScanMatcher_h
#define ScanMatcher_h

#include <Eigen/Dense>

using namespace std;

typedef struct {
    vector<float> distances;
    vector<int> indices;
} nn_result;

class ScanMatcher {
    
    public:
    // constructor and destructor
    ScanMatcher();
    ScanMatcher(int max_iterations, float tolerance, float discard_fraction);
    ~ScanMatcher(){};
    
    // perform ICP
    Eigen::Array3f ICP(Eigen::MatrixX2f &A, Eigen::MatrixX2f &B);
    Eigen::Matrix3f fit_transform(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B);
    nn_result nearest_neighbor(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B);
    
    private:
    int max_iterations;
    float tolerance;
    float discard_fraction;
    
    
};

#endif /* ScanMatcher_h */
