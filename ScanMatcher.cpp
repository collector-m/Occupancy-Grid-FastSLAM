//
//  ScanMatcher.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 30.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <numeric>

#include "ScanMatcher.h"

using namespace Eigen;
using namespace std;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++ Constructor ++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Standard constructor
ScanMatcher::ScanMatcher(){
    
    this->max_iterations = 20;
    this->tolerance = 0.01;
    this->discard_fraction = 0.1;
    
}


// Constructor
ScanMatcher::ScanMatcher(int max_iterations, float tolerance, float discard_fraction): max_iterations(max_iterations), tolerance(tolerance) {
    
    this->discard_fraction = discard_fraction;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++ Print Summary ++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ScanMatcher::summary(){
    
    cout << "Scan Matcher: " << endl;
    cout << "------------" << endl;
    cout << "Max Iterations: " << this->max_iterations << endl;
    cout << "Discard Fraction: " << this->discard_fraction << endl;
    cout << "Tolerance: " << this->tolerance << endl;

}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ Iterative Closest Point ++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void draw_scan_matching(Eigen::MatrixX2f A, Eigen::MatrixX2f B){
    
    Eigen::VectorXf max_A = A.colwise().maxCoeff();
    Eigen::VectorXf min_A = A.colwise().minCoeff();
    Eigen::VectorXf max_B = B.colwise().maxCoeff();
    Eigen::VectorXf min_B = B.colwise().minCoeff();
    float x_min_A = min_A(0);
    float y_min_A = min_A(1);
    float x_max_A = max_A(0);
    float y_max_A = max_A(1);
    float x_min_B = min_B(0);
    float y_min_B = min_B(1);
    float x_max_B = max_B(0);
    float y_max_B = max_B(1);
    float x_min = min(x_min_A, x_min_B);
    float y_min = min(y_min_A, y_min_B);
    float x_max = min(x_max_A, x_max_B);
    float y_max = min(y_max_A, y_max_B);
    
    float width_factor;
    float height_factor;
    if ((x_max - x_min) > (y_max - y_min)){
        width_factor = 1.0;
        height_factor = (float)(y_max - y_min) / (float)(x_max - x_min);
    }
    else {
        height_factor = 1.0;
        width_factor = (float)(x_max - x_min) / (float)(y_max - y_min);
    }

    int width = (int) (600 * width_factor);
    int height = (int) (600 * height_factor);
    cv::Mat canvas = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(255));
    for(int ref_id = 0; ref_id < B.rows(); ref_id++){

        float x_a = A(ref_id, 0);
        float y_a = A(ref_id, 1);
        int x_a_d = (int)((x_a - x_min) / (x_max - x_min) * (width - 100) + 50);
        int y_a_d = (int)((y_a - y_min) / (y_max - y_min) * (height - 100) + 50);
        float x_b = B(ref_id, 0);
        float y_b = B(ref_id, 1);
        int x_b_d = (int)((x_b - x_min) / (x_max - x_min) * (width - 100) + 50);
        int y_b_d = (int)((y_b - y_min) / (y_max - y_min) * (height - 100) + 50);
        cv::circle(canvas, {x_a_d, y_a_d}, 2, {0, 255, 0}, 2);
        cv::circle(canvas, {x_b_d, y_b_d}, 2, {0, 0, 255}, 2);

    }
    cv::imshow("ScanMatching", canvas);
    cv::waitKey();
    
}


// Compute pose correction
Eigen::Vector3f ScanMatcher::ICP(Eigen::MatrixX2f &measurement_estimate, Eigen::MatrixX2f &measurement, const Eigen::Vector3f R){
    
    // Get measurements and measurement estimates
    Eigen::MatrixX2f A = measurement_estimate;
    Eigen::MatrixX2f B = measurement;
    
    // Get the distance offset between measurement and estimate
    Eigen::VectorXf dif_norm  = (A - B).array().pow(2).rowwise().sum();
    
    // Number of worst measurements to be discarded before scan matching
    int n_invalid = (int) (A.rows() * this->discard_fraction);
    
    // Iterate over measurements to delete the worst
    for (int invalid_id = 0; invalid_id < n_invalid; invalid_id++){
        
        // Get index of measurement with largest offset
        VectorXf::Index row_id;
        dif_norm.maxCoeff(&row_id);
        
        // Delete corresponding row from both matrices
        A.block(row_id,0,A.rows()-1-row_id,A.cols()) = A.bottomRows(A.rows()-1-row_id);
        A.conservativeResize(A.rows()-1,A.cols());
        B.block(row_id,0,B.rows()-1-row_id,B.cols()) = B.bottomRows(B.rows()-1-row_id);
        B.conservativeResize(B.rows()-1,B.cols());
        
        // Delete measurement from the list of offsets
        dif_norm.block(row_id, 0, dif_norm.rows()-1-row_id, 1) = dif_norm.bottomRows(dif_norm.rows()-1-row_id);
        dif_norm.conservativeResize(dif_norm.rows()-1,dif_norm.cols());
    }
    
    // Homogeneous transformation matrix
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity(3, 3);
        
    // Pose difference
    Eigen::Vector3f pose_dif = Eigen::Vector3f::Zero(3, 1);
    
    float prev_error = 0;
    float mean_error = 0;
    Eigen::MatrixX2f A_trans = Eigen::MatrixX2f::Zero(A.rows(), A.cols());
    A_trans = A.replicate(1, 1);

    for (int iter = 0; iter<this->max_iterations; iter++){
        
        //draw_scan_matching(A_trans, B);
        
        // Get neighbor information
        nn_result nn_info = nearest_neighbor(A_trans, B);
    
        // Homogeneous version of A
        Eigen::MatrixX3f A_hom = Eigen::MatrixX3f::Zero(A.rows(), A.cols()+1);
        A_hom(A_hom.rows()-1, A_hom.cols()-1) = 1.0;
        A_hom.block(0,0, A.rows(), A.cols()) = A_trans;
                
        // Container for transformed point cloud A
        //Eigen::MatrixX2f A_trans = Eigen::MatrixX2f::Zero(A.rows(), A.cols());
        
        // Version of B with nearest neighbor assigned to same index as in A
        Eigen::MatrixX2f B_ordered = Eigen::MatrixX2f::Zero(B.rows(), B.cols());
        
        for(int ref_id = 0; ref_id < B.rows(); ref_id++){
            B_ordered.block<1,2>(ref_id, 0) = B.block<1,2>(nn_info.indices[ref_id], 0);
        }
                
        // Get best transformation matrix for current point clouds
        Eigen::Matrix3f T_t = fit_transform(A_trans, B_ordered);
                        
        // Get transformed point cloud A
        A_hom = (T_t * A_hom.transpose()).transpose();
        A_trans = A_hom.block(0,0, A.rows(), A.cols());
        
        // Compute mean error
        mean_error = accumulate(nn_info.distances.begin(), nn_info.distances.end(), 0.0)/ nn_info.distances.size();
        if (abs(prev_error - mean_error) < this->tolerance || mean_error > prev_error){
            break;
        }
        cout << "Mean Error: " << mean_error << endl;
        
        // Update prev_error with current error
        prev_error = mean_error;
    
    }
    
    // Final transform
    T = fit_transform(A, A_trans);

    // Only apply pose correction if correction distance smaller than radius of 3 standard deviations of
    // motion model uncertainty
    if (sqrt(pow(T(0, 2), 2) + pow(T(1, 2), 2)) < sqrt(pow(3 * R(0), 2) + pow(3 * R(1), 2))) {
        pose_dif(0) = T(0, 2);
        pose_dif(1) = T(1, 2);
        pose_dif(2) = atan2(T(1, 0), T(0, 0));
    }
    
    return pose_dif;
    
}


// Fit transformation matrix
Eigen::Matrix3f ScanMatcher::fit_transform(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B){
    
    // Container for transformation matrix
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity(3,3);
    
    // Get centroid of each point cloud
    Eigen::Vector2f centroid_A = A.colwise().mean();
    Eigen::Vector2f centroid_B = B.colwise().mean();

    // Center point clouds
    Eigen::MatrixX2f AA = (A).rowwise() - centroid_A.transpose();
    Eigen::MatrixX2f BB = (B).rowwise() - centroid_B.transpose();

    // Compute matrix product
    Eigen::MatrixX2f H = AA.transpose() * BB;
    
    // Containers for SVD matrices
    Eigen::MatrixXf U;
    Eigen::VectorXf S;
    Eigen::MatrixXf V;
    Eigen::MatrixXf Vt;
    Eigen::MatrixXf R;
    Eigen::Vector2f t;
    
    // Compute SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();

    // Get rotation matrix
    R = V * U.transpose();
    
    // Special reflection
    if (R.determinant() < 0 ){
        V.block<2,1>(0,1) *= -1;
        R = V * U.transpose();
    }
    
    // Compute translational offset
    t = centroid_B - R * centroid_A;
    
    // Assign translation vector and rotation matrix
    // to homogeneous transformation matrix
    T.block<2,2>(0,0) = R;
    T.block<2,1>(0,2) = t;
        
    return T;

}


// Find nearest neighbor indices
nn_result ScanMatcher::nearest_neighbor(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B){
    
    // Container for NN-search result
    nn_result result;
    
    // Iterate over all points in point cloud A
    for (int point_id = 0; point_id < A.rows(); point_id++) {
        
        // Get current point
        Eigen::Array2f point = A.block<1,2>(point_id, 0).transpose();
        
        // Initialize min_distance
        float min_distance = 10000;
        
        // Container for index of nearest neighbor
        int min_index;
        
        // Iterate over all points in point cloud B
        for (int ref_id = 0; ref_id < B.rows(); ref_id++){
            
                // Get reference point
                Eigen::Array2f reference = B.block<1,2>(ref_id, 0).transpose();
                
                // Get difference vector and euclidean distance between points
                Eigen::Vector2f dif_vector = point - reference;
                float distance = dif_vector.squaredNorm();
                
                // Set new minumum index if new minumum distance
                if (distance < min_distance){
                    min_distance = distance;
                    min_index = ref_id;
                }
        }
        
        // Append nearest neighbor index and distance
        result.distances.push_back(min_distance);
        result.indices.push_back(min_index);
    }
    
    return result;
}
