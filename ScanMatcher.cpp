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
#include <numeric>

#include "ScanMatcher.h"

using namespace Eigen;
using namespace std;



// standard constructor
ScanMatcher::ScanMatcher(){
    
    this->max_iterations = 20;
    this->tolerance = 0.01;
    this->discard_fraction = 0.1;
    
}


// constructor
ScanMatcher::ScanMatcher(int max_iterations, float tolerance, float discard_fraction): max_iterations(max_iterations), tolerance(tolerance) {
    
    this->discard_fraction = discard_fraction;
}


// fit transform
Eigen::Matrix3f ScanMatcher::fit_transform(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B){
    
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity(3,3);
    
    //cout << "Transformation Matrix: \n " << T << endl;

    Eigen::Vector2f centroid_A = A.colwise().mean();
    Eigen::Vector2f centroid_B = B.colwise().mean();
    
    //cout << "Centroid A: \n" << centroid_A << endl;
    //cout << "Centroid B: \n" << centroid_B << endl;

    Eigen::MatrixX2f AA = (A).rowwise() - centroid_A.transpose();
    Eigen::MatrixX2f BB = (B).rowwise() - centroid_B.transpose();
    
    //cout << "Centered A: \n" << AA << endl;
    //cout << "Centered B: \n" << BB << endl;


    Eigen::MatrixX2f H = AA.transpose() * BB;
    
    //cout << "H: \n" << H << endl;
    
    Eigen::MatrixXf U;
    Eigen::VectorXf S;
    Eigen::MatrixXf V;
    Eigen::MatrixXf Vt;
    Eigen::MatrixXf R;
    Eigen::Vector2f t;
    
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    
    //cout << "U: \n" << U << endl;
    //cout << "S: \n" << S << endl;
    //cout << "V: \n" << V << endl;

    // rotation matrix
    R = V * U.transpose();
    
    // special reflection
    if (R.determinant() < 0 ){
        V.block<2,1>(0,1) *= -1;
        R = V * U.transpose();
    }
    
    //cout << "R: \n" << R << endl;

    t = centroid_B - R * centroid_A;
    
    //cout << "t: \n" << t << endl;
    
    T.block<2,2>(0,0) = R;
    T.block<2,1>(0,2) = t;
    
    //cout << "T: \n" << T << endl;
    
    return T;

}


// find nearest neighbor
nn_result ScanMatcher::nearest_neighbor(const Eigen::MatrixX2f &A, const Eigen::MatrixX2f &B){
    
    nn_result result;
    
    for (int point_id = 0; point_id < A.rows(); point_id++) {
        Eigen::Array2f point = A.block<1,2>(point_id, 0).transpose();
        float min_distance = 10000;
        int min_index;
        for (int ref_id = 0; ref_id < B.rows(); ref_id++){
            Eigen::Array2f reference = B.block<1,2>(ref_id, 0).transpose();
            Eigen::Vector2f dif_vector = point - reference;
            float distance = dif_vector.squaredNorm();
            if (distance < min_distance){
                min_distance = distance;
                min_index = ref_id;
            }
        }
        result.distances.push_back(min_distance);
        result.indices.push_back(min_index);
    }
    
    return result;
}


// perform ICP algorithm
Eigen::Array3f ScanMatcher::ICP(Eigen::MatrixX2f &measurement_estimate, Eigen::MatrixX2f &measurement){
    
    Eigen::MatrixX2f A = measurement_estimate;
    Eigen::MatrixX2f B = measurement;
    
    // get the distance offset between measurement and estimate
    Eigen::VectorXf dif_norm  = (A - B).array().pow(2).rowwise().sum();
    
    // number of worst measurements to be discarded before scan matching
    int n_invalid = (int) (A.rows() * this->discard_fraction);
    
    // iterate over measurements to delete the worst
    for (int invalid_id = 0; invalid_id < n_invalid; invalid_id++){
        
        // get index of measurement with largest offset
        VectorXf::Index row_id;
        dif_norm.maxCoeff(&row_id);
        
        // delete corresponding row from both matrices
        A.block(row_id,0,A.rows()-1-row_id,A.cols()) = A.bottomRows(A.rows()-1-row_id);
        A.conservativeResize(A.rows()-1,A.cols());
        B.block(row_id,0,B.rows()-1-row_id,B.cols()) = B.bottomRows(B.rows()-1-row_id);
        B.conservativeResize(B.rows()-1,B.cols());
        
        // delete measurement from the list of offsets
        dif_norm.block(row_id, 0, dif_norm.rows()-1-row_id, 1) = dif_norm.bottomRows(dif_norm.rows()-1-row_id);
        dif_norm.conservativeResize(dif_norm.rows()-1,dif_norm.cols());
    }
    
    // homogeneous transformation matrix
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity(3, 3);
    
    //cout << "Transformation: \n" << T << endl;
    
    // pose difference
    Eigen::Array3f pose_dif = Eigen::Array3f::Zero(3, 1);
    
    float prev_error = 0;
    float mean_error = 0;
    for (int iter = 0; iter<this->max_iterations; iter++){
        
        // get neighbor information
        nn_result nn_info = nearest_neighbor(A, B);
    
        // homogeneous version of A
        Eigen::MatrixX3f A_hom = Eigen::MatrixX3f::Zero(A.rows(), A.cols()+1);
        A_hom(A_hom.rows()-1, A_hom.cols()-1) = 1.0;
        A_hom.block(0,0, A.rows(), A.cols()) = A;
        
        //cout << "A_hom: \n" << A_hom << endl;
        
        // container for transformed point cloud A
        Eigen::MatrixX2f A_trans = Eigen::MatrixX2f::Zero(A.rows(), A.cols());
        
        // version of B with nearest neighbor assigned to same index as in A
        Eigen::MatrixX2f B_ordered = Eigen::MatrixX2f::Zero(B.rows(), B.cols());
        
        for(int ref_id = 0; ref_id < B.rows(); ref_id++){
            B_ordered.block<1,2>(ref_id, 0) = B.block<1,2>(nn_info.indices[ref_id], 0);
        }
        
        //cout << "B_ordered: \n" << B_ordered << endl;
        
        // get best transformation matrix for current point clouds
        T = fit_transform(A, B);
        
        //cout << "Transform: \n" << T << endl;
        
        A_hom = (T * A_hom.transpose()).transpose();
        A_trans = A_hom.block(0,0, A.rows(), A.cols());
        
        //cout << "A_trans: \n" << A_trans << endl;
         
        // compute mean error
        mean_error = accumulate(nn_info.distances.begin(), nn_info.distances.end(), 0.0)/ nn_info.distances.size();
        if (abs(prev_error - mean_error) < this->tolerance){
            break;
        }
        prev_error = mean_error;
    
    }
    
    // final transform
    T = fit_transform(A, B);

    if (sqrt(pow(T(0, 2), 2) + pow(T(1, 2), 2)) < 0.2) {
        pose_dif(0) = T(0, 2);
        pose_dif(1) = T(1, 2);
        pose_dif(2) = atan2(T(1, 0), T(0, 0));
    }
    
    return pose_dif;
    
}


void ScanMatcher::summary(){
    
    cout << "Scan Matcher: " << endl;
    cout << "------------" << endl;
    cout << "Max Iterations: " << this->max_iterations << endl;
    cout << "Discard Fraction: " << this->discard_fraction << endl;
    cout << "Tolerance: " << this->tolerance << endl;

}
