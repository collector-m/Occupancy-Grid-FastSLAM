//
//  RBPF.cpp
//  FastSLAM
//
//  Created by Mats Steinweg on 20.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#include <stdio.h>
#include <random>
#include <math.h>
#include <algorithm>

#include "RBPF.h"
#include "Robot.h"

using namespace std;

#define PI 3.14159265

// standard constructor
RBPF::RBPF(){
    
    // set member variables
    this->n_particles = 5;
    this->R(0) = 0.01;
    this->R(1) = 0.01;
    this->R(2) = 0.01;
}

// constructor
RBPF::RBPF(int n_particles): scan_matcher(20, 0.001, 0.3){
    
    // set member variables
    this->n_particles = n_particles;
    this->R(0) = 0.01;
    this->R(1) = 0.01;
    this->R(2) = 0.01;
    
    // initialize particles
    for (int i = 0; i < this->n_particles; i++) {
        Particle particle = Particle();
        this->particles.push_back(particle);
    }
}

// summary of RBPF
void RBPF::summary(){
    
    cout << "+++++++++++++++++++++++++++++++ \n";
    cout << "RBPF Summary \n";
    cout << "Number of Particles: " << this->n_particles << endl;
    cout << "+++++++++++++++++++++++++++++++ \n";
    
}


// get current best map estimate
const cv::Mat& RBPF::getMap(){
    

    Particle* best_particle_ptr = 0;
    float best_particle_weight = 0.0;
    
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        if ((*it).getWeight() > best_particle_weight) {
            best_particle_weight = (*it).getWeight();
            best_particle_ptr = &(*it);
        }
    }
    return best_particle_ptr->getMap().getData();
    
}


// run filter
void RBPF::run(Robot& robot){
    
    cout << "+++++++++++++++++++++++++" << endl;
    // cout << "Robot: " << endl;
    // cout << robot.getPose() << endl;
    
    // run RBPF prediction
    this->predict(robot.getV(), robot.getOmega(), robot.getTimestamp());
    
    // get RBPF sensor estimate
    this->sweep_estimate(robot.getSensor(), robot.getPose());
    
    // run scan matching
    this->scan_matching(robot.getPose(), robot.getSensor());
    
    // weight particles
    this->weight(robot.getSensor());
    
    // resample particles
    this->resample();
    
    cout << "+++++++++++++++++++++++++" << endl;

    
}


// apply motion model
void RBPF::predict(const float &v, const float &omega, const float& current_timestamp){
    
    float delta_t = current_timestamp - this->last_timestamp;
    
    default_random_engine generator;
    normal_distribution<float> distribution(0.0, 1.0);
    
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        // pose difference due to control signal since last update
        Eigen::Array3f pose_dif;
        pose_dif(0) = delta_t * v * cos((float)(*it).getPose()(2));
        pose_dif(1) = delta_t * v * sin((float)(*it).getPose()(2));
        pose_dif(2) = delta_t * omega;
        
        // update pose
        (*it).getPose() += pose_dif;
        
        // apply uncorrelated noise
        //(*it).getPose()(0) += (this->R(0) * distribution(generator));
        //(*it).getPose()(1) += (this->R(1) * distribution(generator));
        //(*it).getPose()(2) += (this->R(2) * distribution(generator));
        (*it).getPose()(2) = fmod((float)(*it).getPose()(2)+PI, 2*PI) - PI;
        
        // cout << "Particle " << (*it).getID() << endl;
        //cout << (*it).getPose() << endl;
                
    }
    
    this->last_timestamp = current_timestamp;
    
}


// occupancy grid mapping
void RBPF::mapping(Sensor &sensor, const Eigen::Array3f &pose){
    
    // transform robot pose from world coordinates to map coordinates
    Eigen::Array3f map_pose = Map::world2map(pose);
   
    // transform the sensor's maximum range to map scale
    int map_range = Map::world2map(sensor.getRange());
    
    // iterate over all particles
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        // get a reference to the currently inspected particle's map data
        cv::Mat map_ref = (*it).getMap().getData();
        
        // iterate over all vertical pixels within range of the sensor from robot's current position
        for (int y_px = ((int)map_pose(1) - map_range); y_px <= ((int)(map_pose(1) + map_range)); y_px++) {
            // check that index lies within map boundaries
            if (y_px >= 0 && y_px < (int)(Map::getHeight() / Map::getResolution())) {
                
                // get a pointer to the beginning of the currently inspected row to access all relevant horizontal pixels
                uchar* horizontal_pixel_ptr = map_ref.ptr<uchar>(y_px);
                
                // iterate over all horizontal pixels within range of the sensor from robot's current position
                for (int x_px = ((int)map_pose(0) - map_range); x_px <= ((int)(map_pose(0) + map_range)); x_px++) {
                    // check that index lies within map boundaries
                    if (x_px >= 0 && x_px < (int)(Map::getHeight() / Map::getResolution())) {
                        
                        // get occupancy information from map for currently inspected pixel
                        float occupancy_update = this->inverse_sensor_model(x_px, y_px, map_pose, sensor);
                        
                        // update map with new occupancy information
                        // keep values within range of specified values in case maximum or minimum is reached
                        if (horizontal_pixel_ptr[x_px] >= (Map::getValueMax() - Map::getValueStep())) {
                            horizontal_pixel_ptr[x_px] = Map::getValueMax(); }
                        else if (horizontal_pixel_ptr[x_px] <= Map::getValueStep()) {
                            horizontal_pixel_ptr[x_px] = Map::getValueMin(); }
                        else { horizontal_pixel_ptr[x_px] += occupancy_update; }
                    }
                }
            }
        }
        
        // show map
        (*it).getMap().show();
    }
}


// inverse sensor model
int RBPF::inverse_sensor_model(const int &x_px, const int &y_px, Eigen::Array3f& map_pose, Sensor &sensor){
    
    // occupancy update variable
    int occupancy_update;
    
    // inverse model parameters
    const float alpha = 1.0; // thickness of object in map coordinates
    const float beta = 0.1; // width of a laser beam in map coordinates
    const float max_range = Map::world2map(sensor.getRange());
    
    // get mass center of currently inspected pixel
    const float xc_px = x_px + 0.5;
    const float yc_px = y_px + 0.5;
    
    // get mass center of robot's center pixel
    const float xc_r = (float)map_pose(0) + 0.5;
    const float yc_r = (float)map_pose(1) + 0.5;
    const float heading_r = (float)map_pose(2);
    
    // distance from robot to mass center of currently inspected pixel
    const float pixel_distance = sqrt(pow(xc_px - xc_r, 2) + pow(yc_px - yc_r, 2));
    
    // angle of currently inspected pixel relative to robot's heading
    float pixel_angle = atan2(yc_px - yc_r, xc_px - xc_r) - heading_r;
    // keep angle within range (-PI, PI]
    if (pixel_angle < -PI) {
        pixel_angle = fmod(pixel_angle-PI, 2*PI) + PI; }
    else {
        pixel_angle = fmod(pixel_angle+PI, 2*PI) - PI; }

    // check if inspected pixel lies in sensor's FoV
    if (pixel_angle < -((float)sensor.getFoV()/360.0*PI) || pixel_angle > ((float)sensor.getFoV()/360*PI)) {
        return occupancy_update = 0; }
    else {
    
        // select laser beam with the minimum angle difference to the calculate pixel angle
        vector<float> angle_dif;
        for(int beam_id = 0; beam_id < sensor.getN(); beam_id++) {
            angle_dif.push_back(abs(pixel_angle - sensor.getMeasurements()(beam_id, 0)));
        }
        
        // index of selected measurement
        int beam_id = (int) distance(angle_dif.begin(), min_element(angle_dif.begin(), angle_dif.end()));
    
        // get corresponding range and relative angle of the selected measurement
        int detected_range = Map::world2map(sensor.getMeasurements()(beam_id, 1));
        const float max_detection_range = detected_range + alpha/2.0;
        float beam_angle = sensor.getMeasurements()(beam_id, 0);
        
        // get occupancy value update
        // if angle difference larger than beam width and pixel distance greater than measured range, no update
        if ((abs(pixel_angle-beam_angle) > (beta/2)) || pixel_distance >= min(max_range, max_detection_range)) {
            occupancy_update = 0; }
        // if detected range within sensor range and difference between detected range and pixel distance smaller than object width, occupied pixel detected
        else if (detected_range < max_range && (abs(pixel_distance-detected_range) < alpha/2.0)) {
            occupancy_update = -Map::getValueStep(); }
        // if the distance to the currently inspected pixel is shorter than the detected range, unoccupied pixel detected
        else if (pixel_distance <= detected_range) {
            occupancy_update = Map::getValueStep(); }
        else {
            cout << "Grid occupancy failed" << endl;
            exit(1);
        }
    
        return occupancy_update; }
}


// estimate sensor sweep from particles' maps
void RBPF::sweep_estimate(Sensor &sensor, const Eigen::Array3f &pose){
    
    // iterate over all particles
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        // transform robot pose from world coordinates to map coordinates
        Eigen::Array3f map_pose = Map::world2map((*it).getPose());
        //Eigen::Array3f map_pose = Map::world2map(pose);

        // get mass center of robot's center pixel
        const float xc_r = (float)map_pose(0) + 0.5;
        const float yc_r = (float)map_pose(1) + 0.5;
        const float heading_r = (float)map_pose(2);
        
        // transform the sensor's maximum range to map scale
        int map_range = Map::world2map(sensor.getRange());
        
        (*it).getMeasurementEstimate() = Eigen::MatrixX2f::Ones(sensor.getN(), 2) * sensor.getRange(); // array of measurement values
        (*it).getMeasurementEstimate().col(0) = Eigen::ArrayXf::LinSpaced(sensor.getN(), -PI/180*sensor.getFoV()/2, PI/180*sensor.getFoV()/2);
        
        Eigen::MatrixX2f measurement_ref = sensor.getMeasurements();
        
        // get a reference to the currently inspected particle's map data
        cv::Mat map_ref = (*it).getMap().getData();
        
        // iterate over all vertical pixels within range of the sensor from robot's current position
        for (int y_px = ((int)map_pose(1) - map_range); y_px <= ((int)(map_pose(1) + map_range)); y_px++) {
            // check that index lies within map boundaries
            if (y_px >= 0 && y_px < (int)(Map::getHeight() / Map::getResolution())) {
                
                // get a pointer to the beginning of the currently inspected row to access all relevant horizontal pixels
                uchar* horizontal_pixel_ptr = map_ref.ptr<uchar>(y_px);
                
                // iterate over all horizontal pixels within range of the sensor from robot's current position
                for (int x_px = ((int)map_pose(0) - map_range); x_px <= ((int)(map_pose(0) + map_range)); x_px++) {
                    // check that index lies within map boundaries
                    if (x_px >= 0 && x_px < (int)(Map::getWidth() / Map::getResolution())) {
                    
                        // Update estimated scan only if occupied cell detected
                        if (horizontal_pixel_ptr[x_px] < 120) {
                            
                            // get mass center of currently inspected pixel
                            const float xc_px = x_px + 0.5;
                            const float yc_px = y_px + 0.5;
                            
                            vector<float> pixel_angles;
                            for (float i = -0.5; i <= 0.5; i+=0.5) {
                                for (float j = -0.5; j<= 0.5; j+=0.5) {
                                    
                                    // angle of currently inspected pixel relative to robot's heading
                                    float pixel_angle = atan2((yc_px+j) - yc_r, (xc_px+i) - xc_r) - heading_r;
                                    // keep angle within range (-PI, PI]
                                    if (pixel_angle < -PI) {
                                        pixel_angle = fmod(pixel_angle-PI, 2*PI) + PI; }
                                    else {
                                        pixel_angle = fmod(pixel_angle+PI, 2*PI) - PI; }
                                    
                                    pixel_angles.push_back(pixel_angle);
                                }
                            }
                            
                            float angle_max = *max_element(pixel_angles.begin(), pixel_angles.end());
                            float angle_min = *min_element(pixel_angles.begin(), pixel_angles.end());
                            
                            // distance from robot to mass center of currently inspected pixel
                            const float pixel_distance = sqrt(pow(xc_px - xc_r, 2) + pow(yc_px - yc_r, 2));
                            
                            //find all relevant laser beams and update their estimated distance
                            vector <int> valid_ids;
                            if (angle_min < -PI/2 && angle_max > PI/2) {
                                for (int beam_id = 0; beam_id < sensor.getN(); beam_id++) {
                                    if (measurement_ref(beam_id, 0) <= angle_min && measurement_ref(beam_id, 0) >= angle_max) {
                                        valid_ids.push_back(beam_id);
                                    }
                                }
                            }
                            else {
                                for (int beam_id = 0; beam_id < sensor.getN(); beam_id++) {
                                    if (measurement_ref(beam_id, 0) >= angle_min && measurement_ref(beam_id, 0) <= angle_max) {
                                        valid_ids.push_back(beam_id);
                                    }
                                }
                            }
                            
                            for (vector<int>::iterator beam_it = valid_ids.begin(); beam_it != valid_ids.end(); beam_it++) {
                                if (pixel_distance < map_range) {
                                    (*it).getMeasurementEstimate()((*beam_it), 1) = Map::map2world((int)pixel_distance);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


// polar to cart transform
Eigen::MatrixX2f polar2cart(const Eigen::Array3f& pose, const Eigen::MatrixX2f& measurements_polar){
    
    Eigen::MatrixX2f measurements_cartesian = Eigen::MatrixX2f::Zero(measurements_polar.rows(), measurements_polar.cols());
    
    // rotation matrix for local to global coordinates
    Eigen::Matrix2f R;
    R(0, 0) = cos((float)pose(2));
    R(1, 0) = sin((float)pose(2));
    R(0, 1) = -sin((float)pose(2));
    R(1, 1) = cos((float)pose(2));
    
    for (int beam_id = 0; beam_id < measurements_polar.rows(); beam_id++){
        
        measurements_cartesian(beam_id, 0) = measurements_polar(beam_id, 1) * cos(measurements_polar(beam_id, 0));
        measurements_cartesian(beam_id, 1) = measurements_polar(beam_id, 1) * sin(measurements_polar(beam_id, 0));
    }
    
    Eigen::Vector2f pose2d;
    pose2d(0) = (float)pose(0);
    pose2d(1) = (float)pose(1);
    
    measurements_cartesian = (R * measurements_cartesian.transpose()).transpose();
    measurements_cartesian.rowwise() += pose2d.block<2,1>(0, 0).transpose();
    
    return measurements_cartesian;
}


// scan matching
void RBPF::scan_matching(const Eigen::Array3f &pose, Sensor& sensor){
    
    // transform measurements to cartesian coordinates
    Eigen::MatrixX2f measurements_cartesian = polar2cart(pose, sensor.getMeasurements());
    
    // iterate over all particles
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
    
        // transform measurements to cartesian coordinates
        Eigen::MatrixX2f measurement_estimate_cartesian = polar2cart((*it).getPose(), (*it).getMeasurementEstimate());
        
        Eigen::Array3f pose_dif = this->scan_matcher.ICP(measurement_estimate_cartesian, measurements_cartesian);
        
        (*it).getPose() += pose_dif;
        cout << "Measurement Dif: \n" << measurements_cartesian - measurement_estimate_cartesian << endl;

    }
    
    
}


// weight particles
void RBPF::weight(Sensor &sensor){
    
    Eigen::MatrixX2f measurement_ref = sensor.getMeasurements();
    
    vector<float> weights;
    
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        Eigen::MatrixX2f measurement_estimate = (*it).getMeasurementEstimate();
        
        float p;
        float q = 1;
        for (int beam_id = 0; beam_id < measurement_ref.rows(); beam_id++){
            
            float scan_dif = abs(measurement_ref(beam_id, 1) - measurement_estimate(beam_id, 1));
            p = exp(-0.5*pow((scan_dif/sensor.getQ()(1, 1)), 2) / sqrt(2*PI) * sensor.getQ()(1, 1));
            q *= p;
        }
        
        weights.push_back(q);
        (*it).getWeight() = q;
    }
    
    float sum_of_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (sum_of_weights < 1e-3){
        cout << "Sum of Weights = 0" << endl;
        for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
            (*it).getWeight() = 1.0 / this->getN();
        }
    }
    else {
        for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
            (*it).getWeight() /= sum_of_weights;
        }
    }
}


// resample particles
void RBPF::resample(){
    
    default_random_engine generator;
    uniform_real_distribution<float> distribution(0.0, 1.0);
    
    vector<float> cum_sum;
    vector<Eigen::Array3f> poses;
    float sum = 0;
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        cum_sum.push_back(sum);
        sum += (float)(*it).getWeight();
        poses.push_back((*it).getPose());
    }
    
    float r = distribution(generator) / this->n_particles;
    vector<int> particle_ids;
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        float ref_sum = r + (distance(this->particles.begin(), it) / this->n_particles);
        for (vector<float>::iterator sit = cum_sum.begin(); sit != cum_sum.end(); sit++) {
            if ((*sit) >= ref_sum) {
                int particle_id = (int) distance(cum_sum.begin(), sit);
                particle_ids.push_back(particle_id);
                break;
            }
        }
    }
    
    for (list<Particle>::iterator it = this->particles.begin(); it != this->particles.end(); it++) {
        
        int particle_id = particle_ids[(int)distance(this->particles.begin(), it)];
        (*it).getPose() = poses[particle_id];
        (*it).getWeight() = 1 / this->n_particles;
    }
    
}
