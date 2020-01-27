//
//  Map.h
//  FastSLAM
//
//  Created by Mats Steinweg on 16.08.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef Map_h
#define Map_h

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;


class Map{
    
    public:
        // constructor and destructor
        Map();
        ~Map(){};
    
        // print summary of map
        static void summary();
    
        // display map using cv::imshow
        void draw();
    
        // static getter functions
        static const float& getWidth(){ return Map::width; };
        static const float& getHeight(){ return Map::height; };
        static const float& getResolution(){ return Map::resolution; }
        static const float& getXMin(){ return Map::x_min; };
        static const float& getXMax(){ return Map::x_max; };
        static const float& getYMin(){ return Map::y_min; };
        static const float& getYMax(){ return Map::y_max; };
        static const int& getValueMax(){ return Map::occupancy_value_max; };
        static const int& getValueMin(){ return Map::occupancy_value_min; };
        static const int& getValueStep(){ return Map::occupancy_value_step; };
        static const int& getThreshold(){ return Map::occupancy_threshold; };
    
        // static setter functions
        static void setParameters(float x_min, float x_max, float y_min, float y_max, float map_resolution, int occupancy_value_min, int occupancy_value_max, int occpuancy_value_step, int occupancy_threshold, int simulation_mode, string data_dir);
    
        // non-static getter functions
        cv::Mat& getData(){ return this->data; };
    
        // coordinate transform
        static Eigen::Array3f world2map(const Eigen::Array3f &world_pose);
    
        // scale variable from world coordinates to map coordinates
        static int world2map(const float &world_variable);
        static float map2world(const int &map_variable);
    
    private:
    
        // static variable map type
        static int map_type;
        static string gt_map_file_path;
    
        // static variables map parameters
        static float width; // width of the map in m
        static float height; // height of the map in m
        static float x_min; // min x coordinate of map in m
        static float x_max; // max x coordinate of map in m
        static float y_min; // min y coordinate of map in m
        static float y_max; // max y coordinate of map in m
        static float resolution; // resolution of the map in m/px
    
        // static variables grid mapping parameters
        static int occupancy_value_max;
        static int occupancy_value_min;
        static int occupancy_value_step;
        static int occupancy_threshold;
    
        // map data
        cv::Mat data;
    
};

#endif /* Map_h */
