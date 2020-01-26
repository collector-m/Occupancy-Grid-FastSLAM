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

class Map{
    
    public:
        // constructor and destructor
        Map();
        ~Map(){};
    
        // print summary of map
        static void summary();
    
        // display map using cv::imshow
        cv::Mat draw();
    
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
    
        // non-static getter functions
        cv::Mat& getData(){ return this->data; };
    
        // coordinate transform
        static Eigen::Array3f world2map(const Eigen::Array3f &world_pose);
    
        // scale variable from world coordinates to map coordinates
        static int world2map(const float &world_variable);
        static float map2world(const int &map_variable);
    
    private:
        // static variables map parameters
        const static float width; // width of the map in m
        const static float height; // height of the map in m
        const static float x_min; // min x coordinate of map in m
        const static float x_max; // max x coordinate of map in m
        const static float y_min; // min y coordinate of map in m
        const static float y_max; // max y coordinate of map in m
        const static float resolution; // resolution of the map in m/px
    
        // static variables grid mapping parameters
        const static int occupancy_value_max;
        const static int occupancy_value_min;
        const static int occupancy_value_step;
        const static int occupancy_value_init;
    
        // map data
        cv::Mat data;
    
};

#endif /* Map_h */
