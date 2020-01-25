//
//  WheelEncoder.h
//  FastSLAM
//
//  Created by Mats Steinweg on 09.09.19.
//  Copyright © 2019 Mats Steinweg. All rights reserved.
//

#ifndef WheelEncoder_h
#define WheelEncoder_h

#include <Eigen/Dense>

class WheelEncoder {
    
    public:
        WheelEncoder();
        ~WheelEncoder(){};
        void encode_movement(float v, float omega, const float& delta_t);
        Eigen::Vector2f getOdometry(float current_timestamp);
    
    private:
        int E_T; // number of ticks per wheel rotation
        float B; // distance between wheels in m
        float R_L; // radius of the left wheel
        float R_R; // radius of the right wheel
        Eigen::Vector2f noise; // standard deviation of the measurement noise
        float last_timestamp; // last update of the encoder ticks
        int ticks_left; // accumulated ticks of left wheel
        int ticks_right; // accumulated ticks of right wheel
        int ticks_left_prev; // accumulated ticks of left wheel at last timestamp
        int ticks_right_prev; // accumulated ticks of right wheel at last timestamp

    
};

#endif /* WheelEncoder_h */
