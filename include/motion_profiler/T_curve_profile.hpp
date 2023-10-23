#pragma once
#include <map>

class MotionProfile {

protected:
    float motion_distance;
    float motion_velocity_max;
    float motion_acceleration;
    float motion_time_full;

private:
    float motion_time_sliding;
    float motion_time_speeding;
    struct position_status{
        float displacement, velocity;
    };
    std::map<float, MotionProfile::position_status> profile;

public:
    MotionProfile(float distance, float velocity_max, float acceleration);
    float get_distance(float time);
    float get_velocity(float time);
    float get_time();
    std::map<float, MotionProfile::position_status> get_profile(float delta_t);
};


