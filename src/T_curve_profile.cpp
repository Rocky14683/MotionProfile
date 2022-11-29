#include <cmath>
#include "../include/motion_profiler/T_curve_profile.hpp"
#include <fstream>

/**
 * Construct a new Motion Profile object
 *
 * @param distance The total distance of the path
 * @param velocity_max The maximum velocity during the motion
 * @param acceleration The acceleration of the motion
 */
MotionProfile::MotionProfile(float distance, float velocity_max, float acceleration) {
    // constants
    this->motion_distance = distance;
    this->motion_acceleration = acceleration;
    // calculates the reachable maximum velocity
    float velocity_max_actual = std::fmin(std::sqrt(acceleration * distance), velocity_max);
    this->motion_velocity_max = velocity_max_actual;
    // calculates the time of motion
    float speeding_time = velocity_max_actual / acceleration; // for either accelerate/decelerate
    float speeding_distance = velocity_max_actual * speeding_time;
    float sliding_distance = distance - speeding_distance;
    float sliding_time = sliding_distance / velocity_max_actual;
    this->motion_time_speeding = speeding_time;
    this->motion_time_sliding = sliding_time;
    this->motion_time_full = 2 * speeding_time + sliding_time;
    this->profile.clear();
}

/**
 * Calculates the instantaneous distance at time
 *
 * @param time The time since the start of the motion
 * @return instantaneous distance at time
 */
float MotionProfile::get_distance(float time) {
    float distance_net = 0.0f;
    // accelerate
    float accelerate_time = std::fmin(time, this->motion_time_speeding);
    distance_net += 0.5f * this->motion_acceleration * std::pow(accelerate_time, 2);
    // slide
    float slide_time = std::fmin(time - this->motion_time_speeding, this->motion_time_sliding);
    if (slide_time > 0) distance_net += this->motion_velocity_max * slide_time;
    // decelerate
    float decelerate_time = time - this->motion_time_speeding - this->motion_time_sliding;
    if (decelerate_time > 0) distance_net += this->motion_velocity_max * decelerate_time - 0.5f * this->motion_acceleration * std::pow(decelerate_time, 2);
    return distance_net;
}

/**
 * Calculates the instantaneous velocity at time
 *
 * @param time The time since the start of the motion
 * @return Instantaneous velocity
 */
float MotionProfile::get_velocity(float time) {
    // accelerate
    if (time < this->motion_time_speeding) return this->motion_acceleration * time;
    // slide
    if (time < this->motion_time_speeding + this->motion_time_sliding) return this->motion_velocity_max;
    // decelerate
    return this->motion_velocity_max - this->motion_acceleration * (time - (this->motion_time_speeding + this->motion_time_sliding));
}

/**
 * Calculates the total time of the motion
 *
 * @return Total time of the motion
 */
float MotionProfile::get_time() {
    return this->motion_time_full;
}

/**
 * Sort profile into a map
 *
 * @return displacement to velocity profile
 */
std::map<float, MotionProfile::position_status> MotionProfile::get_profile(float delta_t){
    if(this->profile.size() > 0)return this->profile;
    for(float t = 0; t < this->motion_time_full ; t+= delta_t){
        this->profile[this->get_distance(t)] = (MotionProfile::position_status){.displacement = this->get_distance(t), .velocity = this->get_velocity(t)};
    }
    return this->profile;
}
