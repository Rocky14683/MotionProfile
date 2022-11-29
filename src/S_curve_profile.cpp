#include <cmath>
#include "../include/motion_profiler/S_curve_profile.hpp"
#include <fstream>
#include <ostream>


S_CurveProfile::S_CurveProfile(float distance, float velocity_max, float acceleration, float jerk) : MotionProfile::MotionProfile(distance, velocity_max, acceleration){
    this->profile.clear();
    this->motion_jerk = jerk;
    this->motion_time_accelerate = this->motion_acceleration / this->motion_jerk;
    float t2 = this->motion_velocity_max / this->motion_acceleration;
    float t4 = this->motion_distance / this->motion_velocity_max;
    float t6 = t4 + t2;
    std::array<float, 8> time_seq = {0.0f, motion_time_accelerate, t2, t2 + this->motion_time_accelerate, t4, t4 + this->motion_time_accelerate, t6, t6 + this->motion_time_accelerate};
    this->time_array = time_seq;
    this->motion_time_full = time_array[7];
}


inline void S_CurveProfile::find_time_section(float time){
    if(time < this->time_array[0]) this->time_section_pointer = 0;
    else if(time < this->time_array[1]) this->time_section_pointer = 1;
    else if(time < this->time_array[2]) this->time_section_pointer = 2;
    else if(time < this->time_array[3]) this->time_section_pointer = 3;
    else if(time < this->time_array[4]) this->time_section_pointer = 4;
    else if(time < this->time_array[5]) this->time_section_pointer = 5;
    else if(time < this->time_array[6]) this->time_section_pointer = 6;
    else if(time < this->time_array[7]) this->time_section_pointer = 7;
    else this->time_section_pointer = 8;
}

float S_CurveProfile::get_jerk(float time){
    if(this->time_section_pointer == 1 || this->time_section_pointer == 7) return this->motion_jerk;
    else if(this->time_section_pointer == 3 || this->time_section_pointer == 5) return -this->motion_jerk;
    return 0;
}

float S_CurveProfile::get_acceleration(float time){
    int sgn = this->time_section_pointer < 4 ? 1 : -1;
    float decel = this->motion_jerk * (time - time_array[this->time_section_pointer - 1]);
    if(this->time_section_pointer == 1) return this->motion_jerk * time;
    else if(this->time_section_pointer == 5) return decel;
    else if(this->time_section_pointer == 2 || this->time_section_pointer == 6) return sgn * this->motion_acceleration;
    else if(this->time_section_pointer == 3 || this->time_section_pointer == 7) return sgn * (this->motion_acceleration - decel);
    return 0.0f;
}

float S_CurveProfile::get_velocity(float time){
    float acceleration_time_base = time - 0.5 * this->motion_time_accelerate;
    float acceleration_time      = (this->time_section_pointer < 4) ? acceleration_time_base : (time_array[2] + time_array[5] - acceleration_time_base);
    float velocity_offset        = 0.5 * this->motion_jerk * std::pow(time - time_array[time_section_pointer - 1], 2);
    float velocity_sum = 0.0f;
    if ((time_section_pointer % 4) == 2 || (time_section_pointer % 4) == 3) velocity_sum += this->motion_acceleration * acceleration_time;
    if (time_section_pointer       == 4 || time_section_pointer       == 5) velocity_sum += this->motion_velocity_max;
    if (time_section_pointer       == 1 || time_section_pointer       == 7) velocity_sum += velocity_offset;
    if (time_section_pointer       == 3 || time_section_pointer       == 5) velocity_sum -= velocity_offset;
    return velocity_sum;
}


float S_CurveProfile::get_continuous_distance(float time){
    float velocity_sum = 0;
    float delta_t;
    for(unsigned int t = 0; t < this->time_section_pointer; t++){
        delta_t = this->time_array[this->time_section_pointer + 1] - this->time_array[this->time_section_pointer];
        if(this->time_section_pointer % 2 == 0) velocity_sum += this->get_velocity(time) * delta_t + (this->get_acceleration(time) * std::pow(delta_t, 2)) / 2 + (this->motion_jerk * std::pow(delta_t, 3)) / 6;
        else if(this->time_section_pointer % 4 == 1) velocity_sum += this->get_velocity(time) * delta_t + (this->motion_acceleration * std::pow(delta_t, 2)) / 2;
        else if(this->time_section_pointer == 3) velocity_sum += this->motion_velocity_max * time;
    }
    return velocity_sum;
}

float S_CurveProfile::get_discrete_distance(float time, float delta_t){
    float current_jerk = this->get_jerk(time);
    this->displacement += this->get_velocity(time) * delta_t;

    return this->displacement;
}


std::map<float, S_CurveProfile::position_status> S_CurveProfile::get_profile(float delta_t){
    if(profile.size() > 0) return profile;
    for(float t = 0; t < this->get_time(); t+= delta_t){
        this->find_time_section(t);
        float dis = this->get_discrete_distance(t, delta_t), vel = this->get_velocity(t), acc = this->get_acceleration(t);
        this->profile[dis] = (S_CurveProfile::position_status){.displacement = dis, .velocity = vel, .acceleration = acc};
    }
    return profile;
}


