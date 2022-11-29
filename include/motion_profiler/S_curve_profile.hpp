#include "../motion_profiler/T_curve_profile.hpp"
#include <map>
#include <array>

class S_CurveProfile : public MotionProfile{
public:
    struct position_status{
        float displacement, velocity, acceleration;
    };

private:
    float motion_jerk;
    std::array<float, 8> time_array;
    float motion_time_accelerate;
    std::map<float, position_status> profile;
    unsigned int time_section_pointer;

    float displacement;
    float last_vel;

    inline void find_time_section(float time);
    float get_jerk(float time);

public:
    S_CurveProfile(float distance, float velocity_max, float acceleration, float jerk);
    float get_continuous_distance(float time);
    float get_discrete_distance(float time, float delta_t);
    float get_velocity(float time);
    float get_acceleration(float time);
    std::map<float, position_status> get_profile(float delta_t);
};



