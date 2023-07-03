#include <map>
#include <array>
#include <vector>
#include <cmath>

/*typedef struct{
    std::optional<float> maxVelocity;
    std::optional<float> maxAcceleration;
    std::optional<float> maxDeceleration;
}ProfileConstraints;*/

class SigmoidMotionProfile {
    public:
        struct ProfileStatus {
                float position, velocity, acceleration;
        };
    private:
        constexpr static float integral(float input, float time) { return input * time; }

        constexpr static float doubleIntegral(float input, float time) { return input * std::pow(time, 2) / 2; }

        constexpr static float tripleIntegral(float input, float time) { return input * std::pow(time, 3) / 6; }

        float target;
        float maxVelocity, maxAcceleration, maxDeceleration;
        float jerk;
        float ratio;
        bool maxAccelNotReach = false;
        bool maxVelNotReach = false;

        float timeAtJ;
        float timeAtA;
        float timeAtV;
        std::array<float, 8> timeInterval;

        std::array<float, 8> endPhaseAcceleration;
        std::array<float, 8> endPhaseVelocity;
        std::array<float, 8> endPhasePosition;

        std::map<float, SigmoidMotionProfile::ProfileStatus> profile;
        ProfileStatus step(float time);
        float getJerk(unsigned int currentInterval);
//        ProfileStatus stepByDisance(float distance);
    public:
        SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration, float maxDeceleration, float jerk);
        ~SigmoidMotionProfile() = default;

        //    SigmoidMotionProfile* setConstraints(float startPos, float endPos, ProfileConstraints constraints);
        float getTotalTime();

        std::map<float, SigmoidMotionProfile::ProfileStatus> getProfile(float dt = 0.01);
};
