#include <map>
#include <array>
#include <vector>

/*typedef struct{
    std::optional<float> maxVelocity;
    std::optional<float> maxAcceleration;
    std::optional<float> maxDeceleration;
}ProfileConstraints;*/

class SigmoidMotionProfile {
public:
    struct ProfileStatus{
        float position, velocity, acceleration;
    };

private:
    static float integral(float rhs, float time){
        return rhs * time;
    }
    static float doubleIntegral(float rhs, float time){
        return rhs * time * time / 2;
    }
    static float tripleIntegral(float rhs, float time){
        return rhs * time * time * time / 6;
    }


    float target;
    float maxVelocity, maxAcceleration, maxDeceleration;
    float jerk;

    float currentPosition;
    float currentVelocity;
    float currentAcceleration;
    float currentJerk;

    float timeToMaxAccel, timeToMaxDecel;
    std::map<float, ProfileStatus> profile;

//    std::map<std::pair<float, float>, ProfileConstraints> constraints;

    std::vector<float> timeInterval;


    unsigned int currentInterval;

    void updateInterval(float time);
    float getJerk(float time);
    float getDisplacement(float time);
    float getVelocity(float time);
    float getAcceleration(float time);
public:

    SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration, float maxDeceleration, float jerk);
    SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration, float jerk);
//    SigmoidMotionProfile setConstraints(float startPos, float endPos, ProfileConstraints constraints);
    float getTotalTime();
    std::map<float, ProfileStatus> getProfile(float dt = 0.01);
};



