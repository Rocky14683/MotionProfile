#include <cmath>
#include "../include/motion_profiler/S_curve_profile.hpp"
#include <fstream>
#include <iostream>


namespace math{
    float clamp(float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
    }
}

SigmoidMotionProfile::SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration,
                                           float maxDeceleration, float jerk) {
    this->target = target;
    this->maxVelocity = maxVelocity;
    this->maxAcceleration = maxAcceleration;
    this->maxDeceleration = maxDeceleration;
    this->jerk = jerk;
    this->profile.clear();

    this->timeToMaxAccel = this->maxAcceleration / this->jerk;
    this->timeToMaxDecel = this->maxDeceleration / this->jerk;

    bool shortProfile =
            this->maxVelocity * (this->timeToMaxAccel + this->maxVelocity / this->maxAcceleration) > this->target;
    this->maxVelocity = shortProfile ?
                        (float) (this->maxAcceleration *
                                 (sqrt(this->target / this->maxAcceleration - 0.75 * pow(this->timeToMaxAccel, 2) -
                                       0.5 * this->timeToMaxAccel)))
                                     : this->maxVelocity;

    float t1, t2, t3, t4, t5, t6, t7;
    this->timeInterval = {0.0f,
            t1 = this->timeToMaxAccel,
            t2 = this->maxVelocity / this->maxAcceleration,
            t3 = t2 + this->timeToMaxAccel,
            t4 = shortProfile ? t3 : target / this->maxVelocity,
            t5 = t4 + timeToMaxDecel,
            t6 = t4 + (this->maxVelocity / this->maxDeceleration),
            t7 = t6 + this->timeToMaxDecel};
    for(auto kv : timeInterval){
        std::cout << kv << ", ";
    }
    std::cout << std::endl << &timeInterval[7] << std::endl;
}

SigmoidMotionProfile::SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration, float jerk) {
    SigmoidMotionProfile(target, maxVelocity, maxAcceleration, maxAcceleration, jerk);
}

/*SigmoidMotionProfile SigmoidMotionProfile::setConstraints(float startPos, float endPos, ProfileConstraints constraint) {
    auto setValueIfNotSet = [](auto &target, const auto &source, const std::string &errorMessage) {
        if (source.has_value() && !target.has_value()) {
            target = source;
        } else {
            throw std::runtime_error(errorMessage);
        }
    };

    const auto &key = this->constraints.find({startPos, endPos});
    if (key != this->constraints.end()) {
        setValueIfNotSet(this->constraints[{startPos, endPos}].maxVelocity, constraint.maxVelocity,
                         "Max Velocity has been set once.");
        setValueIfNotSet(this->constraints[{startPos, endPos}].maxAcceleration, constraint.maxAcceleration,
                         "Max Acceleration has been set once.");
        setValueIfNotSet(this->constraints[{startPos, endPos}].maxDeceleration, constraint.maxDeceleration,
                         "Max Deceleration has been set once.");
    }

    if (startPos < 0 || startPos > this->target || endPos < 0 || endPos > this->target) {
        throw std::runtime_error("Limit out of range");
    }

    constraints[{startPos, endPos}] = constraint;

    return *this;
}*/

void SigmoidMotionProfile::updateInterval(float time) {
    for (int interval = 1; interval <= 8; interval++) {
        if (time == math::clamp(time, this->timeInterval[interval - 1], this->timeInterval[interval]))
            this->currentInterval = interval;
        return;
    }
}

float SigmoidMotionProfile::getJerk(float time) {
    switch (this->currentInterval) {
        case 1:
        case 7:
            return this->currentJerk = this->jerk;
        case 0:
        case 2:
        case 4:
        case 6:
            return this->currentJerk = 0;
        case 3:
        case 5:
            return this->currentJerk = -this->jerk;
    }
    return 0;
}

float SigmoidMotionProfile::getAcceleration(float time) {
    float sectionTime = time - this->timeInterval[this->currentInterval - 1];
    switch (this->currentInterval) {
        case 0:
        case 4:
            return this->currentAcceleration = 0;
        case 2:
        case 6:
            return this->currentAcceleration;
        case 1:
        case 7:
            return this->currentAcceleration = this->currentJerk * sectionTime;
        case 3:
        case 5:
            return this->currentAcceleration = -this->currentJerk * sectionTime;
    }
    return 0;
}

float SigmoidMotionProfile::getVelocity(float time) {
    float sectionTime = time - this->timeInterval[this->currentInterval - 1];
    float jerkIntegral = SigmoidMotionProfile::doubleIntegral(this->currentJerk, sectionTime);
    float accelIntegral = SigmoidMotionProfile::integral(this->currentAcceleration, sectionTime);
    switch (this->currentInterval) {
        case 0:
            return this->currentVelocity = 0;
        case 1:
        case 5:
            return this->currentVelocity = jerkIntegral;
        case 2:
        case 6:
            return this->currentVelocity += accelIntegral;
        case 3:
        case 7:
            return this->currentVelocity += accelIntegral + jerkIntegral;
        case 4:
            return this->currentVelocity;
    }
    return 0;
}

float SigmoidMotionProfile::getDisplacement(float time) {
    float sectionTime = time - this->timeInterval[this->currentInterval - 1];
    float jerkIntegral = SigmoidMotionProfile::tripleIntegral(this->currentJerk, sectionTime);
    float accelIntegral = SigmoidMotionProfile::doubleIntegral(this->currentAcceleration, sectionTime);
    float velIntegral = SigmoidMotionProfile::integral(this->currentVelocity, sectionTime);
    switch (this->currentInterval) {
        case 0:
            return this->currentPosition = 0;
        case 1:
            return this->currentPosition = jerkIntegral;
        case 2:
        case 6:
            return this->currentPosition += accelIntegral + jerkIntegral;
        case 3:
        case 7:
            return this->currentPosition += velIntegral + accelIntegral + jerkIntegral;
        case 4:
            return this->currentPosition += velIntegral;
        case 5:
            return this->currentPosition += velIntegral + accelIntegral;

    }
    return 0;
}

float SigmoidMotionProfile::getTotalTime(){
    std::cout<<&this->timeInterval[7];
    return this->timeInterval[7];
}

std::map<float, SigmoidMotionProfile::ProfileStatus> SigmoidMotionProfile::getProfile(float dt) {
    for(auto kv : timeInterval){
        std::cout << kv << ", ";
    }
    std::cout<<std::endl <<&this->timeInterval[7] << " : " << this->timeInterval[7];
    for (float t = 0; t < this->timeInterval[7]; t += dt) {
        this->updateInterval(t);
        this->getJerk(t);
        this->profile[t] = (SigmoidMotionProfile::ProfileStatus){.position = this->getDisplacement(t), .velocity = this->getVelocity(t), .acceleration = this->getAcceleration(t)};
    }
    return this->profile;
}







