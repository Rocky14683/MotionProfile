#include <cmath>
#include "../include/motion_profiler/S_curve_profile.hpp"
#include <fstream>
#include <iostream>

namespace math {
float clamp(float n, float lower, float upper) { return std::max(lower, std::min(n, upper)); }
} // namespace math

SigmoidMotionProfile::SigmoidMotionProfile(float target, float maxVelocity, float maxAcceleration,
                                           float maxDeceleration, float jerk)
    : target(target), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), maxDeceleration(maxDeceleration),
      jerk(jerk), ratio(maxAcceleration / maxDeceleration) {
    this->profile.clear();

    this->timeAtJ = this->maxAcceleration / this->jerk;
    this->timeAtA = this->maxVelocity / this->maxAcceleration - this->timeAtJ;

    if (this->timeAtA < 0) { // maxAcceleration is not reachable
        this->timeAtA = 0;
        this->timeAtJ = std::sqrtf(this->maxVelocity / this->jerk);
        this->maxAccelNotReach = true;
        std::cout << "1. maxAcceleration is not reachable" << std::endl;
    }

    this->timeAtV = this->target / (this->jerk * this->timeAtJ * (this->timeAtJ + this->timeAtA)) -
                    (this->ratio + 1) / 2 * (2 * this->timeAtJ + this->timeAtA);

    if (this->timeAtV < 0) { // maxVelocity is not reachable
        this->timeAtV = 0;
        float lambda = std::powf(this->timeAtJ, 2) + (8 * this->target)/ (this->jerk * this->timeAtJ * (this->ratio + 1));
        this->timeAtA = (std::sqrtf(lambda) -3 * this->timeAtJ ) / 2;
        this->maxVelNotReach = true;
        std::cout << "maxVelocity is not reachable" << std::endl;

        if (this->timeAtA < 0) { // maxAcceleration is not reachable
            this->timeAtA = 0;
            this->timeAtJ = std::powf(this->target / (this->jerk * (this->ratio + 1)), 1 / 3);
            this->maxAccelNotReach = true;
            std::cout << "2. maxAcceleration is not reachable" << std::endl;
        }
    }
    float t0, t1, t2, t3, t4, t5, t6, t7;
    this->timeInterval = {t0 = 0.0f,
                          t1 = this->timeAtJ,
                          t2 = t1 + this->timeAtA,
                          t3 = t2 + this->timeAtJ,
                          t4 = t3 + this->timeAtV,
                          t5 = t4 + this->ratio * this->timeAtJ,
                          t6 = t5 + this->ratio * this->timeAtA,
                          t7 = t6 + this->ratio * this->timeAtJ};

    for (float t : this->timeInterval) { std::cout << t << ","; }
    std::cout << std::endl;

    std::cout << "calculated length: "
              << this->jerk * this->timeAtJ * (this->timeAtJ + this->timeAtA) / 2 *
                     ((this->ratio + 1) * (2 * this->timeAtJ + this->timeAtA) + 2 * this->timeAtV);

    this->endPhaseAcceleration[0] = this->endPhaseVelocity[0] = this->endPhasePosition[0] = 0.0f;
};

float SigmoidMotionProfile::getTotalTime() { return this->timeInterval[7]; }

SigmoidMotionProfile::ProfileStatus SigmoidMotionProfile::step(float time) {
    if (time > this->timeInterval[7]) throw std::runtime_error("Inputed Time is Larger Than Profile Time");

    ProfileStatus status;

    auto updateInterval = [&]() -> unsigned int {
        for (int interval = 0; interval < timeInterval.size(); interval++) {
            if (time == math::clamp(time, this->timeInterval[interval], this->timeInterval[interval + 1])) {
                return interval + 1;
            }
        }
    };

    unsigned int currentInterval = updateInterval();

    auto getJerk = [&]() -> float {
        switch (currentInterval) {
            case 2:
            case 4:
            case 6: return 0.0f;
            case 1:
            case 7: return this->jerk;
            case 5:
            case 3: return -this->jerk;
        }
    };

    float sectionTime = time - timeInterval[currentInterval - 1];
    int deceling = currentInterval < 4 ? 1 : -1;
    float currentJerk = deceling == 1 ? getJerk() : getJerk() / (float)std::pow(this->ratio, 2);
    // calculate
    status.acceleration = this->endPhaseAcceleration[currentInterval] =
        this->endPhaseAcceleration[currentInterval - 1] + integral(currentJerk, sectionTime);

    status.velocity = this->endPhaseVelocity[currentInterval] =
        this->endPhaseVelocity[currentInterval - 1] + doubleIntegral(currentJerk, sectionTime) +
        integral(this->endPhaseAcceleration[currentInterval - 1], sectionTime);

    status.position = this->endPhasePosition[currentInterval] =
        this->endPhasePosition[currentInterval - 1] + tripleIntegral(currentJerk, sectionTime) +
        doubleIntegral(this->endPhaseAcceleration[currentInterval - 1], sectionTime) +
        integral(this->endPhaseVelocity[currentInterval - 1], sectionTime);

    if ((this->maxAccelNotReach && (currentInterval == 1 || currentInterval == 5)) ||
        (this->maxVelNotReach && currentInterval == 3)) {
        this->endPhaseAcceleration[currentInterval + 1] = endPhaseAcceleration[currentInterval];
        this->endPhaseVelocity[currentInterval + 1] = endPhaseVelocity[currentInterval];
        this->endPhasePosition[currentInterval + 1] = endPhasePosition[currentInterval];
    }

    std::cout << time << ": " << status.position << ", " << status.velocity << ", " << status.acceleration << std::endl;
    std::cout << currentInterval << std::endl;
    return status;
}

std::map<float, SigmoidMotionProfile::ProfileStatus> SigmoidMotionProfile::getProfile(float dt) {
    dt = fmin(dt, 0.005);
    for (float t = 0; t < this->getTotalTime(); t += dt) { this->profile[t] = this->step(t); }
    return this->profile;
}
