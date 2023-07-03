#include <fstream>
#include <iostream>
#include <map>
#include "include/motion_profiler/S_curve_profile.hpp"


int main(){


    using namespace std;
    std::ofstream Spos;
    Spos.open("/Users/rockychen/Desktop/motionProfile/Ramsete/include/SPosition.txt");
    std::ofstream Svel;
    Svel.open("/Users/rockychen/Desktop/motionProfile/Ramsete/include/Svelocity.txt");
    std::ofstream Sacc;
    Sacc.open("/Users/rockychen/Desktop/motionProfile/Ramsete/include/Sacceleration.txt");

    std::map<float, SigmoidMotionProfile::ProfileStatus>profile = SigmoidMotionProfile(20, 10, 13, 8,20).getProfile(0.01);
//    std::cout << profile.size();
    for(auto& [time, status] : profile){
        Spos << status.position << std::endl;
        Svel << status.velocity << std::endl;
        Sacc << status.acceleration << std::endl;
    }

    Spos.close();
    Svel.close();
    Sacc.close();

}
