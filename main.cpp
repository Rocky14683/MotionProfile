#include <fstream>
#include <iostream>
#include <map>
#include "include/motion_profiler/S_curve_profile.hpp"


int main(){




    std::ofstream Spos;
    Spos.open("Sposition.txt");
    std::ofstream Svel;
    Svel.open("Svelocity.txt");
    std::ofstream Sacc;
    Sacc.open("Sacceleration.txt");

    std::map<float, SigmoidMotionProfile::ProfileStatus>profile = SigmoidMotionProfile(30, 9, 6, 13).getProfile();
    std::cout << profile.size();
    for(auto& [time, status] : profile){
        Spos << status.position << std::endl;
        std::cout << status.position << ", " << status.velocity << ", " << status.acceleration << std::endl;
        Svel << status.velocity << std::endl;
        Sacc << status.acceleration << std::endl;
    }

    Spos.close();
    Svel.close();
    Sacc.close();

}
