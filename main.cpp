#include <fstream>
#include <iostream>
#include <map>
#include "include/motion_profiler/S_curve_profile.hpp"

int main(){


    using namespace std;

//    BezierSpline spline({0,0}, {1, 1}, {2,0}, {3, 1});
//
//    std::vector<point> pts = spline.generate(100, true);
//
//    for(auto pt: pts){
//        std::cout << pt.x << ", " << pt.y << std::endl;
//    }
    std::ofstream Spos;
    Spos.open("motionProfile/Ramsete/include/SPosition.txt");
    std::ofstream Svel;
    Svel.open("motionProfile/Ramsete/include/Svelocity.txt");
    std::ofstream Sacc;
    Sacc.open("motionProfile/Ramsete/include/Sacceleration.txt");

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
