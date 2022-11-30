#include <fstream>
#include <iostream>
#include <map>
#include "include/motion_profiler/S_curve_profile.hpp"


int main(){

    S_CurveProfile s_profile(30, 9, 6, 13);


    std::ofstream Spos;
    Spos.open("Sposition.txt");
    std::ofstream Svel;
    Svel.open("Svelocity.txt");
    std::ofstream Sacc;
    Sacc.open("Sacceleration.txt");

    std::map<float, float> profile = s_profile.get_displacement_velocity_profile(0.01);

    Spos.close();
    Svel.close();
    Sacc.close();

}
