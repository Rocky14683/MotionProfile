#include <fstream>
#include <map>
#include "include/motion_profiler/S_curve_profile.hpp"


int main(){
    using namespace std;
    S_CurveProfile s_profile(15, 6, 6, 13);

    ofstream Spos;
    Spos.open("Sposition.txt");
    ofstream Svel;
    Svel.open("Svelocity.txt");
    ofstream Sacc;
    Sacc.open("Sacceleration.txt");

    std::map<float, S_CurveProfile::position_status> profile = s_profile.get_profile(0.01);

    for(float t = 0; t < s_profile.get_time(); t += 0.01){
        Spos << s_profile.get_discrete_distance(t, 0.01) << endl;
        Svel << s_profile.get_velocity(t) << endl;
        Sacc << s_profile.get_acceleration(t) << endl;
    }
    Spos.close();
    Svel.close();
    Sacc.close();
}
