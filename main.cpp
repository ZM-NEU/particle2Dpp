#include <iostream>
#include <vector>
#include <string>
#include "includes/bee-map.h"
#include "includes/map.h"

using namespace std;

int main(int argc, char **argv) {
    map_type map;
    string map_name = "../data/map/wean.dat";
    string log1_name = "../data/log/robotdata1.log";
    string log2_name = "../data/log/ascii-robotdata5.log";

    vector<logEntry> logBook1;
    vector<logEntry> logBook2;

    import_logs(log1_name.c_str(), logBook1);
    import_logs(log2_name.c_str(), logBook2);

    printf("Number of entries in Log1: %lu\n",logBook1.size());
    printf("Number of entries in Log2: %lu\n",logBook2.size());

    read_beesoft_map(map_name.c_str(), &map);
    Map map1 = Map();
    Map map2 = Map();
    map1.init_map(map);
    map2.init_map(map);

	int nParticles = 3500;
	map1.init_particles(nParticles);
	map1.run(logBook1);

    return 0;
}

//  TODO
//  MAP FUNCTIONS [BASIC]
//	Pick a random point on the map given a distribution (resample)
//	Find the true distance to wall given (x,y,theta)
//  Move the particles with some uncertainty by the motion
//  Update particles' belief based off sensor data with some uncertainty
//
//  MAP FUNCTIONS [EXTRA]
//	Inject random particles to track kidnapping scenario
//
//  Take from Probabilistic Robotics pdf
//  4.2 Particle Filter - pg77-84
//  5.4 Odometry MotionModel - pg107-113
//  6 Measurements - pg121-154
//  8.3 MonteCarlo Localization - 200-209 [211-216]
//  NOTE: I pulled out relevant information and put it in particleFilterNotes pdf
