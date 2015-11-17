#include <iostream>
#include <vector>
#include <string>
#include "includes/bee-map.h"
#include "includes/logBook.h"
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
    
    printf("Resolution: %i\n",map.resolution);
    printf("SizeX: %i\n",map.size_x);
    printf("SizeY: %i\n",map.size_y);
  
    
    // CLASS LOG READER
    return 0;
}


// MAP FUNCTIONS
//	Pick a random point on the map given a distribution
//	Find the distance to wall given (x,y,theta)

// LOG FILE FUNCTIONS
//	Read in the data

// PARTICLE FUNCTIONS
//	update particle position based off odometry
//	update particle belief based off sensor data
//	add random particles to track kidnapping scenario
