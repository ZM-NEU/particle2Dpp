#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include "includes/bee-map.h"
#include "includes/map.h"

using namespace std;

int main(int argc, char **argv)
{
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

	int nParticles = 2000;
// 	map1.init_particles(nParticles);
// 	map1.run(logBook1);
	map2.init_particles(nParticles);
	map2.run(logBook2);

    return 0;
}
