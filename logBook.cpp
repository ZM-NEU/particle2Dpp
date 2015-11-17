#include "includes/logBook.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>

#define NUM_RANGES 180

using namespace std;

int import_logs(const char *logName, vector<logEntry> & logBook) {
    ifstream log(logName);
    if(!log.is_open()) {
	fprintf(stderr, "ERROR OPENING FILE: %s\n", logName);
	return -1;
    }
    string logLine;
    logBook.clear();
    while(getline(log, logLine)) {
	logEntry logData;
	char debugType;
	istringstream stringin(logLine);
	
	string buf;
	vector<string> tokens;
	while(stringin >> buf) {
	    tokens.push_back(buf);
	}
	
	for(int i = 0; i < tokens.size(); i++)
	{
	    if(i == 0)
	    {
		char type = tokens.at(0)[0];
		if(type == 'O')
		    fprintf(stderr, "ODOM\t");
	    }
	    if(i == 1)
	    {
		float x_test = strtof(tokens.at(i).c_str(), NULL);
		fprintf(stderr, "x is %f\t", x_test);
	    }
	}
	float ts = strtof(tokens.back().c_str(), NULL);
	fprintf(stderr, "ts is %f\n", ts);
	
	
	
// 	if(logLine[0] == 'O') {
// 	    logData.logType = ODOM;
// 	    stringin >> debugType;
// 	    stringin >> logData.robotPose.x;
// 	    stringin >> logData.robotPose.y;
// 	    stringin >> logData.robotPose.theta;
// 	    stringin >> logData.ts;
// 	}
// 	else if(logLine[0] == 'L') {
// 	    logData.logType = LIDAR;
// 	    stringin >> debugType;
// 	    stringin >> logData.robotPose.x;
// 	    stringin >> logData.robotPose.y;
// 	    stringin >> logData.robotPose.theta;
// 	    stringin >> logData.lidarPose.x;
// 	    stringin >> logData.lidarPose.y;
// 	    stringin >> logData.lidarPose.theta;
// 	    fprintf(stderr, "ERROR 3 %f\n",logData.lidarPose.theta);
// 	    for(unsigned int i = 0; i < NUM_RANGES; i++) {
// 		fprintf(stderr, "ERROR range number %i\n",i);
// 		stringin >> logData.ranges[i];
// 		fprintf(stderr, "ERROR range value %f\n",logData.ranges[i]);
// 	    }
// 	    stringin >> logData.ts;
// 	    fprintf(stderr, "ERROR 3 TS_END\n");
// 	}
// 	else {
// 	    fprintf(stderr, "BAD LOG FILE: %s Line: %lu\n", logName, logBook.size());
// 	    return -1;
// 	}
// 	fprintf(stderr, "ERROR 4\n");
// 	logBook.push_back(logData);
// 	fprintf(stderr, "ERROR 5\n");
// 	cout << logBook.back().logType << " " << logBook.back().robotPose.x << " ";
// 	cout << logBook.back().robotPose.y << " " << logBook.back().ts << endl;
// 	fprintf(stderr, "ERROR 6\n");
    }
    log.close();
    return 1;
}