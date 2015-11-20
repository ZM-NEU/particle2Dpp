#include "includes/logBook.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>

#define LIDAR_START 7
#define LIDAR_END 187

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
        logData.logType = (tokens.front()[0] == 'O') ? ODOM : LIDAR;
        logData.robotPose.x = stod(tokens.at(1).c_str())/10;
        logData.robotPose.y = stod(tokens.at(2).c_str())/10;
        logData.robotPose.theta = stod(tokens.at(3).c_str());
        logData.ts = stod(tokens.back().c_str());
        if(logData.logType == LIDAR) {
            logData.lidarPose.x = stod(tokens.at(4).c_str())/10;
            logData.lidarPose.y = stod(tokens.at(5).c_str())/10;
            logData.lidarPose.theta = stod(tokens.at(6).c_str());
            for(int i = LIDAR_START; i < LIDAR_END; i++)
            {
                logData.ranges[i-LIDAR_START] = stod(tokens.at(i).c_str())/10;
            }
        }
        logBook.push_back(logData);
//         fprintf(stderr, "%c %f %f %f\n", logBook.back().logType, logBook.back().robotPose.x, logBook.back().robotPose.y, logBook.back().ts);
    }
    log.close();
    return 1;
}