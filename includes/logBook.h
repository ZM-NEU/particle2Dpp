#ifndef LOGBOOK_H
#define LOGBOOK_H

#include <iostream>
#include <vector>
#define NUM_RANGES 180

using namespace std;

enum log_type
{
	INVALID = -1,
    ODOM = 0,
    LIDAR = 1
};

typedef struct pose2D {
    float x, y, theta;
} pose2D;

typedef struct logEntry {
    log_type logType;
    pose2D robotPose;
    pose2D lidarPose;
    double ts;
    float ranges[NUM_RANGES];
} logEntry;

int import_logs(const char *logName, vector<logEntry> & logBook);
#endif // LOGBOOK_H