#include <iostream>
#include <vector>

#define NUM_RANGES 180

using namespace std;

enum log_type
{
    ODOM = 0,
    LIDAR = 1 
};

typedef struct {
    float x, y, theta;
} pose2D;

typedef struct {
    log_type logType;
    pose2D robotPose;
    pose2D lidarPose;
    double ts;
    float ranges[NUM_RANGES];
} logEntry;

int import_logs(const char *logName, vector<logEntry> & logBook);