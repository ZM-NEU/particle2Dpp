#include <iostream>
#include <vector>

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
    float* ranges;
} logEntry;

int import_logs(const char *logName, vector<logEntry> & logBook);