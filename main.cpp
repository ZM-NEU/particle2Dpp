#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include "includes/bee-map.h"
#include "includes/map.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

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
    
    cv::Mat _map_image;
    _map_image = cv::Mat::zeros( map.size_x, map.size_y, CV_8UC1 );
    for(int i = 0; i < _map_image.rows; i++)
    {
        for(int j = 0; j < _map_image.cols; j++)
        {
            if(map.prob[i][j] < 0)
                _map_image.at<uint8_t>(i, j) = 0;
            else if(map.prob[i][j] > 0.20)
                _map_image.at<uint8_t>(i, j) = 0;
            else
                _map_image.at<uint8_t>(i, j) = 255;
        }
    }
    printf("prob = %f\n", map.prob[400][400]);
    printf("prob = %f\n", map.prob[450][450]);
    cv::cvtColor(_map_image, _map_image, CV_GRAY2RGB);
    //cv::circle(_map_image, cv::Point(400, 400), 4, cv::Scalar(0, 0, 255));
    cv::line(_map_image, cv::Point(405, 400), cv::Point(395, 400), cv::Scalar(0, 0, 255));
    cv::line(_map_image, cv::Point(395, 400), cv::Point(400, 405), cv::Scalar(0, 0, 255));
    cv::line(_map_image, cv::Point(400, 405), cv::Point(405, 400), cv::Scalar(0, 0, 255));
    cv::line(_map_image, cv::Point(450, 455), cv::Point(455, 450), cv::Scalar(0, 0, 255));
    imshow("Image", _map_image);
    cv::waitKey(10000);
    
    
    Map map1 = Map();
    Map map2 = Map();
    map1.init_map(map);
    map2.init_map(map);

	int nParticles = 3500;
	map1.init_particles(nParticles);
	//map1.run(logBook1);

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
//  NOTE: I pulled out relevant information and put it in particleFilterNotes pdf
//  4.2 Particle Filter - pg77-84
//  5.4 Odometry MotionModel - pg107-113
//  6 Measurements - pg121-154
//  8.3 MonteCarlo Localization - 200-209 [211-216]
