#include "includes/map.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <random>
#include <opencv2/core/types_c.h>

#define PI 3.14159
#define NUM_RANGES 180

using namespace std;

Map::Map() {
	// Map Parameters
	_numParticles = 1000; //Random number
	_particles = new particle[_numParticles];
	_threshold = 0.1;
	_max_laser = 800.0;

	// Sensor Parameters
	_z_hit = 0.65;
	_z_short = 0.17;
	_z_max = 0.04;
	_z_rand = 0.14;
	_sigma_hit = 5;
	_lambda_short = 0.001;

	// Odometry Parameters
	_a1 = 0.01;
	_a2 = 0.01;
	_a3 = 0.1;
	_a4 = 0.1;
}

Map::~Map() {
	delete[] _particles;
}

// Load map_type in as a class to do things
void Map::init_map(map_type map) {
	_map = map;
	_prevLog.logType = INVALID;

    // Save map as an opencv Mat type to draw on later
    _mapImage = cv::Mat::zeros( map.size_x, map.size_y, CV_8UC1 );
    for(int i = 0; i < _mapImage.rows; i++)
    {
        for(int j = 0; j < _mapImage.cols; j++)
        {
            if(map.prob[i][j] < 0)
                _mapImage.at<uint8_t>(i, j) = 0;
            else if(map.prob[i][j] > _threshold)
                _mapImage.at<uint8_t>(i, j) = 0;
            else
                _mapImage.at<uint8_t>(i, j) = 255;
        }
    }
}

// Create the particles!
void Map::init_particles(int numParticles)
{
	_numParticles = numParticles;
	delete[] _particles;
	_particles = new particle[_numParticles];

    double x = (_map.max_x - _map.min_x)/(double)RAND_MAX;
    double y = (_map.max_y - _map.min_y)/(double)RAND_MAX;
    double theta = 2*PI/(double)RAND_MAX;
    double prob;
	// Randomly get the particles needed
	// Don't want -1 cells
	for(int i = 0; i < _numParticles; i++)
	{
		do {
			_particles[i].pose.x = _map.min_x + rand()*x;
			_particles[i].pose.y = _map.min_y + rand()*y;
			_particles[i].pose.theta = rand()*theta;
			_particles[i].weight = 1.0/_numParticles;
			prob = _map.prob[(int)_particles[i].pose.x][(int)_particles[i].pose.y];
		} while(prob > _threshold || prob < 0); // Want to pick spaces that are free (close to 0)
	}
}

// Run through a log file
void Map::run(vector<logEntry> logB)
{
	int start_index = 0;
	if(_prevLog.logType == INVALID)
	{
		_prevLog.logType = logB[start_index].logType;
		_prevLog.robotPose.x = logB[start_index].robotPose.x;
		_prevLog.robotPose.y = logB[start_index].robotPose.y;
		_prevLog.robotPose.theta = logB[start_index].robotPose.theta;
		start_index++;
	}
	for(int i = start_index; i < logB.size(); i++)
	{
        fprintf(stderr, "Starting line %i of %lu\n", i, logB.size()-1);
		run_single_step(logB[i]);
        cv::Mat temp_map;
        cv::cvtColor(_mapImage, temp_map, CV_GRAY2RGB);
        int p = 1;
        for(p; p < _numParticles; p++)
        {
            // Plot
            int x = (int)_particles[p].pose.x;
            int y = (int)_particles[p].pose.y;
            double theta = _particles[p].pose.theta;
			cv::circle(temp_map, cv::Point(y, x), 1, cv::Scalar(0, 0, 255));
//          cv::line(temp_map, cv::Point(405, 400), cv::Point(395, 400), cv::Scalar(0, 0, 255));
//          cv::line(temp_map, cv::Point(395, 400), cv::Point(400, 405), cv::Scalar(0, 0, 255));
//          cv::line(temp_map, cv::Point(400, 405), cv::Point(405, 400), cv::Scalar(0, 0, 255));
        }
        cv::imshow("Image", temp_map);
        cv::waitKey(10);
        temp_map = cv::Mat();
	}
}

// Updates the map for a single logEntry
void Map::run_single_step(logEntry logB)
{
	lidarData data;
    data.ranges = new double[NUM_RANGES];
	// Get odometry and update motion
	pose2D motion;
    motion.x = logB.robotPose.x - _prevLog.robotPose.x;
    motion.y = logB.robotPose.y - _prevLog.robotPose.y;
    motion.theta = logB.robotPose.theta - _prevLog.robotPose.theta;
	if(fabs(motion.x) < 0.01  && fabs(motion.y) < 0.01 && fabs(motion.theta) < 0.01)
	{
		return;
	}
	update_location(motion);

	// Get sensor data and update prediction
	if(logB.logType == LIDAR)
	{
		for(int phi = 0; phi < NUM_RANGES; phi++)
		{
			data.ranges[phi] = logB.ranges[phi];
		}
		update_prediction(data);
	}

	// Save last log entry robot pose for next update
	_prevLog.logType = logB.logType;
	_prevLog.robotPose.x = logB.robotPose.x;
	_prevLog.robotPose.y = logB.robotPose.y;
	_prevLog.robotPose.theta = logB.robotPose.theta;
}

// Move every particle by the odometry step with some uncertainty added
void Map::update_location(pose2D motion)
{
    for(int i = 0; i < _numParticles; i++)
    {
		_particles[i].pose = _sample_motion_model_odometry(motion, _particles[i].pose);
        // wrap theta [0,2PI)
        _particles[i].pose.theta = wrap(_particles[i].pose.theta, 0, 2*PI);
    }
}

// Change the weights!
void Map::update_prediction(lidarData data)
{
    double eta_weights = 0;
	for(int i = 0; i < _numParticles; i++)
	{
		_particles[i].weight = _get_particle_weight(data, i);
		eta_weights += _particles[i].weight;
	}
	_low_variance_sampler(eta_weights);
}

// From book
void Map::_low_variance_sampler(double eta_weights)
{
	particle* samples = new particle[_numParticles];
    double r = (rand()/(double)RAND_MAX)/_numParticles;
    double c = _particles[0].weight/eta_weights;
// 	fprintf(stderr,"r:%f c:%f\n",r,c);
	int i = 0;
	for(int m = 0; m < _numParticles; m++)
	{
        double u = r + ((double)m)/((double)_numParticles);
		while(u > c)
		{
            if(i >= _numParticles)
                continue;
            i++;
			c += _particles[i].weight/eta_weights;
// 			if(i >= _numParticles)
// 				fprintf(stderr,"want u:%f > c:%f at index i:%i OUTSIDE OF RANGE!!!!! m:%i\n",u,c,i,m);
		}
// 		samples[m] = _particles[i];
		samples[m].pose.x = _particles[i].pose.x;
		samples[m].pose.y = _particles[i].pose.y;
		samples[m].pose.theta = _particles[i].pose.theta;
		samples[m].weight = _particles[i].weight;
	}
	for(int i = 0; i < _numParticles; i++)
	{
		_particles[i].pose.x = samples[i].pose.x;
		_particles[i].pose.y = samples[i].pose.y;
		_particles[i].pose.theta = samples[i].pose.theta;
		_particles[i].weight = samples[i].weight;
	}
//     delete _particles;
// 	_particles = samples;
}

// From Probabilistic Robotics book - samples motion
pose2D Map::_sample_motion_model_odometry(pose2D motion, pose2D particle_pose)
{
    double dr1 = atan2(motion.y, motion.x) - particle_pose.theta;
    double dtr = sqrt(motion.x*motion.x + motion.y*motion.y);
    double dr2 = motion.theta - dr1;
    double dhr1 = dr1 - _sample_with_variance(_a1*dr1 + _a2*dtr);
    double dhtr = dtr - _sample_with_variance(_a3*dtr + _a4*(dr1 + dr2));
    double dhr2 = dr2 - _sample_with_variance(_a1*dr2 + _a2*dtr);
	pose2D newPose;
	newPose.x = particle_pose.x + dhtr*cos(particle_pose.theta + dhr1);
	newPose.y = particle_pose.y + dhtr*sin(particle_pose.theta + dhr1);
	newPose.theta = particle_pose.theta + dhr1 + dhr2;
	return newPose;
}

// Called by update_prediction to see how well lidarData matches for a particle p
double Map::_get_particle_weight(lidarData data, int p)
{
	pose2D particle_pose = _particles[p].pose;
    double lidar_offset = 2.5;
	pose2D lidar;
	lidar.x = particle_pose.x + lidar_offset*cos(particle_pose.theta);
	lidar.y = particle_pose.y - lidar_offset*sin(particle_pose.theta);

    double prob = _map.prob[(int)lidar.x][(int)lidar.y];
	if(lidar.x < _map.min_x || lidar.x > _map.max_x || lidar.y < _map.min_y || lidar.y > _map.max_y || prob < 0 || prob > _threshold)
	{
		return 0.000;
	}
	//fprintf(stderr, "NO RETURN\n");
	double weight = 0.0;
	for(int i = 0; i < NUM_RANGES; i++)
	{
        lidar.theta = ((double)i)*PI/180 + particle_pose.theta;
        double d_expected = _raytrace(lidar, data.ranges[i]);

        double eta_hit = 1.0/sqrt(2*PI*_sigma_hit*_sigma_hit);
        double p_hit = eta_hit*exp((-0.5*(data.ranges[i] - d_expected)*(data.ranges[i] - d_expected))/(_sigma_hit*_sigma_hit));
        double eta_short = 1.0/(1 - exp(-1*_lambda_short*d_expected));
        double p_short = (data.ranges[i] < d_expected) ? eta_short*_lambda_short*exp(-_lambda_short*data.ranges[i]) : 0.000;
        double p_max = (data.ranges[i] > _max_laser - 0.1 ? 1 : 0);
        double p_rand = 1.0/_max_laser;
        double p_total = _z_hit*p_hit + _z_short*p_short + _z_max*p_max + _z_rand*p_rand;
		weight += p_total;
	}
	return weight;
}

// Ray trace to find the expected distance
double Map::_raytrace(pose2D vec, double range)
{
	int i = 0;
	int x,y;
    double prob;
	do
	{
		x = (int)(vec.x + i*sin(vec.theta));
		y = (int)(vec.y - i*cos(vec.theta));
		prob = _map.prob[x][y];
		i++;
	} while(prob < _threshold && prob > 0);
	i--;
    double approx_dx = vec.x + ((double)i)*sin(vec.theta);
    double approx_dy = vec.y - ((double)i)*cos(vec.theta);
    double approx_d = sqrt(approx_dx*approx_dx + approx_dy*approx_dy) + (1-prob);
	return (approx_d > _max_laser) ? _max_laser : approx_d;
}

// Sample 0 mean gaussian with variance sigma;
double Map::_sample_with_variance(double sigma)
{
    default_random_engine generator;
    normal_distribution<double> distribution(0,sigma);
    return distribution(generator);
}

// Used to wrap numbers i.e. wrap(angle, 0, 2PI)
double wrap(double num, double min, double max)
{
    if(min > max)
        fprintf(stderr, "ERROR min(%f) > max(%f) in WRAP",min, max);
    return fmod(num, max) + ((num < min) ? max : 0);
}