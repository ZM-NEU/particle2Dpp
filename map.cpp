#include "includes/map.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <random>

#define PI 3.14159
#define NUM_RANGES 180

using namespace std;

Map::Map() {
	_numParticles = 1000; //Random number
	_particles = new particle[_numParticles];
    _sigma_lidar = 10;
    _sigma_odom = 10;
}

Map::~Map() {
	delete[] _particles;
}

// Load map_type in as a class to do things
void Map::init_map(map_type map) {
	_map = map;
	_prevLog.logType = INVALID;
}

// Create the particles!
void Map::init_particles(int numParticles)
{
	_numParticles = numParticles;
	delete[] _particles;
	_particles = new particle[_numParticles];

	float x = (_map.max_x - _map.min_x)/(float)RAND_MAX;
	float y = (_map.max_y - _map.min_y)/(float)RAND_MAX;
	float theta = 2*PI/(float)RAND_MAX;
	float prob;
	// Randomly get the particles needed
	// Don't want -1 cells
	for(int i = 0; i < _numParticles; i++)
	{
		do {
			_particles[i].pose.x = _map.min_x + rand()*x;
			_particles[i].pose.y = _map.min_y + rand()*y;
			_particles[i].pose.theta = rand()*theta;
			prob = _map.prob[(int)_particles[i].pose.x][(int)_particles[i].pose.y];
		} while(prob <= 0.99); // Want to pick spaces that are free (close to 1)
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
	}
}

// Updates the map for a single logEntry
void Map::run_single_step(logEntry logB)
{
	lidarData data;
	data.ranges = new float[NUM_RANGES];
	// Get odometry and update motion
	pose2D motion;
    motion.x = logB.robotPose.x - _prevLog.robotPose.x;
    motion.y = logB.robotPose.y - _prevLog.robotPose.y;
    motion.theta = logB.robotPose.theta - _prevLog.robotPose.theta;
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
        // TODO: Find a good weighting system for the sample
        _particles[i].pose.x += motion.x*(1 + _sample_with_variance(_sigma_odom));
        _particles[i].pose.y += motion.y*(1 + _sample_with_variance(_sigma_odom));
        float theta = _particles[i].pose.theta + motion.theta*(1 + _sample_with_variance(_sigma_odom));
        // wrap theta [0,2PI)
        _particles[i].pose.theta = fmod(theta,2*PI) + ((theta < 0) ? 2*PI : 0);
    }
}

// Change the weights!
void Map::update_prediction(lidarData data)
{
    for(int i = 0; i < _numParticles; i++)
    {
        
    }
}

// Called by update_prediction to see how well lidarData matches for a particle p
float Map::_get_particle_weight(lidarData data, particle p)
{

}

// Sample 0 mean gaussian with variance sigma;
float Map::_sample_with_variance(float sigma)
{
    default_random_engine generator;
    normal_distribution<float> distribution(0,sigma);
    return distribution(generator);
}

// Return the total probability on the map (to normalize)
// TODO: Evaluate if this is needed or only sum of weights for chosen particles
float Map::_total_probability() {
	int sx = _map.size_x;
	int sy = _map.size_y;
	float totalP = 0.0;
	for(int i = 0; i < sx; i++)
	{
		for(int j = 0; j < sy; j++)
		{
			if(_map.prob[i][j] >= 0)
			{
				totalP += _map.prob[i][j];
			}
		}
	}
	// TODO: Fix "Declaration not found for fprintf(stderr,...);
	fprintf(stderr, "Total P: %f\n", totalP);
    
    int numValid = 0;
    int numInRange = 0;
    float max = 0.9;
    for(int i = 0; i < sx; i++)
    {
        for(int j = 0; j < sy; j++)
        {
            if(_map.prob[i][j] >= 0)
            {
                numValid++;
                if(_map.prob[i][j] <= max)
                {
                    numInRange ++;
                }
            }
        }
    }
    // 0.85 --> 156243
    fprintf(stderr, "Total: %i\tTotal less than %.2f: %i\n", numValid, max, numInRange);
}