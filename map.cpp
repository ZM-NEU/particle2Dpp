#include "includes/map.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <random>
#include <opencv2/core/types_c.h>

#define PI 3.14159
#define NUM_RANGES 180

using namespace std;

Map::Map()
{
	// TODO Try different parameters here
	// Map Parameters
	_numParticles = 5000; //Random number
	_particles = new particle[_numParticles];
	_threshold = 0.15;
	_max_laser = 800.0;

	// Sensor Parameters
	_z_hit = 0.8;
	_z_short = 0.095;
	_z_max = 0.005;
	_z_rand = 0.1;
	_sigma_hit = 20;
	_lambda_short = 0.0005;

	// Odometry Parameters
	_a1 = 0.01;
	_a2 = 0.01;
	_a3 = 0.1;
	_a4 = 0.1;

    // Augmented_MCL Parameters
    _a_slow = 0.05;
    _a_fast = 0.2;

    //srand();
	default_random_engine _generator;
	normal_distribution<double> _distribution(0,1);
}

Map::~Map()
{
	delete[] _particles;
	_mapImage = cv::Mat();
}

// Load map_type in as a class to do things
void Map::init_map(map_type map)
{
	_map = map;
	_prevLog.logType = INVALID;

    // Save map as an opencv Mat type to draw on later
    _mapImage = cv::Mat::zeros(map.size_x, map.size_y, CV_8UC1);
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

	// Randomly get the particles needed
	// Don't want -1 cells
	for(int p = 0; p < _numParticles; p++)
	{
		_inject_at(p);
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
        augmented_MCL(logB[i]);
//         draw_particles();
		lidarData data;
		data.ranges = new double[NUM_RANGES];
		if(logB[i].logType == LIDAR)
		{
			for(int j = 0; j < NUM_RANGES; j++)
			{
				data.ranges[j] = logB[i].ranges[j];
			}
			draw_best_lidar(data);
		}
	}
}

void Map::draw_particles()
{
    cv::Mat temp_map;
    cv::cvtColor(_mapImage, temp_map, CV_GRAY2RGB);
    for(int p = 0; p < _numParticles; p++)
    {
        // Plot
        int x = (int)_particles[p].pose.x;
        int y = (int)_particles[p].pose.y;
        double theta = _particles[p].pose.theta;
        cv::circle(temp_map, cv::Point(y, x), 1, cv::Scalar(0, 0, 255));
        //             int xp = x + (int)5*cos(_particles[p].pose.theta);
        //             int yp = y - (int)5*sin(_particles[p].pose.theta);
        //             cv::line(temp_map, cv::Point(y, x), cv::Point(yp, xp), cv::Scalar(0, 0, 255));
    }
    cv::imshow("Image", temp_map);
    cv::waitKey(10);
    temp_map = cv::Mat();
}

void Map::draw_best_lidar(lidarData data)
{
	cv::Mat temp_map;
	cv::cvtColor(_mapImage, temp_map, CV_GRAY2RGB);
    particle avg_part;
    avg_part.pose.x = 0;
    avg_part.pose.y = 0;
    avg_part.pose.theta = 0;

	// Get total average
    int best_idx = 0;
    double eta = 0.0;
	for(int p = 0; p < _numParticles; p++)
	{
//         avg_part.pose.x += _particles[p].weight*_particles[p].pose.x;
//         avg_part.pose.y += _particles[p].weight*_particles[p].pose.y;
//         avg_part.pose.theta += _particles[p].weight*_particles[p].pose.theta;
//         eta += _particles[p].weight;
		best_idx = (_particles[best_idx].weight > _particles[p].weight) ? best_idx : p;
	}
// 	avg_part.weight = _particles[best_idx].weight;


	// Get top average
	int top_num = 20;
	int* top = new int[top_num];
	particle* topP = new particle[top_num];
	for(int i = 0; i < top_num; i++)
	{
		topP[i].weight = -1;
		topP[i].pose.x = -1;
	}
	for(int i = 0; i < top_num; i++)
	{
		for(int p = 0; p < _numParticles; p++)
		{
			bool topW = true;
			if(_particles[p].weight > topP[i].weight)
			{
				for(int j = 0; j < i; j++)
				{
					if(topP[j].pose.x == p)
					{
						topW = false;
					}
				}
				if(topW)
				{
					topP[i].weight = _particles[p].weight;
					topP[i].pose.x = p;
				}
			}
		}
	}
	for(int i = 0; i < top_num; i++)
	{
		eta += topP[i].weight;
		int p = topP[i].pose.x;
		topP[i] = _particles[p];
	}
	for(int i = 0; i < top_num; i++)
	{
		avg_part.pose.x += topP[i].weight*topP[i].pose.x;
		avg_part.pose.y += topP[i].weight*topP[i].pose.y;
		avg_part.pose.theta += topP[i].weight*topP[i].pose.theta;
	}

// 	// Plot highest weighted particle
// 	int x = (int)_particles[best_idx].pose.x;
// 	int y = (int)_particles[best_idx].pose.y;
// 	double theta = _particles[best_idx].pose.theta;

//     // Plot weighted average particle
//     int x = (int)(avg_part.pose.x/eta);
//     int y = (int)(avg_part.pose.y/eta);
//     double theta = avg_part.pose.theta/eta;

	// Plot top N particle's weighted average
	int x = (int)(avg_part.pose.x/eta);
	int y = (int)(avg_part.pose.y/eta);
	double theta = avg_part.pose.theta/eta;



    fprintf(stderr,"Best (%i %i %f)\n",x,y,theta);
	// TODO Plot the lidar values at the weighted average location
	// TODO Plot the expected values instead of the actual?
	for(int i = 0; i < NUM_RANGES; i++)
	{
		int x2 = (int)(data.ranges[i]*cos(i*PI/180 + theta - PI/2) + x);
		int y2 = (int)(data.ranges[i]*sin(i*PI/180 + theta - PI/2) + y);
		if (x2 < _map.min_x || x2 > _map.max_x || y2 < _map.min_y || y2 > _map.max_y)
			continue;
		cv::line(temp_map, cv::Point(y, x), cv::Point(y2, x2), cv::Scalar(0, 255, 0));
	}
	for(int p = 0; p < _numParticles; p++)
	{
		// Plot
		int x = (int)_particles[p].pose.x;
		int y = (int)_particles[p].pose.y;
		double theta = _particles[p].pose.theta;
		int w = (int)(_particles[p].weight/_particles[best_idx].weight*255);
		cv::circle(temp_map, cv::Point(y, x), 1, cv::Scalar(255 - w, 0, w));
	}
	cv::imshow("Image", temp_map);
	cv::waitKey(10);
	temp_map = cv::Mat();
}

// Updates the map for a single logEntry
void Map::augmented_MCL(logEntry logB)
{
    static int count;
	static double w_slow;
	static double w_fast;
    double w_avg = 0;
    pose2D motion;
    motion.x = logB.robotPose.x - _prevLog.robotPose.x;
    motion.y = logB.robotPose.y - _prevLog.robotPose.y;
    motion.theta = logB.robotPose.theta - _prevLog.robotPose.theta;
    if(fabs(motion.x) < 0.01  && fabs(motion.y) < 0.01 && fabs(motion.theta) < 0.01)
    {
        return;
    }
    lidarData data;
    double eta_weights = 0;
    data.ranges = new double[NUM_RANGES];
	int w_max_in = 0;
	int w_min_in = 0;
	//fprintf(stderr,"\n");
    for(int m = 0; m < _numParticles; m++)
    {
        // Sample Motion Model
        _particles[m].pose = _sample_motion_model_odometry(motion, m);
		//fprintf(stderr,"p%i(x,y,t,w)=(%f,%f,%f,%f) ",m,_particles[m].pose.x,_particles[m].pose.y,_particles[m].pose.theta,_particles[m].weight);
        // Measurement Model
        if(logB.logType == LIDAR)
        {
            for(int phi = 0; phi < NUM_RANGES; phi++)
            {
                data.ranges[phi] = logB.ranges[phi];
            }
            _particles[m].weight = _get_particle_weight(data, m);
//             _particles[m].weight = _get_particle_weight2(data, m);
			w_max_in = (_particles[w_max_in].weight > _particles[m].weight) ? w_max_in : _particles[m].weight;
			w_min_in = (_particles[w_min_in].weight < _particles[m].weight || _particles[m].weight <= 0.00) ? w_min_in : _particles[m].weight;
        }
        else
        {
            return;
        }
        eta_weights += _particles[m].weight;
    }

    //fprintf(stderr,"\neta_weights: %f\n",eta_weights);
    for(int m = 0; m < _numParticles; m++)
    {
        _particles[m].weight /= eta_weights;
        w_avg += _particles[m].weight/_numParticles;
    }
    w_slow += _a_slow*(w_avg - w_slow);
	w_fast += _a_fast*(w_avg - w_fast);

    // TODO: Fix resampling!
    // A. Use this method
        // Is this method implemented correctly?
    // B. Resample based off variance
    pose2D var = _get_particle_variance();
    fprintf(stderr,"Variance (%f %f %f)\n",var.x,var.y,var.theta);
	if(_particles[w_max_in].weight/_particles[w_max_in].weight > 10)
	{
		_resample(1.0 - w_fast/w_slow);
	}
	else if(_particles[w_max_in].weight/_particles[w_max_in].weight > 2)
    {
        _low_variance_sampler();
    }

    // Save last log entry robot pose for next update
    _prevLog.logType = logB.logType;
    _prevLog.robotPose.x = logB.robotPose.x;
    _prevLog.robotPose.y = logB.robotPose.y;
    _prevLog.robotPose.theta = logB.robotPose.theta;
}

pose2D Map::_get_particle_variance()
{
    pose2D variance;
    variance.x = 0;
    variance.y = 0;
    variance.theta = 0;
    pose2D sum;
    sum.x = 0;
    sum.y = 0;
    sum.theta = 0;
    pose2D sumsq;
    sumsq.x = 0;
    sumsq.y = 0;
    sumsq.theta = 0;
    for(int p = 0; p < _numParticles; p++)
    {
        sum.x += _particles[p].pose.x;
        sum.y += _particles[p].pose.y;
        sum.theta += _particles[p].pose.theta;
        sumsq.x += (_particles[p].pose.x)*(_particles[p].pose.x);
        sumsq.y += (_particles[p].pose.y)*(_particles[p].pose.y);
        sumsq.theta += (_particles[p].pose.theta)*(_particles[p].pose.theta);
    }
    variance.x = sqrt((sumsq.x - (sum.x*sum.x)/_numParticles)/_numParticles);
    variance.y = sqrt((sumsq.y - (sum.y*sum.y)/_numParticles)/_numParticles);
    variance.theta = sqrt((sumsq.theta - (sum.theta*sum.theta)/_numParticles)/_numParticles);
    return variance;
}

void Map::_low_variance_sampler()
{
    particle* samples = new particle[_numParticles];
    double r = (rand()/(double)RAND_MAX)/_numParticles;
    double c = _particles[0].weight;
    int i = 0;
    for(int p = 0; p < _numParticles; p++)
    {
        double u = r + ((double)p)/((double)_numParticles);
        while(u > c)
        {
            if(i >= _numParticles)
            {
                i = _numParticles - 1;
                continue;
            }
            i++;
            c += _particles[i].weight;
        }
        samples[p] = _particles[i];
    }
    for(int i = 0; i < _numParticles; i++)
    {
        _particles[i].pose.x = samples[i].pose.x;
        _particles[i].pose.y = samples[i].pose.y;
        _particles[i].pose.theta = samples[i].pose.theta;
        _particles[i].weight = samples[i].weight;
    }
}

void Map::_resample(double p_rand_pose)
{
	double r_rand_pose = rand()/(double)RAND_MAX;
	//fprintf(stderr, "0 < R:%f < P(r):%f\n",r_rand_pose,p_rand_pose);
	particle* samples = new particle[_numParticles];
	double r = (rand()/(double)RAND_MAX)/_numParticles;
	double c = _particles[0].weight;
	int i = 0;
	for(int m = 0; m < _numParticles; m++)
	{
		if(p_rand_pose > 0 && r_rand_pose < p_rand_pose)
		{
			// TODO: Figure out if this is ever called
			double x = (_map.max_x - _map.min_x)/(double)RAND_MAX;
			double y = (_map.max_y - _map.min_y)/(double)RAND_MAX;
			double theta = 2*PI/(double)RAND_MAX;
			double prob;
			// Draw a new random position
			do {
				samples[m].pose.x = _map.min_x + rand()*x;
				samples[m].pose.y = _map.min_y + rand()*y;
				samples[m].pose.theta = rand()*theta;
				samples[m].weight = 1.0/_numParticles;
				prob = _map.prob[(int)samples[m].pose.x][(int)samples[m].pose.y];
			} while(prob > _threshold || prob < 0); // Want to pick spaces that are free (close to 0)
		}
		else
		{
			double u = r + ((double)m)/((double)_numParticles);
			while(u > c)
			{
				if(i >= _numParticles)
				{
					i = _numParticles - 1;
					continue;
				}
				i++;
				c += _particles[i].weight;
			}
			samples[m] = _particles[i];
		}
	}
	for(int i = 0; i < _numParticles; i++)
	{
		_particles[i].pose.x = samples[i].pose.x;
		_particles[i].pose.y = samples[i].pose.y;
		_particles[i].pose.theta = samples[i].pose.theta;
		_particles[i].weight = samples[i].weight;
	}
}

// From Probabilistic Robotics book - samples motion
pose2D Map::_sample_motion_model_odometry(pose2D motion, int _p)
{
    particle p = _particles[_p];
    double dr1 = atan2(motion.y, motion.x) - p.pose.theta;
    double dtr = sqrt(motion.x*motion.x + motion.y*motion.y);
    double dr2 = motion.theta - dr1;
	double dhr1 = dr1 - _distribution(_generator)*(_a1*dr1 + _a2*dtr);
	double dhtr = dtr - _distribution(_generator)*(_a3*dtr + _a4*(dr1 + dr2));
	double dhr2 = dr2 - _distribution(_generator)*(_a1*dr2 + _a2*dtr);
	pose2D newPose;
	newPose.x = p.pose.x + dhtr*cos(p.pose.theta + dhr1);
	newPose.y = p.pose.y + dhtr*sin(p.pose.theta + dhr1);
// 	newPose.theta = p.pose.theta + dhr1 + dhr2;
    newPose.theta = wrap(p.pose.theta + dhr1 + dhr2, 0, 2*PI);
	return newPose;
}

// Called by update_prediction to see how well lidarData matches for a particle p
double Map::_get_particle_weight(lidarData data, int p)
{
	pose2D particle_pose = _particles[p].pose;
    double lidar_offset = 2.5;
	pose2D lidar;
	lidar.x = particle_pose.x + lidar_offset*cos(particle_pose.theta);
	lidar.y = particle_pose.y + lidar_offset*sin(particle_pose.theta);
	if(lidar.x < _map.min_x || lidar.x > _map.max_x || lidar.y < _map.min_y || lidar.y > _map.max_y)
	{
		_inject_at(p);
		return 0.00;
	}
    double prob = _map.prob[(int)lidar.x][(int)lidar.y];
	if(prob < 0 || prob > _threshold)
	{
		_inject_at(p);
		return 0.00;
	}
	double weight = 0.0;
	for(int i = 0; i < NUM_RANGES; i++)
	{
        lidar.theta = ((double)i)*PI/180 + particle_pose.theta;
        double d_expected = _raytrace2(lidar, data.ranges[i]);

        double eta_hit = 1.0/sqrt(2*PI*_sigma_hit*_sigma_hit);
        double p_hit = eta_hit*exp((-0.5*(data.ranges[i] - d_expected)*(data.ranges[i] - d_expected))/(_sigma_hit*_sigma_hit));
        double eta_short = 1.0/(1 - exp(-1*_lambda_short*d_expected));
        double p_short = (data.ranges[i] < d_expected) ? eta_short*_lambda_short*exp(-_lambda_short*data.ranges[i]) : 0.000;
        double p_max = (data.ranges[i] > _max_laser - 0.1 ? 1 : 0);
        double p_rand = 1.0/_max_laser;
        double p_total = _z_hit*p_hit + _z_short*p_short + _z_max*p_max + _z_rand*p_rand;
        if(p_total <= 0)
            fprintf(stderr,"\nERROR (p,r):(%i,%i)!",p,i);
		// TODO Try different particle weighting algorithms and methods
		weight += log(1+p_total);
	}
	if(weight < 0 || fabs(weight) > 1000)
        fprintf(stderr,"w: %f\n",weight);
	return weight;
}

void Map::_inject_at(int p)
{
	double x = (_map.max_x - _map.min_x)/(double)RAND_MAX;
	double y = (_map.max_y - _map.min_y)/(double)RAND_MAX;
	double theta = 2*PI/(double)RAND_MAX;
	double prob;
	// Randomly get the particles needed
	// Don't want -1 cells
	do {
		_particles[p].pose.x = _map.min_x + rand()*x;
		_particles[p].pose.y = _map.min_y + rand()*y;
		_particles[p].pose.theta = rand()*theta;
		_particles[p].weight = 1.0/_numParticles;
		prob = _map.prob[(int)_particles[p].pose.x][(int)_particles[p].pose.y];
	} while(prob > _threshold || prob < 0); // Want to pick spaces that are free (close to 0)
}

// Ray trace to find the expected distance
double Map::_raytrace(pose2D vec, double range)
{
	int i = 0;
	int x;
    int y;
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

// Ray trace to find the expected distance
double Map::_raytrace2(pose2D vec, double range)
{
    double lower = 0;
    double upper = _max_laser;

    while(upper - lower > 1.0)
    {
        double mid = (upper + lower)/2;
        int x = (int)(mid*cos(vec.theta - PI/2) + vec.x);
        int y = (int)(mid*sin(vec.theta - PI/2) + vec.y);
		if(x < _map.min_x || x > _map.max_x || y < _map.min_y || y > _map.max_y)
            upper = mid;
        else if(_map.prob[x][y] <= _threshold && _map.prob[x][y] >= 0) // If open move lower bound
            lower = mid;
        else
            upper = mid;
    }
    return (upper + lower)/2;
}

// Used to wrap numbers i.e. wrap(angle, 0, 2PI)
double wrap(double num, double min, double max)
{
    if(min > max)
        fprintf(stderr, "ERROR min(%f) > max(%f) in WRAP",min, max);
    return fmod(num, max) + ((num < min) ? max : 0);
}