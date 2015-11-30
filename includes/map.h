#ifndef MAP_H
#define MAP_H

#include "bee-map.h"
#include "logBook.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;

typedef struct particle {
    pose2D pose;
    double weight;
} particle;

typedef struct lidarData {
    double* ranges;
} lidarData;

class Map {
public:
    Map();
    ~Map();

    // Load map_type
    void init_map(map_type map);
    void init_particles(int numParticles);

	// Run through a log file
	void run(vector<logEntry> log);

	// Run a single logEntry
	void run_single_step(logEntry logB);

    void augmented_MCL(logEntry logB);
	void mcl(logEntry logB);

    // Prediction Phase
    void update_location(pose2D motion);

    // Update Phase
    void update_prediction(lidarData data);
    void draw_particles();
    void draw_best_lidar(lidarData data);

private:
    map_type _map;
    int _numParticles;
    particle* _particles;
	logEntry _prevLog;
    cv::Mat _mapImage;
    double _threshold;
	double _max_laser;
    double _z_hit;
    double _z_short;
    double _z_max;
    double _z_rand;
    double _sigma_hit;
    double _lambda_short;
    double _a1;
    double _a2;
    double _a3;
    double _a4;
    double _a_slow;
    double _a_fast;
	default_random_engine _generator;
	normal_distribution<double> _distribution;

	// From Probabilistic Robotics book
	pose2D _sample_motion_model_odometry(pose2D motion, int _p);

    // Update the particle's weight
    double _get_particle_weight(lidarData data, int p);
    double _get_particle_weight2(lidarData data, int p);

    // Return the state with the highest probability
    pose2D _get_estimated_state();

	void _inject_at(int p);

	// From book
    void _low_variance_sampler();

    void _resample(double p_rand_pose);

	// Ray trace to find the expected distance
    double _raytrace(pose2D vec, double range);
    double _raytrace2(pose2D vec, double range);
    double _raytrace3(pose2D vec, double range);
    pose2D _get_particle_variance();
    double _gaussian_noise(double _zk, double _zkp, double _sigma);
};

double wrap(double num, double min, double max);
#endif // MAP_H
