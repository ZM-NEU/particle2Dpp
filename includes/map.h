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
    float weight;
} particle;

typedef struct lidarData {
    float* ranges;
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

    // Prediction Phase
    void update_location(pose2D motion);

    // Update Phase
    void update_prediction(lidarData data);

private:
    map_type _map;
    int _numParticles;
    particle* _particles;
	logEntry _prevLog;
    cv::Mat _mapImage;
    double _threshold;
	double _max_laser;
	float _z_hit;
	float _z_short;
	float _z_max;
	float _z_rand;
	float _sigma_hit;
	float _lambda_short;
	float _a1;
	float _a2;
	float _a3;
	float _a4;

	// From Probabilistic Robotics book
	pose2D _sample_motion_model_odometry(pose2D motion, pose2D particle_pose);

    // Update the particle's weight
    float _get_particle_weight(lidarData data, pose2D particle_pose);

    // Return the state with the highest probability
    pose2D _get_estimated_state();

    // Sample 0 mean gaussian with variance sigma;
    float _sample_with_variance(float sigma);

	// From book
    void _low_variance_sampler(float eta_weights);

	// Ray trace to find the expected distance
    float _raytrace(pose2D vec, float range);
};

float wrap(float num, float min, float max);
#endif // MAP_H
