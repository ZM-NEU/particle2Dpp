#ifndef MAP_H
#define MAP_H

#include "bee-map.h"
#include "logBook.h"
#include <vector>

using namespace std;

typedef struct particle {
    pose2D pose;
    float weight;
} particle;

typedef struct step {
    pose2D current;
    pose2D previous;
} step;

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
    void update_location(step motion);

    // Update Phase
    void update_prediction(lidarData data);

private:
    map_type _map;
    int _numParticles;
    particle* _particles;
	logEntry _prevLog;

    // Update the particle's weight
    float _get_particle_weight(lidarData data, particle p);
    
    // Return the state with the highest probability
    pose2D _get_estimated_state(); 
    
    // Sample 0 mean gaussian with variance sigma;
    float _sample_with_variance(float sigma);
    
    // Get total probability of the map
    float _total_probability();
};
#endif // MAP_H
