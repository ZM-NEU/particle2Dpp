#include "includes/map.h"

using namespace std;

Map::Map() {
	_numParticles = 1000;
	_particles = new particle[_numParticles];
}

Map::~Map() {
	delete[] _particles;
}

// Load map_type in as a class to do things
void Map::init_map(map_type map) {
	_map = map;
}

// Create the particles!
void Map::init_particles(int numParticles)
{
	_numParticles = numParticles;
	delete[] _particles;
	_particles = new particle[_numParticles];
	
	// Randomly get the number of particles needed
	// Don't want -1 cells
	for(int i = 0; i < _numParticles; i++)
	{
		
	}
}

// Move every particle by the odometry step with some uncertainty added
void Map::update_location(step motion)
{

}

// Change the weights!
void Map::update_prediction(lidarData data)
{

}

// Called by update_prediction to see how well lidarData matches for a particle p
float _get_particle_weight(lidarData data, particle p)
{

}

// Sample 0 mean gaussian with variance sigma;
float _sample_with_variance(float sigma)
{
    
}

// Return the total probability on the map (to normalize)
// TODO: Evaluate if this is needed or only sum of weights for chosen particles
float Map::total_probability() {
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
	fprintf(stderr, "Total P: %f\n", totalP);
}