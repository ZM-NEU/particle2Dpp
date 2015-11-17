#include "includes/map.h"

using namespace std;

Map::Map() {
  
}

Map::~Map() {
}

// Load map_type in as a class to do things
void Map::init_map(map_type map) {
  _map = map;
  
}

// Return the total probability on the map (to normalize)
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