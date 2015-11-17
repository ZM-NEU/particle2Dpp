#include "bee-map.h"

using namespace std;

class Map {
public:
  Map();
  ~Map();
  
  // Load map_type
  void init_map(map_type map);

private:
  map_type _map;
  
  // Get total probability of the map
  float total_probability();
  
};