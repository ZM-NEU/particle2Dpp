#include "bee-map.h"

using namespace std;

class Map {
public:
  Map();
  ~Map();
  
  // Initalize
  void init_map(map_type map);

private:
  map_type _map;
  
};