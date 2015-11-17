#include "includes/map.h"

using namespace std;

Map::Map() {
  
}

Map::~Map() {
}

void Map::init_map(map_type map) {
  _map = map;
  int sx = map.size_x;
  int sy = map.size_y;
  for(int i = 0; i < sx; i++)
  {
    for(int j = 0; j < sy; j++)
    {
      if(map.prob[i][j] >= 0)
      {
	
      }
    }
  }
  
}