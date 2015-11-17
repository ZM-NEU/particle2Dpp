#ifndef BEEMAP_H
#define BEEMAP_H

// Other needed libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Define structs and typedef
typedef struct map_type {
    int resolution, size_x, size_y;
    float offset_x, offset_y;
    int min_x, max_x, min_y, max_y;
    float **prob;
} map_type;

// Prototype non external functions
void new_hornetsoft_map(map_type *map, int size_x, int size_y);

// Prototype external functions
int read_beesoft_map(const char *mapName, map_type *map);

#endif // BEEMAP_H