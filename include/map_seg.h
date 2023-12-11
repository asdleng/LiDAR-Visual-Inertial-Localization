#ifndef MAP_SEG_H_
#define MAP_SEG_H_

#include <ikd-Tree/ikd_Tree.h>
#include <map.h>
namespace lvo {
class MapSeg{
public:
    MapSeg();
    double cell_size = 100;
    int cell_number = 3;
    bool map_update(const Map &current_map, double current_x, double current_y, Map &new_map);
    

};











}

# endif