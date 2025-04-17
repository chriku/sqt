#ifndef OSM_HPP
#define OSM_HPP

#include "sqt_tree.hpp"

enum class tile { undefined = 0, coast, water };

void read_osm(sqt_tree<tile>& tree);
void mark_coast(sqt_tree<tile>& tree);

#endif // OSM_HPP
