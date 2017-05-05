/*
 * map.h
 *
 * The Map class
 */

#ifndef MAP_H_
#define MAP_H_

class Map
{
public:

  struct Landmark
  {
    int id ; // Landmark ID
    float x; // Landmark x-position in the map (global coordinates)
    float y; // Landmark y-position in the map (global coordinates)
  };

  std::vector<Landmark> landmark_list ;

};

#endif /* MAP_H_ */
