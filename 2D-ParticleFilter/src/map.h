/*
 * map.h
 *
 * The Map class
 */
#ifndef MAP_H_
#define MAP_H_

#include <vector>

struct Landmark {
  int id ; // Landmark ID
  float x; // Landmark x-position in the map (global coordinates)
  float y; // Landmark y-position in the map (global coordinates)
};


class Map {
public:

  std::vector<Landmark> landmark_list_;

  // constructor
  Map();

  // destructor
  ~Map();

  void readData(std::string filename);

  std::vector<Landmark> getLandmarkList() const;
};

#endif /* MAP_H_ */
