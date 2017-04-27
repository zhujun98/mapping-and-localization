/*
 * map.h
 */
#ifndef MAP_H_
#define MAP_H_

class Map
{
public:
  // Define a single landmark:
  struct Landmark
  {
    int id;
    float x;
  };

  std::vector<Landmark> landmark_list_;
};
#endif /* MAP_H_ */