/*
 * measurement_package.h
 */

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <vector>

class MeasurementPackage
{
public:

  struct Control
  {
    float delta_x; // move to successor in x position
  };

  struct Observation
  {
    std::vector<float> distance; // distance to observed landmark
  };

  Control control_;
  Observation observation_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */