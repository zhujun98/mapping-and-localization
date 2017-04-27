/*
 * bayesianFilter.h
 */

#ifndef BAYESIANFILTER_H_
#define BAYESIANFILTER_H_

#include <vector>
#include <string>
#include <fstream>

#include "measurement_package.h"
#include "map.h"
#include "utilities.h"

class BayesianFilter
{
public:
  // Constructor
  BayesianFilter();

  // Destructor
  virtual ~BayesianFilter();

  // Estimate the beliefs
  void processMeasurement(const MeasurementPackage &measurements,
                          const Map &map_1d,
                          Utilities &utilities);

  // Belief of state x
  std::vector<float> bel_x_;

private:

  // Flag, if filter is initialized
  bool is_initialized_;

  // Standard deviation of control
  float control_std_;

  // Standard deviation of observations:
  float observation_std_;

  // Initial belief of state x
  std::vector<float> bel_x_init_;

};

#endif /* BAYESIANFILTER_H_ */
