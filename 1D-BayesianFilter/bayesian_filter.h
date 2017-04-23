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
#include "help_functions.h"

class BayesianFilter {
public:
  //constructor
  BayesianFilter();

  //destructor
  virtual ~BayesianFilter();

  //estimate the beliefs
  void process_measurement(const MeasurementPackage &measurements,
                           const map &map_1d,
                           help_functions &helpers);

  //belief of state x
  std::vector<float> bel_x_;

private:

  //flag, if filter is initialized
  bool is_initialized_;

  //precision of control information
  float control_std_;

  //initial belief of state x
  std::vector<float> bel_x_init_;

};

#endif /* BAYESIANFILTER_H_ */
