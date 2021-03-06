/*
 * Modified from the code used in "Markov Localization" course at Udacity.
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "measurement_package.h"
#include "map.h"
#include "utilities.h"

#include "bayesian_filter.h"


int main()
{
  // Define example: 01, 02, 03, 04
  std::string example_string = "03";

  Map map_1d;

  std::vector<MeasurementPackage> measurement_pack_list;

  Utilities utilities;

  // Define file names
  char in_file_name_ctr[1024];
  char in_file_name_obs[1024];
  char in_file_name_gt[1024];

  // Read map
  utilities.readMapData("data/map_1d.txt", map_1d);

  // Define file name of controls
  sprintf(in_file_name_ctr, "data/example%s/control_data.txt",
          example_string.c_str());

  // Define file name of observations
  sprintf(in_file_name_obs, "data/example%s/observations/",
          example_string.c_str());

  // Read in data to measurement package list
  utilities.readMeasurementData(in_file_name_ctr,
                                in_file_name_obs,
                                measurement_pack_list);

  /*
  //print the landmarks in the map
  for (std::size_t i=0; i<map_1d.landmark_list_.size(); ++i) {
    std::cout << "ID " << map_1d.landmark_list_[i].id_i << "\t"
              << "Distance " << map_1d.landmark_list_[i].x_f << std::endl;
  }

  //print the controls and observations
  std::cout << "Print out the measurement packages:" << std::endl;

  for(std::size_t i=0;i<measurement_pack_list.size();i++) {
    std::cout << "Step " << i+1 << " includes the move "
              << measurement_pack_list[i].control_s_.delta_x_f
              << "[m] in driving direction " << std::endl;

    //run over observations
    if (measurement_pack_list[i].observation_s_.distance_f.size() < 1) {

      std::cout << "	No observations in step " << i << std::endl;
    } else {
      std::cout << "	Number of Observations in current step: "
                << measurement_pack_list[i].observation_s_.distance_f.size()
                << std::endl;

      for (int j = 0; j < measurement_pack_list[i].observation_s_.distance_f.size(); j++) {
        std::cout << "	Distance to a landmark: "
                  << measurement_pack_list[i].observation_s_.distance_f[j]
                  << " m" << std::endl;
      }
    }
  }
  */

  BayesianFilter localization_1d_bayesian;

  for (size_t t = 0; t < measurement_pack_list.size(); ++t)
  {
    // Call 1d_bayesian filter
    localization_1d_bayesian.processMeasurement(
        measurement_pack_list[t], map_1d, utilities);
  }

  sprintf(in_file_name_gt, "data/example%s/gt_example%s.txt",
          example_string.c_str(), example_string.c_str() );

  // Compare ground truth data with results
  utilities.compareData(in_file_name_gt, localization_1d_bayesian.bel_x_);

  return 0;
}