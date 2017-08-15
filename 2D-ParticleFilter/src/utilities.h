/*
 * utilities.h
 * Some helper functions for the 2D particle filter.
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <sstream>
#include <fstream>
#include <vector>
#include "map.h"


//
// control feedback
//
struct Control {
  double velocity; // Velocity [m/s]
  double yaw_rate; // Yaw rate [rad/s]
};

//
// ground truth coordinates
//
struct GroundTruth {
  double x; // Global vehicle x position [m]
  double y;	// Global vehicle y position
  double theta;	// Global vehicle yaw [rad]
};

//
// landmark observations
//
struct Observation {
  int id; // Id of matching landmark in the map.
  double x; // Local (vehicle coordinates) x position of landmark [m]
  double y;	// Local (vehicle coordinates) y position of landmark [m]
};

//
// Computes the Euclidean distance between two 2D points.
// @param (x1,y1): x and y coordinates of first point
// @param (x2,y2): x and y coordinates of second point
// @return: Euclidean distance between two 2D points

inline double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

//
//
//
inline double * getError(double gt_x, double gt_y, double gt_theta,
                         double pf_x, double pf_y, double pf_theta) {
  static double error[3];
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  if (error[2] > M_PI) {
    error[2] = 2.0 * M_PI - error[2];
  }
  return error;
}

//
// Reads control data from a file.
//
// @param filename: Name of file containing control measurements.
// @param control:
// @return: True if opening and reading file was successful
//
inline bool readControlData(std::string filename, std::vector<Control>& controls) {
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);

  if (!in_file_pos) { return false; }

  std::string line_pos;

  while(getline(in_file_pos, line_pos)) {
    std::istringstream iss_pos(line_pos);

    Control control;

    iss_pos >> control.velocity;
    iss_pos >> control.yaw_rate;

    controls.push_back(control);
  }
  return true;
}

//
// Reads ground truth data from a file.
//
// @param filename: Name of file containing ground truth.
// @param ground_truth:
// @return: True if opening and reading file was successful
//
inline bool readGroundTruthData(std::string filename, std::vector<GroundTruth>& ground_truths) {
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);

  if (!in_file_pos) { return false; }

  std::string line_pos;

  while(getline(in_file_pos, line_pos)) {
    std::istringstream iss_pos(line_pos);

    GroundTruth ground_truth;

    iss_pos >> ground_truth.x;
    iss_pos >> ground_truth.y;
    iss_pos >> ground_truth.theta;

    ground_truths.push_back(ground_truth);
  }
  return true;
}

/*
 * Reads landmark observation data from a file.
 *
 * @param filename: Name of file containing landmark observation measurements.
 * @return: True if opening and reading file was successful
 */
inline bool readLandmarkData(std::string filename, std::vector<Observation>& observations) {
  std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);

  if (!in_file_obs) { return false; }

  std::string line_obs;

  while(getline(in_file_obs, line_obs)) {
    std::istringstream iss_obs(line_obs);

    Observation observation;

    iss_obs >> observation.x;
    iss_obs >> observation.y;

    observations.push_back(observation);
  }
  return true;
}

#endif /* UTILITIES_H_ */
