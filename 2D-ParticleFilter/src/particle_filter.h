/*
 * particle_filter.h
 *
 * 2D particle filter class.
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "utilities.h"


struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
};


class ParticleFilter {
public:
  // Constructor
  ParticleFilter();

  // Destructor
  ~ParticleFilter();

  std::vector<Particle> particles_;

  void processMeasurement(const std::vector<double>& starting_point,
                          const std::vector<Control>& control,
                          const std::vector<Observation>& observations,
                          const Map& map,
                          int index);

private:

  bool is_initialized_; // Flag, if filter is initialized

  int num_particles_; // Number of particles

  double delta_t_ = 0.1; // Time elapsed between measurements [sec]

  double sensor_range_ = 50; // Sensor range [m]

  //
  // Here 'sigma_xxx' is only an estimation. The noise usually comes from
  // the uncertainty of a sensor, but data from multiple sensors are used,
  // it's difficult to find these uncertainties directly.
  //

  // GPS measurement uncertainty: x [m], y [m], theta [rad]
  std::vector<double> sigma_gps_;
  // Landmark measurement uncertainty: x [m], y [m]
  std::vector<double> sigma_landmark_;

  //
  // Predicts the state for the next time step using the process model.
  //
  // @param velocity Velocity of car from t to t+1 [m/s]
  // @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
  //
  void prediction(double velocity, double yaw_rate);

  //
  // Finds which observations correspond to which landmarks (likely by using
  // a nearest-neighbors data association).
  //
  // @param prediction Vector of predicted landmark observations
  // @param measurement Vector of landmark observations
  //
  void dataAssociation(std::vector<Observation> prediction,
                       std::vector<Observation>& measurement);

  //
  // Updates the weights for each particle based on the likelihood of the
  // observed measurements.

  // @param observations Vector of landmark observations
  // @param map Map class containing map landmarks
   //
  void updateWeight(const std::vector<Observation>& observations, const Map& map);

  //
  // Resample from the updated set of particles to form the new set of particles.
  //
  void resample();

  //
  // write Writes particle positions to a file.
  // @param filename File to write particle positions to.
  //
  void writeParticle(std::string filename);
};

#endif /* PARTICLE_FILTER_H_ */
