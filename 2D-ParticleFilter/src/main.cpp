/*
 * main.cpp
 *
 * Reads in data and runs 2D particle filter.
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>

#include "particle_filter.h"


int main() {
  // Start timer.
  clock_t start = std::clock();

  // Read map data
  Map map;
  map.readData("data/map_data.txt");

  // Read position data
  std::vector<Control> control_feedback;
  if (!readControlData("data/control_data.txt", control_feedback)) {
    std::cout << "Error: Could not open control_feedback data file"
              << std::endl;
    return -1;
  }

  // Read ground truth data
  std::vector<GroundTruth> ground_truth;
  if (!readGroundTruthData("data/gt_data.txt", ground_truth)) {
    std::cout << "Error: Could not open ground truth data file" << std::endl;
    return -1;
  }

  std::size_t num_time_steps = control_feedback.size();
  ParticleFilter particle_filter;

  // Precise starting point of the car
  std::vector<double> starting_point = {
      ground_truth[0].x,
      ground_truth[0].y,
      ground_truth[0].theta
  };

  std::vector<double> total_error = {0, 0, 0};
  std::vector<double> cumulated_mean_error = {0, 0, 0};

  for (int i = 0; i < num_time_steps; ++i) {
    std::cout << "Time step: " << i << std::endl;
    // Read in landmark observations for current time step.
    std::ostringstream file;
    file << "data/observation/observations_" << std::setfill('0')
         << std::setw(6) << i+1 << ".txt";
    std::vector<Observation> observations;
    if (!readLandmarkData(file.str(), observations)) {
      std::cout << "Error: Could not open observation file " << i+1 << std::endl;
      return -1;
    }

    // Run particle filter at this time step.
    particle_filter.processMeasurement(
        starting_point, control_feedback, observations, map, i);

    // Calculate and output the average weighted error of the particle
    // filter over all time steps so far.
    double highest_weight = 0.0;
    Particle best_particle;
    for (size_t j=0; j < particle_filter.particles_.size(); ++j) {
      if (particle_filter.particles_[j].weight > highest_weight) {
        highest_weight = particle_filter.particles_[j].weight;
        best_particle = particle_filter.particles_[j];
      }
    }

    double *avg_error = getError(ground_truth[i].x, ground_truth[i].y,
                                 ground_truth[i].theta,
                                 best_particle.x, best_particle.y,
                                 best_particle.theta);

    for (int k = 0; k < 3; ++k) {
      total_error[k] += avg_error[k];
      cumulated_mean_error[k] = total_error[k] / (double)(i + 1);
    }

    // Print the cumulative weighted error
    std::cout << "Cumulative mean weighted error: x " << cumulated_mean_error[0]
              << " y " << cumulated_mean_error[1]
              << " yaw " << cumulated_mean_error[2]
              << std::endl;
  }

  // Output the runtime for the filter.
  clock_t stop = clock();
  double runtime = (stop - start) / double(CLOCKS_PER_SEC);
  std::cout << "Runtime (sec): " << runtime << std::endl;

  return 0;
}


