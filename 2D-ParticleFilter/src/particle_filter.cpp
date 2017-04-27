/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>

#include "particle_filter.h"


const double PI = std::atan(1.0)*4;


// constructor
ParticleFilter::ParticleFilter()
{
  is_initialized_ = false;

  num_particles_ = 100;

  // GPS measurement uncertainty: x [m], y [m], theta [rad]
  sigma_gps_ = {0.3, 0.3, 0.01};

  // Landmark measurement uncertainty: x [m], y [m]
  sigma_landmark_ = {0.3, 0.3};
}

// destructor
ParticleFilter::~ParticleFilter() {}

void ParticleFilter::processMeasurement(
    const std::vector<double>& starting_point, const std::vector<Control>& control,
    const std::vector<Observation>& observations, const Map& map, int index)
{
  std::default_random_engine gen;
  std::normal_distribution<double> normal_x(0, sigma_gps_[0]);
  std::normal_distribution<double> normal_y(0, sigma_gps_[1]);
  std::normal_distribution<double> normal_theta(0, sigma_gps_[2]);
  std::normal_distribution<double> normal_observation_x(0, sigma_landmark_[0]);
  std::normal_distribution<double> normal_observation_y(0, sigma_landmark_[1]);

  // Initialize particle filter if this is the first time step.
  if (!is_initialized_)
  {
    for (int i=0; i < num_particles_; ++i)
    {
      // Add Gaussian noise to the initial coordinates
      double new_x = starting_point[0] + normal_x(gen);
      double new_y = starting_point[1] + normal_y(gen);
      double new_theta = starting_point[2] + normal_theta(gen);
      Particle particle{i, new_x, new_y, new_theta, 1.0};
      particles_.push_back(particle);
    }
    is_initialized_ = true;
  }
  else
  {
    // Predict the vehicle's next state (noiseless).
    prediction(control[index-1].velocity, control[index-1].yaw_rate);
  }

  // simulate the addition of noise to noiseless observation data.
  std::vector<Observation> noisy_observations;
  Observation observation;
  for (std::size_t j = 0; j < observations.size(); ++j)
  {
    observation = observations[j];
    observation.x = observation.x + normal_observation_x(gen);
    observation.y = observation.y + normal_observation_y(gen);
    noisy_observations.push_back(observation);
  }

  char buffer1[50];
  std::sprintf(buffer1, "output/prior_particles%06d.txt", index);
  writeParticle(buffer1);

  // Update the weights and resample
  updateWeight(noisy_observations, map);

  resample();

  char buffer2[50];
  std::sprintf(buffer2, "output/particles%06d.txt", index);
  writeParticle(buffer2);

}

void ParticleFilter::prediction(double velocity, double yaw_rate)
{
  // why GPS noise is used in the prediction step? Shouldn't it
  // come from the control noise?
  std::default_random_engine gen;
  std::normal_distribution<double> normal_x(0, sigma_gps_[0]);
  std::normal_distribution<double> normal_y(0, sigma_gps_[1]);
  std::normal_distribution<double> normal_theta(0, sigma_gps_[2]);

  for (size_t i=0; i<particles_.size(); ++i)
  {
    if (std::abs(yaw_rate) > 1.0d-12)
    {
      double v_y = velocity / yaw_rate;
      particles_[i].x += v_y * (sin(particles_[i].theta + yaw_rate*delta_t_)
                               - sin(particles_[i].theta)) + normal_x(gen);
      particles_[i].y += v_y * (cos(particles_[i].theta)
                               - cos(particles_[i].theta
                                     + yaw_rate*delta_t_)) + normal_y(gen);
    }
    else
    {
      particles_[i].x += velocity * cos(particles_[i].theta) * delta_t_ + normal_x(gen);
      particles_[i].y += velocity * sin(particles_[i].theta) * delta_t_ + normal_y(gen);
    }
    particles_[i].theta += yaw_rate*delta_t_ + normal_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<Observation> prediction,
                                     std::vector<Observation>& measurement)
{
  // What's the better algorithm to find the nearest neighbors?
  for (size_t i=0; i<measurement.size(); ++i) {
    double min_dist = sensor_range_*sensor_range_;
    double min_dist_index = 0;
    for (size_t j=0; j<prediction.size(); ++j) {
      double dx = prediction[j].x - measurement[i].x;
      double dy = prediction[j].y - measurement[i].y;

      double current_dist = dx*dx + dy*dy;

      if (j==0)
      {
        min_dist = current_dist;
        min_dist_index = 0;
      }
      else
      {
        if (min_dist > current_dist)
        {
          min_dist = current_dist;
          min_dist_index = j;
        }
      }
    }

    // id will be used to match prediction and observation later
    measurement[i].id = int(min_dist_index);
  }
}

void ParticleFilter::updateWeight(const std::vector<Observation> &observations, const Map &map)
{
  // Should I use the sensor_range here? It could be used in dataAssociation,
  // but with the fixed interface, I can not pass this argument.
  for (size_t i=0; i<particles_.size(); ++i)
  {
    double px = particles_[i].x;
    double py = particles_[i].y;
    double theta = particles_[i].theta;

    std::vector<Observation> prediction;

    for (size_t j=0; j<map.landmark_list.size(); ++j)
    {
      Observation observation;

      // just copy
      observation.id = map.landmark_list[j].id;
      observation.x = map.landmark_list[j].x;
      observation.y = map.landmark_list[j].y;

      prediction.push_back(observation);
    }

    std::vector<Observation> measurement;

    for (size_t j=0; j<observations.size(); ++j)
    {
      double c = std::cos(theta);
      double s = std::sin(theta);
      double ox = observations[j].x;
      double oy = observations[j].y;

      Observation observation;
      observation.id = -1;
      // transform the measurement from the car's coordinate system to
      // the global coordinate system.
      observation.x = px + ox*c - oy*s;
      observation.y = py + ox*s + oy*c;

      measurement.push_back(observation);
    }

    dataAssociation(prediction, measurement);

    particles_[i].weight = 1;
    for (size_t j=0; j<measurement.size(); ++j)
    {
      double Sx = sigma_landmark_[0];
      double Sy = sigma_landmark_[1];
      double c1 = 1/(2*PI*Sx*Sy);
      double dx = prediction[measurement[j].id].x - measurement[j].x;
      double dy = prediction[measurement[j].id].y - measurement[j].y;

      // multi-variant Gaussian distribution
      particles_[i].weight *= c1*std::exp(-0.5*(dx*dx/(Sx*Sx) + dy*dy/(Sy*Sy)));
    }
  }

  // Calculate sum of weights
  double weight_sum = 0.0;
  for (size_t i=0; i<particles_.size(); ++i)
  {
    weight_sum += particles_[i].weight;
  }

  // Handle the situation where the sum of weight is zero.
  if (weight_sum == 0.0)
  {
    char buffer[50];
    std::sprintf(buffer, "Sum of weight is zero!");
    throw std::out_of_range(buffer);
  }
  // Normalize the weight
  for (size_t i=0; i<particles_.size(); ++i)
  {
    particles_[i].weight /= weight_sum;
  }

}

void ParticleFilter::resample()
{
  std::vector<Particle> new_particles;
  std::default_random_engine gen;
  std::uniform_real_distribution<double> d(0, 1.0);

  // Universal resampling algorithm
  double step_size = 1.0/particles_.size();
  double r = d(gen)*step_size;
  double weight_wheel = particles_[0].weight;
  int j=0;
  for (size_t i=0; i < particles_.size(); ++i)
  {
    while (r > weight_wheel)
    {
      j++;
      weight_wheel += particles_[j].weight;
    }

    new_particles.push_back(particles_[j]);

    r += step_size;
  }
  particles_ = new_particles;
}

void ParticleFilter::writeParticle(std::string filename)
{
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (size_t i = 0; i < particles_.size(); ++i)
  {
		dataFile << particles_[i].x << " " << particles_[i].y << " "
             << particles_[i].theta << "\n";
	}
	dataFile.close();
}
