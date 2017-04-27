/*
 * bayesianFilter.cpp
 */

#include "bayesian_filter.h"
#include <algorithm>


BayesianFilter::BayesianFilter()
{
  is_initialized_ = false;

  control_std_ = 1.0f;

  observation_std_ = 1.0f;

  // Define size of different state vectors.
  bel_x_.resize(100,0);
  bel_x_init_.resize(100,0);
}

BayesianFilter::~BayesianFilter() {}

void BayesianFilter::processMeasurement(const MeasurementPackage &measurements,
                                        const Map &map_1d,
                                        Utilities &utilities)
{
  if(!is_initialized_)
  {
    for (std::size_t i=0; i<map_1d.landmark_list_.size(); ++i){

      Map::Landmark landmark;

      landmark = map_1d.landmark_list_[i];

      // Check if the landmark position is in the range of state vector x
      if(landmark.x > 0 && landmark.x < bel_x_init_.size())
      {
        int position_x = int(landmark.x);

        // Start with a uniform belief distribution around each landmark
        bel_x_init_[position_x]   = 1.0f;
        bel_x_init_[position_x-1] = 1.0f;
        bel_x_init_[position_x+1] = 1.0f;
      }
    }

    bel_x_init_ = utilities.normalizeVector(bel_x_init_);

    is_initialized_ = true;
  }

  // Get current observations and control information
  MeasurementPackage::Control controls = measurements.control_;
  MeasurementPackage::Observation observations = measurements.observation_;

  // Loop over the whole state (index represents the pose in x!)
  for (std::size_t i=0; i<bel_x_.size(); ++i)
  {
    float pose_i = float(i);
    float posterior_motion = 0.0f;

    /*
     * Control update (prediction)
     */

    // Loop over state space x_t-1 (convolution)
    for (std::size_t j=0; j<bel_x_init_.size(); ++j)
    {
      float pose_j = float(j);
      float distance_ij = pose_i - pose_j;

      // Transition probabilities
      float transition_prob = utilities.normalPdf(
          distance_ij, controls.delta_x, control_std_);

      posterior_motion += transition_prob*bel_x_init_[j];
    }

    /*
     * Measurement update (correction)
     */

    // Define pseudo observation vector:
    std::vector<float> pseudo_ranges;

    // Define maximum measurable distance:
    float distance_max = 100;

    // Loop over number of landmarks and estimate pseudo ranges:
    for (std::size_t l=0; l<map_1d.landmark_list_.size(); ++l)
    {
      // Calculate difference between landmark position and current
      // believe state index
      float range_l = map_1d.landmark_list_[l].x - pose_i;

      // Sensor can only measure landmark in front of the car
      // if distances are positive, and store positive range:
      if (range_l > 0.0f)
      {
        pseudo_ranges.push_back(range_l);
      }
    }
    // Sort pseudo range vector in ascending order
    // this should already in ascending order??
    std::sort(pseudo_ranges.begin(), pseudo_ranges.end());

    // Define observation posterior:
    float posterior_obs = 1.0f;

    // The following calculation assumes observations.distance_f is
    // in ascending order.
    for (std::size_t z=0; z<observations.distance.size(); ++z)
    {
      // Define min distance:
      float pseudo_range_min;

      // Check, if distance vector exists:
      if (pseudo_ranges.size() > 0)
      {
        // Set min distance:
        pseudo_range_min = pseudo_ranges[0];
        // Remove this entry from pseudo_ranges-vector:
        pseudo_ranges.erase(pseudo_ranges.begin());
      }
      else
      {
        pseudo_range_min = distance_max;
      }

      // Estimate the posterior for observation model
      // Note that pseudo_range is calculated from the current belief.
      posterior_obs *= utilities.normalPdf(
          observations.distance[z], pseudo_range_min, observation_std_);
    }

    bel_x_[i] = posterior_obs*posterior_motion;
  }

  bel_x_ = utilities.normalizeVector(bel_x_);

  bel_x_init_ = bel_x_;
}