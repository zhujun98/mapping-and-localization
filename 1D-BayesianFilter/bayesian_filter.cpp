/*
 * bayesianFilter.cpp
 */

#include "bayesian_filter.h"
#include <algorithm>


//constructor
BayesianFilter::BayesianFilter() {

  is_initialized_ = false;

  control_std_ = 1.0f;

  observation_std_ = 1.0f;

  //define size of different state vectors:
  bel_x_.resize(100,0);
  bel_x_init_.resize(100,0);

}

//destructor
BayesianFilter::~BayesianFilter() {}

void BayesianFilter::process_measurement(const MeasurementPackage &measurements,
                                         const map &map_1d,
                                         help_functions &helpers){

  if(!is_initialized_){
    for (std::size_t i=0; i<map_1d.landmark_list_.size(); ++i){

      map::single_landmark_s landmark_temp;

      landmark_temp = map_1d.landmark_list_[i];

      //check if the landmark position is in the range of state vector x
      if(landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init_.size() ){

        int position_x = int(landmark_temp.x_f);

        //start with a uniform belief distribution around each landmark
        bel_x_init_[position_x]   = 1.0f;
        bel_x_init_[position_x-1] = 1.0f;
        bel_x_init_[position_x+1] = 1.0f;
      }
    }

    bel_x_init_ = helpers.normalize_vector(bel_x_init_);

    is_initialized_ = true;
  }

  //get current observations and control information
  MeasurementPackage::control_s controls = measurements.control_s_;
  MeasurementPackage::observation_s observations = measurements.observation_s_;

  //loop over the whole state (index represents the pose in x!)
  for (std::size_t i=0; i<bel_x_.size(); ++i){

    float pose_i = float(i);
    float posterior_motion = 0.0f;

    /*
     * Control update (prediction)
     */

    //loop over state space x_t-1 (convolution)
    for (std::size_t j=0; j<bel_x_init_.size(); ++j){
      float pose_j = float(j);
      float distance_ij = pose_i - pose_j;

      //transition probabilities
      float transition_prob = helpers.normpdf(
          distance_ij, controls.delta_x_f, control_std_);

      posterior_motion += transition_prob*bel_x_init_[j];
    }

    /*
     * Measurement update (correction)
     */

    //define pseudo observation vector:
    std::vector<float> pseudo_ranges;

    //define maximum measurable distance:
    float distance_max = 100;

    //loop over number of landmarks and estimate pseudo ranges:
    for (std::size_t l=0; l<map_1d.landmark_list_.size(); ++l) {

      //calculate difference between landmark position and current
      //believe state index
      float range_l = map_1d.landmark_list_[l].x_f - pose_i;

      //sensor can only measure landmark in front of the car
      //if distances are positive, and store positive range:
      if (range_l > 0.0f) {
        pseudo_ranges.push_back(range_l);
      }
    }
    //sort pseudo range vector in ascending order
    //this should already in ascending order??
    std::sort(pseudo_ranges.begin(), pseudo_ranges.end());

    //define observation posterior:
    float posterior_obs = 1.0f;

    // The following calculation assumes observations.distance_f is
    // in ascending order.
    for (std::size_t z=0; z<observations.distance_f.size(); ++z){
      //define min distance:
      float pseudo_range_min;

      //check, if distance vector exists:
      if (pseudo_ranges.size() > 0) {
        //set min distance:
        pseudo_range_min = pseudo_ranges[0];
        //remove this entry from pseudo_ranges-vector:
        pseudo_ranges.erase(pseudo_ranges.begin());
      } else {
        pseudo_range_min = distance_max;
      }

      //estimate the posterior for observation model
      //Note that pseudo_range is calculated from the current belief.
      posterior_obs *= helpers.normpdf(observations.distance_f[z],
                                      pseudo_range_min,
                                      observation_std_);
    }

    bel_x_[i] = posterior_obs*posterior_motion;
  }

  bel_x_ = helpers.normalize_vector(bel_x_);

  bel_x_init_ = bel_x_;
}