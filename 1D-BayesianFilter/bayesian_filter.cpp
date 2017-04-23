/*
 * bayesianFilter.cpp
 */

#include "bayesian_filter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor
BayesianFilter::BayesianFilter() {

  //set initialization to false:
  is_initialized_ = false;

  //set standard deviation of control:
  control_std_ = 1.0f;

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
    for (int i=0; i<map_1d.landmark_list_.size(); ++i){

      map::single_landmark_s landmark_temp;

      landmark_temp = map_1d.landmark_list_[i];

      //check, if landmark position is in the range of state vector x
      if(landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init_.size() ){

        int position_x = int(landmark_temp.x_f);
        cout << position_x << endl;

        bel_x_init_[position_x]   = 1.0f;
        bel_x_init_[position_x-1] = 1.0f;
        bel_x_init_[position_x+1] = 1.0f;
      }
    }

    bel_x_init_ = helpers.normalize_vector(bel_x_init_);

    is_initialized_ = true;
  }

  /******************************************************************************
   *  motion model and observation update
  ******************************************************************************/
  std::cout <<"-->motion model for state x ! \n" << std::endl;

  //get current observations and control information
  MeasurementPackage::control_s     controls = measurements.control_s_;
  MeasurementPackage::observation_s observations = measurements.observation_s_;

  //run over the whole state (index represents the pose in x!)
  for (int i=0; i< bel_x_.size(); ++i){

    float pose_i = float(i);
    /**************************************************************************
     *  posterior for motion model
    **************************************************************************/

    // motion posterior
    float posterior_motion = 0.0f;

    //loop over state space x_t-1 (convolution)
    for (int j=0; j< bel_x_.size(); ++j){
      float pose_j = float(j);

      float distance_ij = pose_i-pose_j;

      //transition probabilities
      float transition_prob = helpers.normpdf(distance_ij,
                                              controls.delta_x_f,
                                              control_std_);
      //motion model
      posterior_motion += transition_prob*bel_x_init_[j];
    }

    //update = motion_model
    bel_x_[i] = posterior_motion;

  };
  //normalize
  bel_x_ = helpers.normalize_vector(bel_x_);

  bel_x_init_ = bel_x_;
};