/*
 * help_functions.h
 */

#ifndef HELP_FUNCTIONS_H_
#define HELP_FUNCTIONS_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "measurement_package.h"


const float PI = std::atan(1.0f)*4;

class Utilities
{
public:

  // Calculate the square of a number
  float squared(float x)
  {
    return x*x;
  }

  //
  // Computer the probability density function of a Normal distribution
  // at value x.
  //
  // @param x: x coordinate.
  // @param mu: mean.
  // @param std: standard deviation.
  // @return: probability density.
  //
  float normalPdf(float x, float mu, float std)
  {
    return std::exp(-0.5f*squared((x-mu)/std))/(std::sqrt(2*PI)*std);
  }

  //
  // Normalize a vector.
  //
  // @param vector_in: Input vector.
  // @return: Normalized vector.
  //
  std::vector<float> normalizeVector(std::vector<float> vector_in)
  {
    float sum = 0.0f;

    //declare and resize output vector
    std::vector<float> vector_out;
    vector_out.resize(vector_in.size());

    //estimate the sum
    for (int i = 0; i < vector_in.size(); ++i)
    {
      sum += vector_in[i];
    }

    //normalize with sum
    for (int i = 0; i < vector_in.size(); ++i)
    {
      vector_out[i] = vector_in[i]/sum;
    }

    return vector_out;
  }

  //
  // Read map data from a file.
  //
  // @param filename: Name of file containing map data.
  //
  inline bool readMapData(std::string filename, Map& map)
  {
    // Get file of map
    std::ifstream in_file_map(filename.c_str(), std::ifstream::in);

    if (!in_file_map)
    {
      return false;
    }

    std::string line_map;

    while(getline(in_file_map, line_map))
    {
      std::istringstream iss_map(line_map);

      Map::Landmark landmark;

      iss_map >> landmark.id;
      iss_map >> landmark.x;

      map.landmark_list_.push_back(landmark);

    }
    return true;
  }

  //
  // Read measurements from a file.
  //
  // @param filename: Name of file containing measurement  data.
  //
  inline bool readMeasurementData(std::string filename_control,
                                  std::string filename_obs,
                                  std::vector<MeasurementPackage>& measurement_pack_list)
  {
    std::ifstream in_file_control(filename_control.c_str(), std::ifstream::in);
    if (!in_file_control)
    {
      return false;
    }

    std::string line;

    int count = 1;

    while(getline(in_file_control, line))
    {
      MeasurementPackage meas_package;

      std::istringstream iss(line);

      iss >> meas_package.control_.delta_x;

      char str_obs[1024];

      sprintf(str_obs,"%sobservations_%06i.txt", filename_obs.c_str(), count);

      std::string in_file_name_observation = std::string(str_obs);

      std::ifstream in_file_observation(in_file_name_observation.c_str(),
                                        std::ifstream::in);
      if (!in_file_observation)
      {
        return false;
      }

      std::string line_obs;

      while(getline(in_file_observation, line_obs))
      {
        std::istringstream iss_obs(line_obs);

        float distance_f;

        iss_obs >> distance_f;

        meas_package.observation_.distance.push_back(distance_f);
      }
      measurement_pack_list.push_back(meas_package);

      count++;
    }
    return true;
  }

  /*
   *
   */
  inline bool compareData(std::string filename_gt,
                          std::vector<float>& result_vec)
  {
    std::vector<float> gt_vec;

    std::ifstream in_file_gt(filename_gt.c_str(), std::ifstream::in);

    std::string line_gt;

    while(getline(in_file_gt, line_gt))
    {
      std::istringstream iss_gt(line_gt);
      float gt_value;

      iss_gt >> gt_value;
      gt_vec.push_back(gt_value);

    }

    float error_sum;
    float belief_sum;
    error_sum  = 0.0f;
    belief_sum = 0.0f;
    std::cout <<"..................................................."<< std::endl;
    std::cout <<"...............----> Results <----................."<< std::endl;
    std::cout <<"..................................................."<< std::endl;

    for (int i = 0; i <  result_vec.size(); ++i)
    {
      error_sum+= (gt_vec[i]-result_vec[i])*(gt_vec[i]-result_vec[i]);
      belief_sum+= result_vec[i] ;
      std::cout << std::fixed << std::setprecision(5) <<"bel_x="<< i <<":" << "\t"
                << result_vec[i]<<"\t"
                << "ground truth:" << "\t"
                << gt_vec[i]<<"\t" << std::endl ;
    }
    std::cout <<"..................................................."<< std::endl;
    std::cout << std::fixed << std::setprecision(5)<< "sum bel:"    << "\t" << belief_sum <<std::endl;
    std::cout <<"..................................................."<< std::endl;
    std::cout << std::fixed << std::setprecision(5)<< "sqrt error sum:     " << "\t" << sqrt((error_sum)) <<std::endl;
    std::cout <<"..................................................."<< std::endl;
    std::cout <<"..................................................."<< std::endl;
    return true;

  }
};

#endif /* HELP_FUNCTIONS_H_ */