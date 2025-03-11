/**
 * @file EKF_Localization_known_correspondences.cpp
 * @author Mauricio Bittencourt Pimenta
 * @brief Implementation of the EKF (Extended Kalman Filter) Localization with known correspondences.
 * This code is based on the algorithm presented in page 204, table 7.2 of the book 'Probabilistic Robotics'
 * by Sebastian Thrun, Wolfram Burgard, and Dieter Fox.
 * @version 0.1
 * @date 2024-04-23
 */

#include <iostream>
#include <string>
#include <list>
#include <array>
#include <vector>

class ExtendedKalmanFilter
{
public:
  /* Public Atributes */
    //std::string dataset;  // File path to the dataset to use
    int end_frame;        // Last frame of the dataset
    std::array<double, 2> R;  // State Covariance Matrix
    std::array<double, 2> Q;  // Measurement Covariance Matrix


  /* Public Methods */
  ExtendedKalmanFilter(std::string dataset, int end_frame, std::array<double,2> R, std::array<double,2> Q){
    this->load_data(dataset, end_frame);
  }
  ~ExtendedKalmanFilter();

private:
  /* Private Atributes */

  /* Private Methods */
  void load_data(std::string dataset, int end_frame){

  }
};
