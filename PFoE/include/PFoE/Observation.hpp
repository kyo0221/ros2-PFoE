#ifndef PARTICLE_FILTER_ON_EPISODE_ROS2__OBSERVATION_HPP_
#define PARTICLE_FILTER_ON_EPISODE_ROS2__OBSERVATION_HPP_

#include <vector>
#include <cmath>

namespace pfoe
{

class Observation
{
public:
  Observation();
  Observation(const std::vector<float> & feature_vector);

  void setValues(const std::vector<float> & feature_vector);

  std::vector<float> feature;  // Image embedding vector
  size_t feature_dim;           // Dimension of feature vector

  // Compute distance between two observations
  double distance(const Observation & other) const;
};

}  // namespace pfoe

#endif  // PARTICLE_FILTER_ON_EPISODE_ROS2__OBSERVATION_HPP_
