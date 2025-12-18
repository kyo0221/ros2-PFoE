#ifndef PARTICLE_FILTER_ON_EPISODE_ROS2__PROB_DISTRIBUTION_HPP_
#define PARTICLE_FILTER_ON_EPISODE_ROS2__PROB_DISTRIBUTION_HPP_

#include <random>

namespace pfoe
{

class ProbDistributions
{
public:
  ProbDistributions()
  {
    rd = new std::random_device();
    gen = new std::mt19937((*rd)());
  }

  ~ProbDistributions()
  {
    delete gen;
    delete rd;
  }

  double normalRand(double mean, double stddev)
  {
    std::normal_distribution<> nd(mean, stddev);
    return nd(*gen);
  }

  double uniformRand(double min, double max)
  {
    std::uniform_real_distribution<> ud(min, max);
    return ud(*gen);
  }

  int uniformRandInt(int min, int max)
  {
    std::uniform_int_distribution<> ud(min, max);
    return ud(*gen);
  }

private:
  std::random_device * rd;
  std::mt19937 * gen;
};

}  // namespace pfoe

#endif  // PARTICLE_FILTER_ON_EPISODE_ROS2__PROB_DISTRIBUTION_HPP_
