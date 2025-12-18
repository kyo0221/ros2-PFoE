#ifndef PARTICLE_FILTER_ON_EPISODE_ROS2__PARTICLE_HPP_
#define PARTICLE_FILTER_ON_EPISODE_ROS2__PARTICLE_HPP_

namespace pfoe
{

class Particle
{
public:
  explicit Particle(double w);
  virtual ~Particle() {}

  double weight;
  int pos;  // Position in episode timeline
};

}  // namespace pfoe

#endif  // PARTICLE_FILTER_ON_EPISODE_ROS2__PARTICLE_HPP_
