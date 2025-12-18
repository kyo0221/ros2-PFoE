#ifndef PFOE__PARTICLE_FILTER_HPP_
#define PFOE__PARTICLE_FILTER_HPP_

#include <vector>
#include "PFoE/Particle.hpp"
#include "PFoE/ProbDistribution.hpp"
#include "PFoE/Episodes.hpp"
#include "PFoE/Event.hpp"
#include "pfoe_msg/msg/pfoe_output.hpp"

namespace pfoe
{

class ParticleFilter
{
public:
  ParticleFilter(int num, Episodes * ep);

  void init();
  void print() const;

  Action sensorUpdate(
    Observation * obs, Action * act, Episodes * ep,
    pfoe_msg::msg::PfoeOutput * out);

  Action mode(Episodes * ep);
  Action modeParticle(Episodes * ep);
  Action average(Episodes * ep);

  void motionUpdate(Episodes * ep);

private:
  std::vector<Particle> particles;
  ProbDistributions prob;

  Episodes * episodes;

  double likelihood(Observation * past, Observation * last);
  double likelihood(Observation * past, Observation * last, Action * past_a, Action * last_a);

  void resampling(std::vector<Particle> * ps);
  void normalize();
};

}  // namespace pfoe

#endif  // PFOE__PARTICLE_FILTER_HPP_
