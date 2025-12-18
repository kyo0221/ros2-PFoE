#ifndef PFOE__EVENT_HPP_
#define PFOE__EVENT_HPP_

#include <string>
#include <vector>
#include "PFoE/Observation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pfoe
{

struct Action
{
  double linear_x;
  double angular_z;
};

class Event
{
public:
  Event(Observation obs, Action act, int rw);
  Event();

  int reward;
  Observation observation;
  Action action;
  rclcpp::Time time;

  std::string str() const;

  int episode_id;
  int event_id;
  int counter;  // Used for mode calculation in ParticleFilter
};

}  // namespace pfoe

#endif  // PFOE__EVENT_HPP_
