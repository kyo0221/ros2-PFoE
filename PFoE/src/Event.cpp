#include "PFoE/Event.hpp"
#include <sstream>

namespace pfoe
{

Event::Event()
: reward(0), episode_id(0), event_id(0), counter(0)
{
}

Event::Event(Observation obs, Action act, int rw)
: reward(rw), observation(obs), action(act), episode_id(0), event_id(0), counter(0)
{
}

std::string Event::str() const
{
  std::stringstream ss;
  ss << "Event[" << event_id << "]: "
     << "linear_x=" << action.linear_x << ", "
     << "angular_z=" << action.angular_z << ", "
     << "feature_dim=" << observation.feature_dim;
  return ss.str();
}

}  // namespace pfoe
