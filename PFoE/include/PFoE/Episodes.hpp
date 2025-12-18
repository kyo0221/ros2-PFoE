#ifndef PFOE__EPISODES_HPP_
#define PFOE__EPISODES_HPP_

#include <vector>
#include <string>
#include "PFoE/Event.hpp"

namespace pfoe
{

class Episodes
{
public:
  Episodes();

  std::vector<Event> data;

  void append(Event e);
  void print(std::string filename = "") const;
  void reset();

  Event * At(int pos);
  Observation * obsAt(int pos);
  Action * actionAt(int pos);

private:
  int current_event_id;
};

}  // namespace pfoe

#endif  // PFOE__EPISODES_HPP_
