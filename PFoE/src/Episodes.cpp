#include "PFoE/Episodes.hpp"
#include <iostream>
#include <fstream>

namespace pfoe
{

Episodes::Episodes()
: current_event_id(0)
{
}

void Episodes::append(Event e)
{
  e.event_id = current_event_id++;
  data.push_back(e);
}

void Episodes::print(std::string filename) const
{
  if (filename.empty()) {
    for (const auto & e : data) {
      std::cout << e.str() << std::endl;
    }
  } else {
    std::ofstream ofs(filename);
    for (const auto & e : data) {
      ofs << e.str() << std::endl;
    }
  }
}

void Episodes::reset()
{
  data.clear();
  current_event_id = 0;
}

Event * Episodes::At(int pos)
{
  if (pos < 0 || pos >= static_cast<int>(data.size())) {
    return nullptr;
  }
  return &data[pos];
}

Observation * Episodes::obsAt(int pos)
{
  if (pos < 0 || pos >= static_cast<int>(data.size())) {
    return nullptr;
  }
  return &data[pos].observation;
}

Action * Episodes::actionAt(int pos)
{
  if (pos < 0 || pos >= static_cast<int>(data.size())) {
    return nullptr;
  }
  return &data[pos].action;
}

}  // namespace pfoe
