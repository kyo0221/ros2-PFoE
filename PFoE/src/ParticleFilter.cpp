#include "PFoE/ParticleFilter.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace pfoe
{

ParticleFilter::ParticleFilter(int num, Episodes * ep)
: episodes(ep)
{
  double w = 1.0 / num;
  Particle p(w);
  for (int i = 0; i < num; i++) {
    particles.push_back(p);
  }
}

void ParticleFilter::init()
{
  double w = 1.0 / particles.size();
  for (auto & p : particles) {
    p.pos = prob.uniformRandInt(0, episodes->data.size() - 2);
    p.weight = w;
  }
}

void ParticleFilter::print() const
{
  int i = 0;
  for (const auto & p : particles) {
    if (p.pos >= 0 && p.pos < static_cast<int>(episodes->data.size())) {
      const auto & d = episodes->data[p.pos];
      std::cout << d.str() << std::endl;
      i++;
    }
    if (i == 10) {
      return;
    }
  }
}

Action ParticleFilter::modeParticle(Episodes * ep)
{
  double fw = 0.0;
  double rot = 0.0;
  double max = 0.0;
  std::cout << "mode particle" << std::endl;

  for (const auto & p : particles) {
    auto * e = ep->actionAt(p.pos);
    if (e && max < p.weight) {
      max = p.weight;
      fw = e->linear_x;
      rot = e->angular_z;
    }
  }

  Action a;
  a.linear_x = fw;
  a.angular_z = rot;
  return a;
}

Action ParticleFilter::mode(Episodes * ep)
{
  // Reset counters
  for (auto & p : particles) {
    auto * e = ep->At(p.pos);
    if (e) {
      e->counter = 0;
    }
  }

  // Count particles at each position
  for (const auto & p : particles) {
    auto * e = ep->At(p.pos);
    if (e) {
      e->counter++;
    }
  }

  // Find mode (most frequent position)
  int max = 0;
  Action * mode_a = nullptr;
  for (const auto & p : particles) {
    auto * e = ep->At(p.pos);
    if (e && e->counter > max) {
      max = e->counter;
      mode_a = ep->actionAt(p.pos + 1);  // Get action from next step
    }
    if (e) {
      e->counter = 0;
    }
  }

  Action a;
  if (mode_a) {
    a.linear_x = mode_a->linear_x;
    a.angular_z = mode_a->angular_z;
  } else {
    a.linear_x = 0.0;
    a.angular_z = 0.0;
  }
  return a;
}

Action ParticleFilter::average(Episodes * ep)
{
  double fw = 0.0;
  double rot = 0.0;
  std::cout << "avg" << std::endl;

  for (const auto & p : particles) {
    auto * e = ep->actionAt(p.pos + 1);
    if (e) {
      fw += p.weight * e->linear_x;
      rot += p.weight * e->angular_z;
    }
  }

  Action a;
  a.linear_x = fw;
  a.angular_z = rot;
  return a;
}

Action ParticleFilter::sensorUpdate(
  Observation * obs, Action * act, Episodes * ep,
  pfoe_msg::msg::PfoeOutput * out)
{
  out->eta = 0.0;
  std::cout << "obs likelihood" << std::endl;

  // Update particle weights based on observation likelihood
  for (auto & p : particles) {
    double h = likelihood(episodes->obsAt(p.pos), obs);
    p.weight *= h;
    out->eta += p.weight;
  }

  normalize();
  resampling(&particles);

  // Collect particle positions for output
  for (const auto & p : particles) {
    out->particles_pos.push_back(static_cast<uint32_t>(p.pos));
  }

  std::cout << "mode" << std::endl;
  return mode(ep);
}

double ParticleFilter::likelihood(Observation * past, Observation * last)
{
  if (!past || !last) {
    return 1e-6;
  }

  // Compute distance between feature vectors
  double dist = past->distance(*last);

  // Convert distance to likelihood
  // Using exponential decay: likelihood = exp(-dist / sigma)
  // Or inverse relationship: likelihood = 1 / (1 + dist)
  double ans = 1.0 / (1.0 + dist);

  return ans;
}

double ParticleFilter::likelihood(
  Observation * past, Observation * last,
  Action * past_a, Action * last_a)
{
  if (!past || !last || !past_a || !last_a) {
    return 1e-6;
  }

  double dist = past->distance(*last);
  double ans = 1.0 / (1.0 + dist);

  // Consider action similarity as well
  ans /= (1.0 + 0.2 * std::fabs(past_a->linear_x - last_a->linear_x));
  ans /= (1.0 + 0.2 * std::fabs(past_a->angular_z - last_a->angular_z));

  return ans;
}

void ParticleFilter::resampling(std::vector<Particle> * ps)
{
  std::vector<Particle> prev;
  std::shuffle(ps->begin(), ps->end(), std::mt19937());

  double sumweight = 0.0;
  int num = static_cast<int>(ps->size());

  for (int i = 0; i < num; i++) {
    ps->at(i).weight += sumweight;
    sumweight = ps->at(i).weight;
    prev.push_back(ps->at(i));
  }

  double step = sumweight / num;
  int * choice = new int[num];
  double accum = step * prob.uniformRand(0.0, 0.999999999);
  int j = 0;

  for (int i = 0; i < num; i++) {
    while (prev[j].weight <= accum) {
      j++;
    }

    if (j == num) {
      j--;
    }

    accum += step;
    choice[i] = j;
  }

  for (int i = 0; i < num; i++) {
    int j = choice[i];
    ps->at(i) = prev[j];
    ps->at(i).weight = 1.0 / num;
  }

  delete[] choice;
}

void ParticleFilter::normalize()
{
  double eta = 0.0;
  for (const auto & p : particles) {
    eta += p.weight;
  }

  std::cout << "eta: " << eta << std::endl;

  if (eta > 1e-9) {
    for (auto & p : particles) {
      p.weight /= eta;
    }
  }
}

void ParticleFilter::motionUpdate(Episodes * ep)
{
  std::cout << "odom" << std::endl;

  for (auto & p : particles) {
    // 10% chance of random jump (global localization)
    if (rand() % 10 == 0) {
      p.pos = prob.uniformRandInt(0, episodes->data.size() - 2);
      continue;
    }

    // Random forward motion (1 or 2 steps)
    int r = rand() % 3;
    if (r == 0) {
      p.pos++;
    } else if (r == 1) {
      p.pos += 2;
    }

    // Reset if out of bounds
    if (p.pos >= static_cast<int>(ep->data.size()) - 1) {
      p.pos = prob.uniformRandInt(0, episodes->data.size() - 2);
    }
  }
}

}  // namespace pfoe
