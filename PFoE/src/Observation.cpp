#include "PFoE/Observation.hpp"
#include <numeric>

namespace pfoe
{

Observation::Observation()
: feature_dim(0)
{
}

Observation::Observation(const std::vector<float> & feature_vector)
: feature(feature_vector), feature_dim(feature_vector.size())
{
}

void Observation::setValues(const std::vector<float> & feature_vector)
{
  feature = feature_vector;
  feature_dim = feature_vector.size();
}

double Observation::distance(const Observation & other) const
{
  if (feature_dim != other.feature_dim || feature_dim == 0) {
    return 1e9;  // Invalid distance
  }

  // Compute cosine distance: distance = sqrt(2 - 2*dot_product)
  // Assumes feature vectors are already L2-normalized
  double dot_product = 0.0;
  for (size_t i = 0; i < feature_dim; ++i) {
    dot_product += static_cast<double>(feature[i]) * static_cast<double>(other.feature[i]);
  }

  double distance = std::sqrt(std::max(0.0, 2.0 - 2.0 * dot_product));
  return distance;
}

}  // namespace pfoe
