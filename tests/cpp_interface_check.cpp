#include <cassert>

#include "s23/coordinates/coordinates.hpp"
#include "s23/estimator/realtime_estimator.hpp"
#include "s23/gnss/ephemeris_provider.hpp"
#include "s23/gnss/observation.hpp"

int main() {
  const s23::Lla origin{0.0, 0.0, 0.0};
  const s23::Ecef ecef = s23::LlaToEcef(origin);
  assert(ecef.x_m > 6378136.0 && ecef.x_m < 6378138.0);

  s23::GnssObservation observation;
  observation.constellation_type = 1;
  observation.svid = 5;
  observation.pseudorange_m = 2.1e7;

  s23::EstimatorConfig config;
  assert(config.output_rate_hz == 100.0);
  return 0;
}
