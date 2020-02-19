#ifndef PLANNER_HELPERS_H
#define PLANNER_HELPERS_H

#include <algorithm>
#include <cmath>
#include <limits>

namespace planner {

inline bool validBehaviorInputs(double speed_mph, double speed_limit_mph) {
  return std::isfinite(speed_mph) && std::isfinite(speed_limit_mph) &&
         speed_mph >= 0.0 && speed_limit_mph > 0.0;
}

inline double behaviorCost(double speed_mph, double speed_limit_mph,
                           bool blocked) {
  if (!validBehaviorInputs(speed_mph, speed_limit_mph)) {
    return std::numeric_limits<double>::infinity();
  }

  const double deficit =
      std::max(0.0, speed_limit_mph - speed_mph) / speed_limit_mph;
  return deficit + (blocked ? 1.0 : 0.0);
}

inline double behaviorCostAlternative(double speed_mph, double speed_limit_mph,
                                      bool blocked) {
  if (!validBehaviorInputs(speed_mph, speed_limit_mph)) {
    return std::numeric_limits<double>::infinity();
  }

  const double deficit =
      std::max(0.0, speed_limit_mph - speed_mph) / speed_limit_mph;
  return deficit * deficit + (blocked ? 0.5 : 0.0);
}

}  // namespace planner

#endif  // PLANNER_HELPERS_H
