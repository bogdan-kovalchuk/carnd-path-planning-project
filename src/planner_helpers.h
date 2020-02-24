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

inline bool vehicleAheadWithin(double ego_s, double other_s,
                               double max_gap = 30.0) {
  return std::isfinite(ego_s) && std::isfinite(other_s) &&
         std::isfinite(max_gap) && max_gap > 0.0 && other_s > ego_s &&
         other_s - ego_s < max_gap;
}

inline bool collisionRisk(double ego_s, double ego_d, double other_s,
                          double other_d, double longitudinal_range = 30.0,
                          double lateral_range = 2.0) {
  return std::isfinite(ego_s) && std::isfinite(ego_d) &&
         std::isfinite(other_s) && std::isfinite(other_d) &&
         std::isfinite(longitudinal_range) && std::isfinite(lateral_range) &&
         longitudinal_range > 0.0 && lateral_range > 0.0 &&
         std::fabs(ego_s - other_s) < longitudinal_range &&
         std::fabs(ego_d - other_d) < lateral_range;
}

}  // namespace planner

#endif  // PLANNER_HELPERS_H
