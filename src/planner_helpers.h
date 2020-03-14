#ifndef PLANNER_HELPERS_H
#define PLANNER_HELPERS_H

#include <algorithm>
#include <cmath>
#include <limits>

namespace planner {

constexpr int kLaneCount = 3;
constexpr double kLaneWidth = 4.0;
constexpr double kLaneCenterOffset = 2.0;

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

inline bool collisionRiskAlternative(double ego_s, double ego_d,
                                     double ego_speed, double other_s,
                                     double other_d, double other_speed,
                                     double horizon = 1.0,
                                     double longitudinal_range = 5.0,
                                     double lateral_range = 2.0) {
  if (!std::isfinite(ego_speed) || !std::isfinite(other_speed) ||
      ego_speed < 0.0 || other_speed < 0.0 || !std::isfinite(horizon) ||
      horizon < 0.0) {
    return false;
  }

  return collisionRisk(ego_s + ego_speed * horizon, ego_d,
                       other_s + other_speed * horizon, other_d,
                       longitudinal_range, lateral_range);
}

inline bool validLane(int lane) {
  return lane >= 0 && lane < kLaneCount;
}

inline double laneCenter(int lane) {
  return validLane(lane)
             ? kLaneCenterOffset + kLaneWidth * lane
             : std::numeric_limits<double>::quiet_NaN();
}

inline bool inLane(double d, int lane) {
  const double center = laneCenter(lane);
  return std::isfinite(d) && std::isfinite(center) &&
         d > center - kLaneWidth / 2.0 && d < center + kLaneWidth / 2.0;
}

inline bool canChangeLane(int current_lane, int target_lane,
                          bool target_free) {
  return validLane(current_lane) && validLane(target_lane) &&
         std::abs(target_lane - current_lane) == 1 && target_free;
}

inline int preferredLaneChange(int current_lane, bool left_free,
                               bool right_free) {
  if (!validLane(current_lane)) {
    return current_lane;
  }
  if (canChangeLane(current_lane, current_lane - 1, left_free)) {
    return current_lane - 1;
  }
  if (canChangeLane(current_lane, current_lane + 1, right_free)) {
    return current_lane + 1;
  }
  return current_lane;
}

}  // namespace planner

#endif  // PLANNER_HELPERS_H
