#include <cassert>
#include <cmath>

#include "planner_helpers.h"

int main() {
  assert(std::fabs(planner::behaviorCost(50.0, 50.0, false)) < 1e-12);
  assert(std::fabs(planner::behaviorCost(40.0, 50.0, false) - 0.2) < 1e-12);
  assert(std::fabs(planner::behaviorCost(50.0, 50.0, true) - 1.0) < 1e-12);
  assert(std::fabs(planner::behaviorCostAlternative(40.0, 50.0, false) - 0.04) < 1e-12);
  assert(std::fabs(planner::behaviorCostAlternative(50.0, 50.0, true) - 0.5) < 1e-12);
  assert(std::isinf(planner::behaviorCost(-1.0, 50.0, false)));
  assert(std::isinf(planner::behaviorCostAlternative(50.0, 0.0, false)));
  assert(std::isinf(planner::behaviorCostAlternative(50.0, NAN, false)));
  assert(planner::vehicleAheadWithin(100.0, 120.0));
  assert(!planner::vehicleAheadWithin(100.0, 70.0));
  assert(planner::collisionRisk(100.0, 6.0, 120.0, 7.5));
  assert(!planner::collisionRisk(100.0, 6.0, 120.0, 9.0));
  assert(planner::collisionRiskAlternative(100.0, 6.0, 20.0,
                                           120.0, 6.0, 0.0));
  assert(!planner::collisionRiskAlternative(100.0, 6.0, 20.0,
                                            120.0, 6.0, 20.0));
  assert(!planner::collisionRisk(100.0, 6.0, 130.0, 6.0));
  assert(!planner::collisionRisk(100.0, 6.0, 120.0, 8.0));
  assert(!planner::collisionRisk(NAN, 6.0, 120.0, 6.0));
  assert(!planner::collisionRiskAlternative(100.0, 6.0, -1.0,
                                            120.0, 6.0, 0.0));
  assert(!planner::collisionRiskAlternative(100.0, 6.0, 20.0,
                                            120.0, 6.0, 0.0, NAN));
  assert(planner::validLane(0));
  assert(planner::validLane(2));
  assert(!planner::validLane(-1));
  assert(planner::inLane(6.0, 1));
  assert(!planner::inLane(4.0, 1));
  assert(planner::preferredLaneChange(1, true, true) == 0);
  assert(planner::preferredLaneChange(0, true, true) == 1);
  assert(planner::bestLaneChange(1, 45.0, 35.0) == 0);
  assert(planner::bestLaneChange(1, 35.0, 45.0) == 2);
  assert(planner::preferredLaneChange(2, true, true) == 1);
  assert(planner::preferredLaneChange(-1, true, true) == -1);
  assert(planner::bestLaneChange(1, NAN, 40.0) == 1);
  assert(planner::bestLaneChange(1, 40.0, 40.0, 0.0) == 1);
  return 0;
}
