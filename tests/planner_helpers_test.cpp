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
  return 0;
}
