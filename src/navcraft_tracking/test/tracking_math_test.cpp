#include "navcraft_tracking/tracking_math.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

TEST(TrackingMathTest, SignedLateralErrorMatchesStanleyConventionOnXAxisPath) {
  constexpr double path_yaw = 0.0;

  EXPECT_NEAR(navcraft_tracking::SignedLateralError(0.0, 0.0, path_yaw, 0.0, 1.0),
              1.0, 1e-9);
  EXPECT_NEAR(navcraft_tracking::SignedLateralError(0.0, 0.0, path_yaw, 0.0, -1.0),
              -1.0, 1e-9);
}

TEST(TrackingMathTest, SignedLateralErrorRotatesWithPathYaw) {
  constexpr double path_yaw = M_PI_2;

  EXPECT_NEAR(navcraft_tracking::SignedLateralError(0.0, 0.0, path_yaw, -1.0, 0.0),
              1.0, 1e-9);
  EXPECT_NEAR(navcraft_tracking::SignedLateralError(0.0, 0.0, path_yaw, 1.0, 0.0),
              -1.0, 1e-9);
}

TEST(TrackingMathTest, ForwardProjectionSeparatesForwardAndBehindTargets) {
  EXPECT_GT(navcraft_tracking::ForwardProjection(0.0, 1.0, 0.0), 0.0);
  EXPECT_LT(navcraft_tracking::ForwardProjection(0.0, -1.0, 0.0), 0.0);
  EXPECT_GT(navcraft_tracking::ForwardProjection(M_PI_2, 0.0, 1.0), 0.0);
  EXPECT_LT(navcraft_tracking::ForwardProjection(M_PI_2, 0.0, -1.0), 0.0);
}

TEST(TrackingMathTest, IsFinalWaypointReachedRequiresFinalIndexAndTolerance) {
  EXPECT_TRUE(navcraft_tracking::IsFinalWaypointReached(2, 3, 0.04, 0.10));
  EXPECT_FALSE(navcraft_tracking::IsFinalWaypointReached(1, 3, 0.04, 0.10));
  EXPECT_FALSE(navcraft_tracking::IsFinalWaypointReached(2, 3, 0.11, 0.10));
  EXPECT_FALSE(navcraft_tracking::IsFinalWaypointReached(0, 0, 0.0, 0.10));
}

TEST(TrackingMathTest, IsFinalWaypointReachedClampsNegativeToleranceToZero) {
  EXPECT_TRUE(navcraft_tracking::IsFinalWaypointReached(2, 3, 0.0, -0.10));
  EXPECT_FALSE(navcraft_tracking::IsFinalWaypointReached(2, 3, 0.001, -0.10));
}

}  // namespace
