#include <gtest/gtest.h>

#include "recovery_supervisor.h"

TEST(EstimatorRecoverySupervisorTest, RequiresThreeInvalidAndThreeValidationFrames)
{
  EstimatorRecoverySupervisor supervisor;
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::HEALTHY);
  EXPECT_FALSE(supervisor.formalMapFrozen());

  supervisor.onInvalidFrame(false);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::DEGRADED);
  EXPECT_TRUE(supervisor.formalMapFrozen());
  supervisor.onInvalidFrame(false);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::DEGRADED);
  supervisor.onInvalidFrame(false);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
  EXPECT_EQ(supervisor.attempts, 0);

  EXPECT_TRUE(supervisor.beginLocalReinitialization());
  EXPECT_EQ(supervisor.attempts, 1);
  EXPECT_EQ(supervisor.rollback_count, 1);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::VALIDATING);
  EXPECT_EQ(supervisor.validation_streak, 1);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::VALIDATING);
  EXPECT_EQ(supervisor.validation_streak, 2);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::HEALTHY);
  EXPECT_EQ(supervisor.success_count, 1);
  EXPECT_EQ(supervisor.attempts, 0);
  EXPECT_FALSE(supervisor.formalMapFrozen());
}

TEST(EstimatorRecoverySupervisorTest, CovarianceRetryBudgetDoesNotPreemptDataRecovery)
{
  EstimatorRecoverySupervisor supervisor;
  supervisor.trigger_frames = 1;
  supervisor.max_attempts = 2;

  supervisor.onInvalidFrame(true);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
  EXPECT_EQ(supervisor.attempts, 0);
  EXPECT_TRUE(supervisor.beginLocalReinitialization());
  EXPECT_EQ(supervisor.attempts, 1);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::VALIDATING);

  supervisor.onInvalidFrame(false);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
  EXPECT_EQ(supervisor.attempts, 1);
  EXPECT_TRUE(supervisor.beginLocalReinitialization());
  EXPECT_EQ(supervisor.attempts, 2);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::VALIDATING);

  supervisor.onInvalidFrame(false);
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
  EXPECT_FALSE(supervisor.beginLocalReinitialization());
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
  EXPECT_FALSE(supervisor.terminal());
  EXPECT_TRUE(supervisor.formalMapFrozen());
  EXPECT_EQ(supervisor.failure_count, 0);

  supervisor.onValidFrame();
  supervisor.onValidFrame();
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::HEALTHY);
  EXPECT_EQ(supervisor.success_count, 1);
}

TEST(EstimatorRecoverySupervisorTest, SustainedInvalidEpisodeIsExplicitlyBounded)
{
  EstimatorRecoverySupervisor supervisor;
  supervisor.trigger_frames = 1;
  supervisor.max_degraded_frames = 5;
  for (int frame = 0; frame < 4; ++frame) supervisor.onInvalidFrame(false);
  EXPECT_FALSE(supervisor.terminal());
  supervisor.onInvalidFrame(false);
  EXPECT_TRUE(supervisor.terminal());
  EXPECT_EQ(supervisor.failure_count, 1);
}

TEST(EstimatorRecoverySupervisorTest, QualifiedRelativeConstraintsKeepRecoveryFiniteButAlive)
{
  EstimatorRecoverySupervisor supervisor;
  supervisor.trigger_frames = 1;
  supervisor.max_degraded_frames = 5;
  for (int frame = 0; frame < 20; ++frame)
  {
    supervisor.onInvalidFrame(false);
    supervisor.onRecoveryConstraintFrame();
    EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::RECOVERING);
    EXPECT_FALSE(supervisor.terminal());
    EXPECT_TRUE(supervisor.formalMapFrozen());
    EXPECT_EQ(supervisor.degraded_frames, 0);
  }
}

TEST(EstimatorRecoverySupervisorTest, SingleTransientFrameNeedsNoRecoverySuccess)
{
  EstimatorRecoverySupervisor supervisor;
  supervisor.onInvalidFrame(false);
  supervisor.onValidFrame();
  EXPECT_EQ(supervisor.state, EstimatorRecoverySupervisor::HEALTHY);
  EXPECT_EQ(supervisor.success_count, 0);
  EXPECT_EQ(supervisor.rollback_count, 0);
}

TEST(EstimatorRecoverySupervisorTest, ExplicitTerminalFailureIsIdempotent)
{
  EstimatorRecoverySupervisor supervisor;
  supervisor.failUnrecoverable();
  supervisor.failUnrecoverable();
  EXPECT_TRUE(supervisor.terminal());
  EXPECT_EQ(supervisor.failure_count, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
