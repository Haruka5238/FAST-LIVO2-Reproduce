#ifndef FAST_LIVO_RECOVERY_SUPERVISOR_H
#define FAST_LIVO_RECOVERY_SUPERVISOR_H

#include <algorithm>

// A finite estimator-level recovery protocol.  It owns no estimator data; the
// mapper uses it to decide when the formal map must remain immutable and when a
// locally reinitialized state has accumulated enough independent valid frames
// to be trusted again.
struct EstimatorRecoverySupervisor
{
  enum State
  {
    HEALTHY = 0,
    DEGRADED = 1,
    RECOVERING = 2,
    VALIDATING = 3,
    UNRECOVERABLE = 4
  };

  State state = HEALTHY;
  int trigger_frames = 3;
  int validation_frames = 3;
  int max_attempts = 6;
  int max_degraded_frames = 300;
  int degraded_frames = 0;
  int validation_streak = 0;
  int attempts = 0;
  int rollback_count = 0;
  int success_count = 0;
  int failure_count = 0;

  void reset()
  {
    state = HEALTHY;
    degraded_frames = 0;
    validation_streak = 0;
    attempts = 0;
    rollback_count = 0;
    success_count = 0;
    failure_count = 0;
  }

  bool formalMapFrozen() const { return state != HEALTHY; }
  bool terminal() const { return state == UNRECOVERABLE; }

  void failUnrecoverable()
  {
    if (!terminal()) failure_count++;
    state = UNRECOVERABLE;
    validation_streak = 0;
  }

  void enterRecovery()
  {
    validation_streak = 0;
    state = RECOVERING;
  }

  bool beginLocalReinitialization()
  {
    if (terminal()) return false;
    if (attempts >= std::max(max_attempts, 1))
    {
      // The bounded budget counts distinct covariance reinitializations.  Its
      // exhaustion is not itself proof that the sensor data are unrecoverable:
      // a scan-to-scan constraint may still arrive before max_degraded_frames.
      // Keep the map frozen and let the caller hold the last regular covariance
      // without pretending that another independent attempt was made.
      return false;
    }
    attempts++;
    rollback_count++;
    state = RECOVERING;
    validation_streak = 0;
    return true;
  }

  void onInvalidFrame(bool numerical_failure)
  {
    if (terminal()) return;
    degraded_frames++;
    validation_streak = 0;
    if (degraded_frames >= std::max(max_degraded_frames, 1))
    {
      failUnrecoverable();
      return;
    }
    const int trigger = std::max(trigger_frames, 1);
    if (state == HEALTHY)
    {
      state = DEGRADED;
      if (numerical_failure) enterRecovery();
    }
    else if (state == DEGRADED)
    {
      if (numerical_failure || degraded_frames >= trigger) enterRecovery();
    }
    else if (state == VALIDATING)
    {
      enterRecovery();
    }
    else if (state == RECOVERING) state = RECOVERING;
  }

  void onValidFrame()
  {
    if (terminal()) return;
    if (state == HEALTHY)
    {
      degraded_frames = 0;
      return;
    }
    if (state == DEGRADED)
    {
      // A single transient invalid frame needs no state rollback episode.
      state = HEALTHY;
      degraded_frames = 0;
      attempts = 0;
      return;
    }
    if (state == RECOVERING)
    {
      state = VALIDATING;
      validation_streak = 1;
      degraded_frames = 0;
    }
    else if (state == VALIDATING)
    {
      validation_streak++;
    }

    if (state == VALIDATING && validation_streak >= std::max(validation_frames, 1))
    {
      state = HEALTHY;
      validation_streak = 0;
      degraded_frames = 0;
      attempts = 0;
      success_count++;
    }
  }

  void onRecoveryConstraintFrame()
  {
    if (terminal()) return;
    // A directionally qualified relative constraint is usable odometry data,
    // but it is not independent scan-to-map validation.  Keep the formal map
    // frozen and the state in RECOVERING while preventing a long, successful
    // fallback chain from being mislabeled as 300 consecutive failures.
    state = RECOVERING;
    degraded_frames = 0;
    validation_streak = 0;
  }
};

#endif
