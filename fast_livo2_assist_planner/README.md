# FAST-LIVO2 Assist Planner

`fast_livo2_assist_planner` is a human-in-the-loop advisory package for the
FAST-LIVO2 reliable online mapping project.

It is not a flight controller and does not publish MAVROS/PX4/DJI control
commands. It only publishes suggested local viewpoints, sparse paths, RViz
markers, and operator prompts.

The pilot-facing display is the video overlay, not RViz. RViz remains an
engineering/debug view for point clouds, paths, and markers. In rosbag replay,
`/left_camera/image` is used as a substitute first-person camera feed.

## Role

MVP scope: this package is a local assist advisor, not a complete autonomous
coverage planner. It uses current odometry, registered point cloud evidence, a
short-lived local evidence grid, and `/fast_livo2_health/advice` to recommend
short-distance local rescan, side-step, pull-back, climb/descend, or hover/check
actions.

Offline rosbag replay is a development and validation method. The intended
runtime target is online flight with FAST-LIVO2 and the health monitor running
in real time.

## Inputs

```text
/aft_mapped_to_init
/cloud_registered
/fast_livo2_health/advice
/fast_livo2_assist/pilot_intent
```

The health monitor itself may also consume `/LIVO2/imu_propagate`, `/livox/imu`,
`/cloud_effected`, `/fast_livo2/lio_diag`, and `/fast_livo2/vio_diag`; the assist
planner reads the fused health decision rather than subscribing to all evidence
directly.

## Outputs

```text
/fast_livo2_assist/recommended_path      nav_msgs/Path
/fast_livo2_assist/recommended_goal      geometry_msgs/PoseStamped
/fast_livo2_assist/candidate_viewpoints  geometry_msgs/PoseArray
/fast_livo2_assist/marker_array          visualization_msgs/MarkerArray
/fast_livo2_assist/status                std_msgs/String
/fast_livo2_assist/overlay_image         sensor_msgs/Image
/fast_livo2_assist/overlay_image/compressed sensor_msgs/CompressedImage
/tmp/fast_livo2_assist.csv               per-tick replay/validation log
```

`/fast_livo2_assist/status` uses the same key-value style as the health monitor:

```text
mode=EXPLORE;health_state=NORMAL;pilot_prompt=建议按推荐方向进行局部补扫;...
```

Values are kept semicolon-free because the dashboard parses the topic by `;`.
`reliability_score` follows `fast_livo2_health_monitor`: 0-100, where 100 is
best. The assist planner's `80.0` reliability threshold is therefore intentional.
The planner also consumes `confidence_score` and `confidence_level`.

## Pilot Intent

Reliable mapping is treated as a constraint, not as the final navigation goal.
The planner therefore needs a coarse pilot intent before it publishes normal
local advisory paths. Without an intent, `FREE` mode acts as a health/safety HUD
only in `NORMAL`, `WATCH`, and `RECOVERING`; `WARNING` can still trigger
pull-back/hold safety advice.

Intent topic:

```text
/fast_livo2_assist/pilot_intent  std_msgs/String
```

Accepted payloads may be plain strings:

```text
FORWARD_SCAN
LEFT_ORBIT
RIGHT_ORBIT
UP_SCAN
DOWN_SCAN
PULL_BACK
HOLD_STABLE
RESCAN_LOCAL
FREE
```

or semicolon key-value commands:

```text
intent=FORWARD_SCAN;strength=1.0;duration_s=8.0
```

`FORWARD_SCAN` latches the current yaw when the command is received, so short
camera/pose shake does not constantly redefine "forward". Orbit intents prefer
the tangent of the nearest local occupied surface when surface evidence is
available; otherwise they fall back to the latched yaw +/- 90 degrees.

**Important**: the planner rejects an intent message if no odometry has been
received yet and no explicit `yaw_deg` is provided. Re-send the intent once
FAST-LIVO2 is publishing `/aft_mapped_to_init`. Intent messages use
`latch=False`; a restarted planner node will not automatically replay the last
command.

Quick command-line test:

```bash
rosrun fast_livo2_assist_planner publish_intent.py FORWARD_SCAN 8
rosrun fast_livo2_assist_planner publish_intent.py LEFT_ORBIT 8
rosrun fast_livo2_assist_planner publish_intent.py FREE 0
```

The status topic and CSV include `pilot_mode`, `pilot_intent`,
`intent_bearing_deg`, `pilot_intent_strength`, and `task_progress`. These fields
show whether the chosen candidate is actually advancing the pilot's coarse
intent instead of only chasing local reliability.

## Health Policy

```text
NORMAL      -> EXPLORE, generate local rescan suggestions
WATCH       -> CONSERVATIVE, shorter goals and larger obstacle margin
WARNING     -> RETREAT_OR_HOLD, no close-in exploration
CRITICAL    -> SUPPRESSED, publish an empty path and hover/check prompt
RECOVERING  -> RECOVER, short and low-turn suggestions only
```

Confidence policy is applied only in the conservative direction:

```text
confidence_level=LOW       NORMAL is treated as WATCH for planning
confidence_level=VERY_LOW  SUPPRESSED, no path is published
```

### Action Code Gate

The planner treats `action_code` from the health monitor as an upper-level
safety supervisor. It is applied **after** the raw health state check and
**after** the confidence adjustment, producing a final `planning_state`:

```text
action_code=HOVER                           → planning suppressed (treat as CRITICAL)
action_code=CHECK_EVIDENCE + LOW confidence → planning suppressed
action_code=CHECK_EVIDENCE (normal conf.)   → at least CONSERVATIVE (WATCH)
action_code=REDUCE_MOTION                   → at least CONSERVATIVE (WATCH)
action_code=RECOVER_SLOW                    → RECOVERING planning mode
```

The original `health_state` is preserved in logs and CSV; only the internal
`planning_state` variable is adjusted.

### WARNING Sub-modes

In `WARNING`, retreat candidates are centered on the direction away from the
nearest local obstacle. If no valid obstacle direction is available, the planner
falls back to the UAV body-backward direction. The view yaw stays at the current
heading so the operator can pull away while still observing the structure.

The `motion_flags` field influences **how** the retreat is performed, not
**whether** to retreat. When `motion_flags != NONE` (fast or jerky motion
detected), the step radius is halved and the corridor is tightened to promote
smoother output. The retreat direction is unchanged.

`primary_reason` controls the exception to retreat:

```text
VISUAL_DEGRADED_LIO_STABLE → LIO is confirmed stable; allow small view-angle
                              change in the forward hemisphere instead of retreat.
VISUAL_POINTS_LOW           → treated as normal retreat; may indicate proximity,
VISUAL_TRACK_POOR             occlusion, or motion blur, not a safe scan opportunity.
other WARNING reasons       → default retreat / standoff increase.
```

The forward-motion health penalty (`+0.80`) is also skipped for the
`VISUAL_DEGRADED_LIO_STABLE` view-change branch so score discrimination between
candidates is preserved.

## Local Evidence Grid

The planner maintains a short-lived voxel evidence grid:

```text
FREE      observed free space along recent point rays
OCCUPIED  recent surface / obstacle hits
UNKNOWN   not recently verified
```

Candidate paths are sampled through this grid. A candidate is rejected if the
path intersects `OCCUPIED` voxels or if the path contains too much `UNKNOWN`.
Remaining `UNKNOWN` evidence becomes `unknown_risk` in the score. This prevents
the old single-frame failure mode where "no current point" could be mistaken for
"safe free space".

Surface gain is based on short-lived surface/frontier evidence: a candidate
is rewarded when it can view occupied surface voxels that border unknown cells,
while staying inside a configured standoff distance band. The older directional
point-density heuristic is retained as a fallback when `evidence_grid` is
disabled, and as a low-weight fallback when all candidate surface/frontier gains
are exhausted.

By default, `FREE` evidence is short-lived while `OCCUPIED` evidence is kept
longer. This keeps recently observed building surfaces available for retreat
and standoff reasoning without pretending to be a permanent global map.

**Cloud semantics**: `FREE` raycasting assumes `/cloud_registered` is the
current-frame registered scan (not a cumulative map). If the input is an
accumulated map, set `evidence_grid/free_raycast_enabled: false` to disable
ray-casting while retaining `OCCUPIED` surface evidence for retreat and standoff
reasoning. Setting `evidence_grid/enabled: false` removes both `FREE` and
`OCCUPIED` evidence entirely and is therefore not recommended for accumulated-map
use cases.

## Goal Commitment

The planner holds the current recommended goal for a minimum of
`stability/min_goal_hold_s` seconds unless the path becomes unsafe, the health
mode changes, or a new candidate is clearly better (score margin >
`switch_score_margin`). This keeps `/recommended_path` and the video cue stable
enough for a pilot to act on.

After goal commitment, the committed candidate's score is re-evaluated against
`scoring/min_accept_score`. If the refreshed score is too low (e.g., the
environment has changed), the commitment is released and planning is suppressed
until a valid candidate is found.

## Launch

```bash
roslaunch fast_livo2_assist_planner assist_planner.launch
```

Pilot-facing video overlay:

```bash
roslaunch fast_livo2_assist_planner assist_video_overlay.launch
```

View the overlay during replay:

```bash
rqt_image_view /fast_livo2_assist/overlay_image
```

The overlay consumes `/fast_livo2_assist/status` and draws a compact HUD on the
camera image:

```text
top bar             health state, assist mode, confidence
intent text         current pilot mode / intent
center cue          small camera reticle or HOLD symbol
flight director     relative left/right/up/down target cue (instrument-style)
intent tick         small marker on the flight-director rim showing coarse intent
target step bar     distance to recommended next waypoint; ideal-distance tick
bottom bar          pilot-facing short prompt
debug text          optional score/risk/candidate numbers
```

The bottom distance bar is labelled **TARGET STEP** — it shows the distance to
the recommended next waypoint, not the UAV-to-surface standoff distance.

When mode is `SUPPRESSED` the HUD uses grey regardless of `health_state`.
Even with `health_state=NORMAL`, grey indicates that no guidance is being
published (e.g., no pilot intent selected).

This first version uses `target_bearing_deg` and `target_distance_m` from the
assist status, so it can run without camera calibration. It is suitable for
rosbag demonstration and pilot-interface prototyping. A later calibrated version
can project the 3D recommended path into the camera using camera intrinsics,
extrinsics, and TF.

`assist_planner.launch` loads `config/assist_planner.yaml` inside the node's
private namespace, matching the C++ `ros::NodeHandle("~")` parameter lookup.

For rosbag联调:

```bash
roslaunch fast_livo2_assist_planner replay_full_stack.launch
```

Common replay options:

```bash
roslaunch fast_livo2_assist_planner replay_full_stack.launch \
  start_fast_livo2:=true \
  start_video_overlay:=true \
  play_bag:=true \
  bag:=/path/to/data.bag \
  bag_rate:=1.0
```

Use `fast_livo2_launch:=...` if the dataset is not the Livox AVIA config.

## ROS Time Reset

When a rosbag is replayed from the beginning, `/clock` rolls back. The planner
detects backward time jumps at the start of each evaluation tick and
automatically clears:

- the local evidence grid (all FREE/OCCUPIED voxels)
- sector visit counts
- the committed goal
- the active pilot intent

One `TIME_RESET` suppressed tick is published, after which the planner resumes
normally. This prevents stale TTL comparisons and ghost committed goals from the
previous run.

## Freshness Gates

The planner suppresses path suggestions when required upstream evidence is stale:

```text
runtime/odom_timeout_s     FAST-LIVO2 odom freshness gate
runtime/health_timeout_s   health advice freshness gate
map/cloud_timeout_s        registered cloud freshness gate
map/require_fresh_cloud    require non-empty fresh cloud before planning
map/suppress_on_frame_mismatch suppress planning when cloud frame != world_frame
```

## Validation CSV

When `logging/enable_csv` is true, the planner writes `/tmp/fast_livo2_assist.csv`.
Useful columns include:

```text
stamp,health_state,mode,reason,candidate_count,top_score,
pilot_mode,pilot_intent,intent_bearing_deg,task_progress,
coverage_gain,collision_risk,health_penalty,unknown_risk,path_unknown_ratio,
path_occupied_count,target_distance_m,
reliability_score,confidence_score,confidence_level,
health_action_code,
odom_age_s,health_age_s,cloud_age_s,cloud_frame_ok,grid_voxels,grid_occupied
```

This file is meant to be aligned with the health monitor CSV during rosbag
replay, so checks such as "CRITICAL implies candidate_count=0" can be verified
from data rather than only from RViz.

## Current Limitations

- The evidence grid is short-lived and local; it is not a global OctoMap or a
  certified collision map.
- The planner prevents short-term repeated local suggestions, but it does not
  remember global facade coverage after the UAV leaves the local grid.
- UNKNOWN is treated conservatively for path suggestions, but the pilot remains
  the final safety authority.
- The recommended path is a short advisory line, not a dynamically feasible
  flight trajectory.
- Coverage gain is a local surface/frontier heuristic, not a full next-best-view
  optimizer.
- Obstacle and surface evidence is estimated from `/cloud_registered`; stale,
  empty, or wrong-frame clouds must suppress planning.
- `/cloud_registered` is assumed to be the current-frame registered scan in
  `world_frame` for MVP. Raycasting FREE evidence is only valid for current-frame
  scans. The node suppresses planning on frame mismatch but does not perform TF
  transforms yet.
- The node must not be connected to MAVROS/PX4/DJI control interfaces in this
  stage.
- `target_distance_m` in the HUD is the step distance to the recommended
  waypoint, not the UAV-to-surface standoff. True surface standoff requires
  exporting `surface_distance_m` from the planner (future work).
- AR path projection into the camera image requires verified camera intrinsics,
  camera-LiDAR extrinsics, TF chain, and time synchronisation (future work).
