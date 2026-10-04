# FAST-LIVO2 Source Modifications

## Nine-point goal R1: directional reliability and quarantined map recovery (2026-07-14)

R1 is a bounded refactor inside the existing nine-point scope. It does not use
the frozen baseline as a development source and does not add a tenth direction.

- The prior-whitened coupled 6DoF degeneracy solve now exports its six state
  directions and their reliability factors. This preserves the already-correct
  coupled IEKF update and provides the directional interface required by the
  later LIO-VIO complementarity stage.
- Rejected frontier observations no longer enter the formal voxel map in the
  same frame. They are held in a bounded persistent candidate map, require
  consistent geometry in at least three frames, expire after a bounded age,
  and may be promoted only after pose/map risk returns below the normal write
  threshold. Dynamic conflicts never enter the candidate map. Recovery freezes
  the formal map, and reset discards candidates without promotion.
- Default residual weighting was reduced to one robust residual precision.
  Correlated normal-balance, incidence, scan-distortion, plane-quality and
  preprocessing sampling scalars remain available for ablation where useful,
  but are disabled by default. Obsolete correlated-fusion and dynamic minimum
  weight parameters were removed; dynamic evidence remains a hard conflict and
  map-quarantine signal.
- LIO diagnostics append candidate-map counts, promotion/expiry counts, formal
  map freeze state and six directional reliability factors at `[77..87]`.
  The authoritative LIO/VIO widths are therefore `88/45`.
- `/cloud_registered` no longer mixes empty PointCloud2 messages into LIVO
  output. A VIO cycle publishes its colored registered cloud when non-empty and
  otherwise falls back to the paired registered intensity cloud. Estimation is
  unchanged; this restores truthful point-count/rate evidence for monitoring
  and downstream mapping consumers.

R1 verification in `/home/fastlivo/codex_rc_ws_20260712`:

- ROS Noetic Release build with testing enabled succeeded. The three gtest
  binaries reported 36 records, 0 errors, 0 failures and 0 skipped tests.
- The six mapping YAML files passed the 107-key adaptive configuration and
  release-safety validator.
- A 11.75 s real Mid360/IMU/camera bag replay produced 114 finite 88-field LIO
  rows and 113 45-field VIO rows. All LIO rows converged; median effective
  constraints were 3,592, residual P95 was 0.01416 m and LIO time P95 was
  10.89 ms. No NaN, crash or shutdown segmentation fault occurred.
- On the same replay, registered-cloud evidence changed from an empty-message
  mixture near 20 Hz to valid output near 10 Hz, with 5,241--8,887 points in
  the final health samples and 100% required evidence confidence. The segment
  still has zero tracked visual constraints and is correctly reported as
  `WATCH:VISUAL_DEGRADED_LIO_STABLE`; R1 does not claim visual recovery.
- Reproducible evidence is stored under
  `Log/r1_realbag_smoke_20260714_after_cloud_fix`; the runner is
  `scripts/run_r1_realbag_smoke.sh`.

## Latest round: bounded convergence and estimator-consistency closure

### Release-candidate safety closure (2026-07-12)

- A LIO frame is accepted only when at least one finite, positive-precision
  point-to-plane constraint actually reaches the linear system. Zero-match and
  all-invalid-constraint frames roll back to the propagated prior and cannot
  write the voxel map or run their paired VIO update.
- LiDAR point offsets are filtered to the active propagation window and sorted
  before reverse deskew. A configurable forward IMU gap limit
  (`imu/max_propagation_interval`, default `0.5 s`) rejects discontinuous
  integration and requests the same coordinated reset used for clock rewind.
- `debug/state_log_en` and `debug/imu_log_en` default to `false`. State and IMU
  files are opened and written only when explicitly enabled; hot-path records
  use newline without a per-sample forced flush.
- Sensor-enable combinations, LiDAR type, extrinsic SO(3) matrices and COLMAP
  camera type now fail fast. LiDAR-only operation no longer requires a camera
  model to initialize an unused VIO pipeline.
- General mapping configs do not enable unbounded PCD accumulation. Dedicated
  save configs use a positive segment interval so long runs have a bounded
  in-memory save buffer.

The early instrumentation-only and test-count paragraphs below are retained as
historical revision records. They do not describe the current algorithm scope
or the current three-test-source suite.

### Release-candidate verification (2026-07-12)

- A clean ROS Noetic Release build with testing enabled completed in the
  isolated VM workspace `/home/fastlivo/codex_rc_ws_20260712`. All core shared
  libraries and `fastlivo_mapping` linked successfully.
- The three gtest binaries ran 16 cases. `catkin_test_results --verbose`
  reported 32 records, 0 errors, 0 failures and 0 skipped tests. The added
  `StateEstimationAcceptance.RejectsFrameWithoutUsablePointToPlaneConstraint`
  case exercised the zero-constraint rollback path directly.
- Runtime dependency inspection of `fastlivo_mapping` reported no unresolved
  shared libraries. The verified executable SHA-256 was
  `2fe4675cfd19a7bf37663e71ab07e071f10a145bb5a372aa41cbf007e045f7a5`.
- `cppcheck` warning/portability analysis passed for the four touched core
  implementations and the recovery test. `git diff --check`, Python byte
  compilation, and the extended YAML validator passed; the validator confirmed
  109 adaptive keys plus release safety defaults across all six mapping YAMLs.
- This verification closes build, link, unit-test and static-contract evidence.
  It does not replace target-board long-duration replay, RSS/P95/P99 measurement,
  or ground-truth ATE/RPE experiments.

This round deliberately adds no new degradation score. It closes the confirmed
correctness and experiment-contract issues around the existing algorithms:

- VIO direct patches use exact per-level bounds and `cv::Mat::step`; each
  pyramid solve keeps a fixed support set, and the final injected state is
  reprojected once for acceptance before its covariance is committed.
- Visual-point lifecycle and reference changes are transactional. Numerical or
  no-valid-linearization rejection restores point state and never deletes an
  observation from the long-term visual map.
- LIO map-write correspondences are refreshed at the final accepted IEKF pose,
  while filter mean/covariance/information stay untouched. The map gate uses the
  current LIO pose-health risk rather than a stale joint VIO risk.
- All 72 adaptive floating-point parameters have finite fallback and range
  protection. Voxel topology, extrinsic vector sizes and point timestamps are
  validated before indexed or floating-point use.
- ARM no longer enables global `-ffast-math`; every core target explicitly keeps
  IEEE NaN/Inf semantics. x86 keeps the distribution-PCL-compatible Eigen
  alignment contract.
- Replay contract v3 records input/config hashes and a six-artifact runtime
  bundle, uses `/use_sim_time` plus `rosbag --clock`, and rejects mismatched
  parameters, executables/libraries or incomplete output.

The authoritative diagnostic layouts are:

- `/fast_livo2/lio_diag`: 88 values, indices `[0..87]`.
- `/fast_livo2/vio_diag`: 45 values, indices `[0..44]`.

### IMU processing and scheduling safety

- The point-time comparator now has internal linkage in `IMU_Processing.cpp`,
  removing the header-level ODR/link risk.
- IMU processing returns `Rejected`, `Initializing`, `PropagatedOnly`, or
  `Ready`; only `Ready` enters LIO/VIO estimation and map writing.
- Null/empty inputs, non-finite IMU and point data, invalid acceleration scale,
  backward time and out-of-window samples are rejected before map use. LIO also
  requires an IMU sample newer than the previous propagation watermark.
- Empty-IMU VIO is accepted only when VIO time, paired LIO time and the latest
  propagation time are equal. Empty LiDAR can advance IMU propagation as
  `PropagatedOnly` without running ESIKF or writing the map.
- Rejected LIVO LIO drops exactly its paired VIO. Pending points are rebased to
  the last accepted LIO time, while valid consumed IMU samples are carried into
  the next LIO batch so neither point nor inertial time history is lost.
- Runtime rejection restores the previous IMU propagation/filter snapshot; it
  does not reinitialize IMU orientation against an existing map. Startup reset
  clears propagation timing, scratch clouds/poses and degradation history.
- Fatal ESIKF rollback is returned to the main scheduler, blocks the paired VIO,
  and removes a voxel map tentatively built by a failing bootstrap frame.
- `imu_proc` uses `-fno-fast-math` so finite guards preserve IEEE semantics on ARM.
- `motion_degrade` now contains only velocity, angular-rate and jerk evidence.
  Vibration remains an independent process-noise/downstream risk input.
- IMU initialization is transactional: the first sample is counted once, and
  means, counters, the previous sample and the initialized state are committed
  only after the complete candidate state passes finite checks.
- Standard/Livox LiDAR, IMU and image callbacks reject null/non-finite inputs
  before queue timestamps advance. The high-rate IMU path also validates each
  sample, time step, acceleration scale and propagated state before publishing.
- A timestamp rewind now requests one coordinated main-thread reset. It clears
  all paired sensor queues and rejected-IMU carry, resets IMU/VIO timing and risk
  history, deletes both voxel and visual maps, restores the estimator prior and
  clears pending publication/save clouds. Rewind-triggering packets are dropped
  so samples from two rosbag sessions cannot be mixed.
- LiDAR preprocessing bounds a Livox `point_num`/container mismatch and protects
  empty standard-LiDAR handlers, empty scan lines and all-blind feature scans.
  `last_valid_count` is now accumulated in each handler before downsampling,
  rather than inferred from `output_count * point_filter_num`.
- `debug/verbose_runtime_log` defaults to `false` in all primary YAML files and
  suppresses per-frame LIO/VIO timing tables, point-count prints and map-sliding
  debug output. Diagnostic topics remain unchanged; enabling the key restores the
  detailed console path for profiling.
- Hot-path allocation was reduced without decimating any algorithm: normal bins
  are fixed constants, normal/range/residual buffers retain capacity, and LIO no
  longer frees/reallocates `pv_list_` every frame. OpenMP residual association now
  writes per-index byte flags and cached constraints, removing the per-point mutex
  required by the previous bit-packed `vector<bool>` implementation.

### Innovation 23: candidate/matched dual range distributions

Innovation 20 updated its range reference from successful matches only. That
selection is biased during altitude changes or incomplete map coverage because
the far returns that should raise the reference are also likely to fail matching.

- `adaptive/lidar_range_candidate_source_en=true` uses all finite positive
  iteration-zero `pv_list_` ranges as the active quantile source.
- Setting it to `false` restores the matched-only `ptpl_list_` source, including
  the original 20-sample threshold, quantile index, EMA and base/max bounds.
- Candidate/matched quantiles use local copies. Existing intensity and physical
  range thresholds are unchanged, and range parameters have finite fallbacks.
- Fatal ESIKF rollback restores the persistent effective range reference,
  accepted quantile and EMA initialization state.
- Candidate/matched diagnostics remain active when the outer range-adaptation
  switch is disabled, allowing the fixed-reference baseline to plot selection gap.

The original LIO diagnostic prefix `[0..61]` is unchanged. New fields are:

| Index | Field | Meaning |
|---:|---|---|
| `[62]` | `candidate_quantile_m` | All-candidate quantile, `-1` if unavailable |
| `[63]` | `matched_quantile_m` | Successful-match quantile, `-1` if unavailable |
| `[64]` | `selection_gap_m` | Candidate minus matched quantile, `-1` if unavailable |
| `[65]` | `candidate_count` | Finite positive candidate ranges at iteration zero |
| `[66]` | `matched_coverage` | Matched range count divided by candidate count |

LIO diagnostics contain 88 values (`[0..87]`); VIO contains 45 values (`[0..44]`).

### Verification and bounded replay closure (2026-07-11)

All final-code results below use the source archived with SHA-256
`1833b031cd36d3bcb2f902406741b8f25f2de93c73918483fed294e160b313ba` and
expanded into both VM workspaces before building.

- Historical verification at that revision: a cleanly reconfigured Noetic Release build completed in
  `/home/fastlivo/codex_catkin_ws_20260710`. The two registered gtest binaries
  ran 9 cases in total; `catkin_test_results --verbose` reported 18 records,
  0 errors, 0 failures and 0 skipped tests.
- Cppcheck 2.19.0 completed the `warning,portability` exhaustive pass with no
  findings after suppressing only missing system headers and PCL registration
  parser macros. `git diff --check`, Python byte compilation and both Bash
  syntax checks passed. `validate_adaptive_config.py` confirmed 109 adaptive
  keys across all 6 mapping YAML files, including duplicate-key and non-finite
  value rejection.
- A `RelWithDebInfo` AddressSanitizer build completed in
  `/home/fastlivo/codex_asan_ws_20260710`. A 20-second Bright `all_off`
  contract-v3 smoke run covered both known sub-millisecond out-of-order IMU
  packets, completed with zero process failures and emitted no AddressSanitizer
  or undefined-runtime report. Leak detection was disabled because ROS/PCL
  process teardown ownership is outside this package; bounds/use-after-free
  detection remained fatal.
- The formal HKU result root is
  `/home/fastlivo/codex_experiments_20260711/hku_contract_v3_20260711_final`.
  All five requested modes used the same 25-second input interval at rate 0.5
  and passed the analyzer contract. Current-code modes each produced 246 LIO
  rows of width 77 and 245 VIO rows of width 45. LIO numerical fallback and VIO
  covariance failure counts were zero; the accepted visual-update ratio was
  `0.995918`. Their runtime-bundle SHA-256 was
  `fd17b09e29163bae403efc81075f5fa696baaf7db4b669dd0c3db76acb7e87f0`.
  The stock binary correctly has no modified diagnostic topics and has a
  distinct baseline runtime bundle.
- The formal bright-screen result root is
  `/home/fastlivo/codex_experiments_20260711/bright_contract_v3_20260711_final`.
  `all_off` and `full` covered 66.68 seconds at rate 0.5 with compressed-image
  republishing alive throughout. Both produced 663 LIO and 663 VIO rows of
  widths 77/45, with zero LIO numerical fallbacks, zero VIO covariance failures
  and no coordinated reset. Each mode discarded only the two source-bag IMU
  packets that arrive `0.505 ms` and `0.405 ms` behind their predecessors. The
  accepted visual-update ratio was `0.998492`; maximum map-write starvation was
  zero frames in both modes.
- Re-analysis of `hku_matrix_20260710_213003` and
  `hku_matrix_r2_20260710_213635` intentionally marks both roots invalid as
  formal evidence: they predate contract v2, lack input/config/binary hashes and
  simulated-time coverage proof, and contain the historical 44-field VIO
  layout. They remain historical repeatability references only.
- The intermediate contract-v2 HKU/Bright roots are also intentionally invalid
  under the final analyzer. They hashed only `fastlivo_mapping`, while estimator
  changes live in five dynamically linked core libraries. Contract v3 adds
  `runtime_manifest.txt` and a bundle digest over the executable plus
  `liblaser_mapping`, `liblio`, `libvio`, `libpre` and `libimu_proc`.

The final independent cold review found and fixed two P1 experiment breakers:

- Sensor callbacks previously treated every backward jump above `1 us` as a
  session rewind. The Bright bag contains ordinary IMU delivery inversions of
  only `0.505 ms` and `0.405 ms`, so each run cleared the estimator and maps
  twice. LiDAR, IMU and image callbacks now discard late packets below `0.1 s`
  and reserve coordinated reset for an actual clock/session rewind.
- Hashing only the launcher executable could report an unchanged binary after
  `LIVMapper.cpp`, VIO or voxel-map code changed in shared libraries. The v3
  runtime bundle closes that false-positive path and the analyzer verifies the
  manifest artifact set, manifest digest and cross-mode bundle identity.

Bounded residual risks recorded at goal closure, without further expansion:

- VIO fixed-support borders and visual-map transactions still lack a dedicated
  unit-test fixture. They are covered here by Release replay and the Bright ASan
  run, which is weaker than direct fault-injection coverage.
- Map-write recovery episode length and long-horizon nominal EMA absorption
  remain P2 tuning risks. The final short replays show zero starvation frames,
  but they cannot substitute for hour-scale degraded-flight validation.
- The `0.1 s` clock-rewind discriminator is a conservative cross-sensor safety
  constant. Sensor-specific networks with legitimate reordering above that
  bound would require a separately justified parameter study.
- AddressSanitizer was run, but ThreadSanitizer was not. Mutex/atomic ownership
  was reviewed statically and replayed, not dynamically race-instrumented.
- No accepted ground truth or real-flight ablation set is present, so accuracy,
  long-term map quality and flight-computer real-time margins remain unproven.

Neither replay bag contains accepted ground truth in this experiment contract.
The analyzer's trajectory disagreements therefore show mode divergence and
repeatability only; they are not ATE/RPE and do not prove mapping-accuracy gains.

本文档只记录对原始 FAST-LIVO2 源码本身的修改。新的健康监测节点、网页仪表盘、评分逻辑和启动方式见相邻包：

`C:\Users\13365\FAST-LIVO2-Reproduce\fast_livo2_health_monitor\README.md`

当前修改版源码位置：

`C:\Users\13365\FAST-LIVO2-Reproduce\FAST-LIVO2`

原始拿到的源码位置：https://github.com/hku-mars/FAST-LIVO2

## 1. 修改目标

本次对 FAST-LIVO2 的修改目标不是改变定位建图算法，而是把原本只存在于内部变量或终端打印中的质量指标发布成 ROS 话题，供外部健康监测节点旁路订阅。

核心原则：

- 不改变 LIO/VIO/ESIKF 的状态估计流程。
- 不改变原有 odom、点云、图像等输出话题语义。
- 不改变原有阈值、优化迭代、滤波更新和地图维护逻辑。
- 只新增少量成员变量和 publisher，把已经计算出来的中间指标暴露出去。

因此，这些改动属于 instrumentation / observability，即工程观测能力增强，不属于算法主体改写。

## 2. 新增诊断话题

修改后的 FAST-LIVO2 会额外发布两个诊断话题：

| 话题 | 类型 | 发布位置 | 用途 |
|---|---|---|---|
| `/fast_livo2/lio_diag` | `std_msgs/Float64MultiArray` | 每个 LIO 更新周期 | 暴露 LiDAR/LIO 内部配准质量 |
| `/fast_livo2/vio_diag` | `std_msgs/Float64MultiArray` | 每个 VIO 更新周期 | 暴露视觉跟踪质量 |

这两个话题由新建的 `fast_livo2_health_monitor` 节点订阅，用于判断建图可靠性是否下降。

## 3. LIO 诊断字段

`/fast_livo2/lio_diag` 的数组字段定义如下：

| Index | 字段 | 含义 | 单位 |
|---:|---|---|---|
| `[0]` | `stamp` | FAST-LIVO2 内部 LIO 更新时间戳 | s |
| `[1]` | `raw_pts` | 去畸变后的原始 LiDAR 点数 | points |
| `[2]` | `down_pts` | 下采样后的点数 | points |
| `[3]` | `effective_pts` | ESIKF 更新中实际采用的有效点到平面约束数量 | points |
| `[4]` | `avg_residual` | 平均点到平面残差 | m |
| `[5]` | `converged` | 当前 LIO/ESIKF 迭代是否收敛，`1.0` 为收敛，`0.0` 为未收敛 | - |
| `[6]` | `lio_frame_time` | 当前 LIO 帧处理耗时 | s |
| `[7]` | `cov_trace` | ESIKF 位姿 6x6 子块协方差 trace，旋转 3 维 + 平移 3 维 | - |

这些字段的工程意义：

- `effective_pts` 反映 LiDAR 几何约束是否充足，比单纯点云总点数更接近配准质量。
- `avg_residual` 反映点到平面约束误差是否变大。
- `converged` 反映 ESIKF 迭代是否按预期完成。
- `cov_trace` 反映滤波器对当前位姿估计的不确定性。
- `lio_frame_time` 用于观察计算负载，但当前健康监测逻辑主要将它作为展示指标。

## 4. VIO 诊断字段

`/fast_livo2/vio_diag` 的数组字段定义如下：

| Index | 字段 | 含义 | 单位 |
|---:|---|---|---|
| `[0]` | `stamp` | FAST-LIVO2 内部 VIO 更新时间戳 | s |
| `[1]` | `tracked` | 当前视觉子图中参与跟踪的视觉点数量 | points |
| `[2]` | `good` | 误差满足 `error <= propa_error` 的良好视觉点数量 | points |
| `[3]` | `mean_error` | 平均光度/重投影相关误差，按 FAST-LIVO2 原内部误差量纲输出 | - |
| `[4]` | `map_size` | 当前视觉稀疏地图体素数量 | voxels |
| `[5]` | `vio_frame_time` | 当前 VIO 帧处理耗时 | s |

健康监测节点主要使用：

- `tracked`
- `good / tracked`
- `vio_frame_time`

其中 `good / tracked` 被解释为视觉跟踪质量比例。它不是绝对视觉精度真值，而是基于 FAST-LIVO2 内部误差判据构造的工程指标。

## 5. 具体源码修改

### 5.1 `include/voxel_map.h`

在 `VoxelMapManager` 相关类中新增成员变量：

```cpp
double avg_residual_ = 0.0;
bool ekf_converged_ = false;
```

目的：

- `avg_residual_` 保存最后一次 LIO ESIKF 更新中的平均点到平面残差。
- `ekf_converged_` 保存最后一次 LIO ESIKF 更新是否收敛。

原 FAST-LIVO2 中这些量只在局部变量或终端打印中出现，函数返回后外部无法读取。

### 5.2 `src/voxel_map.cpp`

在 LIO 状态估计过程中保存平均残差：

```cpp
avg_residual_ = effct_feat_num_ > 0 ? total_residual / effct_feat_num_ : -1.0;
```

并在 ESIKF 迭代结束后保存收敛标志：

```cpp
ekf_converged_ = flg_EKF_converged;
```

这两处都是对已有计算结果的缓存，不改变原来的计算路径。

### 5.3 `include/vio.h`

在 `VIOManager` 中新增：

```cpp
double last_frame_time = 0.0;
```

目的：

- 将 VIO 单帧处理耗时保存为成员变量，供 `LIVMapper` 发布诊断话题。

### 5.4 `src/vio.cpp`

在 `processFrame()` 中保存 VIO 单帧耗时：

```cpp
last_frame_time = t7 - t1 - (t5 - t4);
```

该表达式与原终端统计中的 VIO 总耗时口径一致，减去了部分参考 patch 更新时间。

### 5.5 `include/LIVMapper.h`

新增头文件：

```cpp
#include <std_msgs/Float64MultiArray.h>
```

新增 publisher 成员：

```cpp
ros::Publisher pubLioDiag;
ros::Publisher pubVioDiag;
```

### 5.6 `src/LIVMapper.cpp`

在 `initializeSubscribersAndPublishers()` 中注册诊断话题：

```cpp
pubLioDiag = nh.advertise<std_msgs::Float64MultiArray>("/fast_livo2/lio_diag", 100);
pubVioDiag = nh.advertise<std_msgs::Float64MultiArray>("/fast_livo2/vio_diag", 100);
```

在 `handleVIO()` 完成 VIO 帧处理后发布 `/fast_livo2/vio_diag`：

```cpp
std_msgs::Float64MultiArray vio_diag;
vio_diag.data.push_back(LidarMeasures.last_lio_update_time);
vio_diag.data.push_back(static_cast<double>(tracked));
vio_diag.data.push_back(static_cast<double>(good));
vio_diag.data.push_back(tracked > 0 ? err_sum / tracked : -1.0);
vio_diag.data.push_back(static_cast<double>(voxelmap_manager->voxel_map_.size()));
vio_diag.data.push_back(vio_manager->last_frame_time);
pubVioDiag.publish(vio_diag);
```

实现细节：

- 遍历视觉子图中的 `voxel_points`、`errors`、`propa_errors`。
- 使用三者最小长度作为遍历上界，避免旁路诊断越界。
- `error <= propa_error` 的点计为 `good`。

在 `handleLIO()` 完成 LIO 帧处理后发布 `/fast_livo2/lio_diag`：

```cpp
std_msgs::Float64MultiArray lio_diag;
lio_diag.data.push_back(LidarMeasures.last_lio_update_time);
lio_diag.data.push_back(static_cast<double>(feats_undistort->points.size()));
lio_diag.data.push_back(static_cast<double>(feats_down_size));
lio_diag.data.push_back(static_cast<double>(voxelmap_manager->effct_feat_num_));
lio_diag.data.push_back(voxelmap_manager->avg_residual_);
lio_diag.data.push_back(voxelmap_manager->ekf_converged_ ? 1.0 : 0.0);
lio_diag.data.push_back(t4 - t0);
lio_diag.data.push_back(_state.cov.block<6,6>(0,0).trace());
pubLioDiag.publish(lio_diag);
```

## 6. 影响范围评估

| 文件 | 类型 | 对原算法影响 |
|---|---|---|
| `include/voxel_map.h` | 新增缓存成员变量 | 无，仅读取已有中间结果 |
| `src/voxel_map.cpp` | 保存残差和收敛状态 | 无，不改变 ESIKF 更新 |
| `include/vio.h` | 新增耗时成员变量 | 无 |
| `src/vio.cpp` | 保存 VIO 帧耗时 | 无 |
| `include/LIVMapper.h` | 新增 publisher 成员 | 无 |
| `src/LIVMapper.cpp` | 注册并发布诊断话题 | 无，不改变原发布话题 |

潜在运行成本：

- 每个 LIO/VIO 帧构造并发布一个很小的 `Float64MultiArray`。
- VIO 诊断需要遍历视觉子图统计 `tracked/good`，但只做简单计数和误差求和。
- 对 CPU 和带宽影响很小，远低于点云、图像和地图维护的主计算量。

## 7. 与健康监测节点的关系

健康监测节点可以只依赖原始 FAST-LIVO2 话题运行，但诊断能力会下降。

有本源码修改时，健康监测节点可以获得：

- LIO 有效匹配点
- LIO 平均残差
- LIO 收敛状态
- ESIKF 位姿协方差 trace
- VIO 跟踪点数
- VIO 良好跟踪比例

没有本源码修改时，健康监测节点仍可根据 odom、点云、IMU 心跳做粗粒度监测，但无法判断“正在退化但尚未断流”的内部质量变化。

## 8. 编译与验证

在 catkin 工作空间中编译：

```bash
cd ~/catkin_ws
catkin_make --pkg fast_livo
```

启动 FAST-LIVO2 后检查诊断话题：

```bash
rostopic list | grep fast_livo2
rostopic echo -n 1 /fast_livo2/lio_diag
rostopic echo -n 1 /fast_livo2/vio_diag
```

预期：

- `/fast_livo2/lio_diag` 应在 LIO 更新后持续输出 88 个数，当前索引为 `[0..87]`。
- `/fast_livo2/vio_diag` 应在 VIO 更新后持续输出 45 个数，当前索引为 `[0..44]`。

若没有这些话题，请优先确认：

- 当前运行的是修改后的 `FAST-LIVO2`。
- catkin 工作空间已经重新编译并 `source devel/setup.bash`。
- 启动的是正确的 FAST-LIVO2 launch 文件。

---

# 第二阶段修改：主链路退化感知与自适应建图（算法改动）

以上第 1–8 节的修改属于旁路观测（instrumentation），**不改变算法**。本节记录的第二阶段修改是**算法主体改动**：针对无人机边飞边建中除计算资源外的退化因素（光照、飞行速度、观测视角、场景几何结构），在 FAST-LIVO2 主链路内部实现退化感知与自适应处理。所有改动处均以 `MODIFICATIONS:` 注释标注，可全局搜索定位。

## 9. 五个创新点总览

| # | 创新点 | 对应退化因素 | 修改文件 |
|---|---|---|---|
| 1 | H^T R⁻¹ H 分块特征值退化检测 + 方向性增量抑制 + 协方差一致性 | 大平面/走廊等几何退化 | `voxel_map.h/.cpp` |
| 2 | 光照/纹理质量评估 + 自适应视觉观测协方差 + 坏帧地图门控 | 弱光、过曝、低纹理 | `vio.h/.cpp` |
| 3 | 运动激励自适应过程噪声 | 高速飞行、急转弯 | `IMU_Processing.h/.cpp` |
| 4 | 点到面约束法向各向异性分析 | 俯视地面/单面墙的视角退化 | `voxel_map.h/.cpp` |
| 5 | LIO 信息增益驱动的 VIO 信任度自适应 | LIO-VIO 融合权重固定、互不感知 | `LIVMapper.h/.cpp`, `vio.cpp` |

数据流闭环：

```text
IMU 传播(3: 运动激励→过程噪声膨胀, 输出 motion_degrade)
  → LIO ESIKF(4: 法向各向异性; 1: 分块特征值→方向性抑制 D, 协方差一致更新)
  → LIO 信息增益(5: tr(P⁻¹) 前后差 → vio_trust_factor)
  → VIO 帧(2: 图像质量评估) → R_vio = R_base / max(Q²,ε) · (1+3·md) · clamp(trust,0.1,2)
  → 坏帧门控(低质量帧不写长期视觉地图, 带饥饿保护)
```

## 10. 创新点 1：方向性几何退化抑制（`voxel_map.cpp` `StateEstimation()`）

**问题**：无人机俯视大面积地面/单面墙时，点到面约束的法向近乎平行，H^T R⁻¹ H 在某些方向病态，ESIKF 等权更新导致沿平面方向漂移。原码 L468 被注释掉的 `EigenSolver` 表明原作者考虑过但未实现。

**实现**（相比常见 LOAM 式二元退化判断的两点改进）：

1. **分块分析**：旋转块(rad)与平移块(m)量纲不同、量级差异大（旋转项随杠杆臂平方增长），联合 6×6 特征值比较会让量级大的块掩盖另一块的退化。因此对两个 3×3 子块分别做特征值分解，块内做比值：
   `d_i = sigmoid(α·(λ_i/λ_max^blk − τ))`，α=10、τ=0.05（可配），连续缩放无硬切换抖动。
2. **增量重映射**：`D = blkdiag(V_r diag(d_r) V_rᵀ, V_t diag(d_t) V_tᵀ)`，只作用于**测量驱动的增量**部分：`Δx₆ = Δx_prior + D·(Δx₆ − Δx_prior)`，退化方向回退到 IMU 先验增量而非被病态约束污染。
3. **协方差一致性**：协方差更新时对增益矩阵 G 的位姿行块同样左乘 D（`G(0:5,0:5) = D·G(0:5,0:5)`），被抑制的方向保持先验不确定性、不虚假收缩——否则滤波器在退化方向过度自信，且会污染创新点 5 的信息增益指标。

新增成员（`voxel_map.h`）：`degeneracy_aware_en_`、`degeneracy_alpha_`、`degeneracy_tau_`、`degeneracy_eigenvalues_`、`degeneracy_factor_`（0=完全退化，1=健康）。

计算代价：每次迭代两个 3×3 特征值分解，微秒级。

## 11. 创新点 4：法向各向异性分析（`voxel_map.cpp` `StateEstimation()`）

对每次迭代的有效点到面约束法向构造散布矩阵 `N = (1/k)Σ nᵢnᵢᵀ`，`normal_anisotropy_ = λ_min(N)/λ_max(N)`：趋近 0 表示法向集中单一方向（平移沿平面不可观），趋近 1 表示法向分布均匀。它从约束几何本质上**先于** H 矩阵解释"为什么俯视时退化"，与创新点 1 交叉验证，并通过诊断话题输出。

## 12. 创新点 2：光照自适应视觉协方差与坏帧门控（`vio.cpp`）

**问题**：`img_point_cov` 是启动时固定的常数；过曝/欠曝/模糊/低纹理帧照常参与 EKF 更新、照常生成视觉地图点和更新参考 patch，短期污染位姿、长期污染地图。

**实现**：

- 新增 `assessImageQuality()`：在 4 像素抽样网格上一次遍历同时统计梯度均值/方差（纹理丰富度）、饱和+欠曝像素比例、逆曝光时间帧间稳定性，合成 `Q = 0.4·σ_grad + 0.3·(1−r_sat) + 0.3·s_expo`，下限 0.05。抽样计算量约为全图 Sobel 的 1/16。
- `processFrame()` 开头计算本帧有效协方差（三因子融合，创新点 2/3/5 汇聚点）：
  `current_frame_img_cov = img_point_cov / max(Q²,0.01) · (1+3·motion_degrade) · clamp(lio_info_ratio,0.1,2)`
- `updateState()` 与 `updateStateInverse()` 中 `state->cov / img_point_cov` 改为除以 `current_frame_img_cov`。
- **坏帧门控**：`visual_quality = Q·(1−0.5·motion_degrade)` 低于 `min_gen_quality`(0.15) 时跳过 `generateVisualMapPoints()`，低于 `min_ref_update_quality`(0.20) 时跳过 `updateReferencePatch()`——坏帧仍可用于短期跟踪，但不写入长期地图。
- **饥饿保护**（工程安全设计）：当前跟踪点少于 30 个时无条件允许生成新点，防止长时间弱光飞行导致视觉地图耗尽。

## 13. 创新点 3：运动激励自适应过程噪声（`IMU_Processing.cpp` `UndistortPcl()`）

**问题**：高速/急转时点云去畸变误差和图像运动模糊增大，常数过程噪声使滤波器过度信任传播。

**实现**：前向传播每个 IMU 子步计算
`md = 0.5·min(|v|/v_max,1) + 0.5·min(|ω|/ω_max,1)`（v_max=5 m/s、ω_max=2 rad/s，可配），
EMA 平滑（0.9/0.1，约 50 ms 时间常数）得 `motion_degrade`，过程噪声乘 `1 + gain·md²`（gain=2.0，仅作用于姿态/速度噪声，不动 bias 噪声）。`motion_degrade` 同时喂给 VIO（协方差膨胀 + 门控）。

## 14. 创新点 5：LIO 信息增益驱动 VIO 信任度（`LIVMapper.cpp` `handleLIO()`）

**问题**：LIO 与 VIO 顺序更新同一 ESIKF，但各用固定观测噪声、互不感知——LiDAR 几何退化时视觉不会自动补位。

**实现**：LIO 更新前后取位姿 6×6 协方差逆的迹之差
`ΔI = max(tr(P_post⁻¹) − tr(P_pred⁻¹), 0)`，
与在线 EMA 标定的名义增益之比 clamp 到 [0.1, 2] 得 `vio_trust_factor_`，传给 VIO 乘进观测协方差：LIO 增益塌缩（退化）→ 因子变小 → VIO 协方差变小 → 视觉被信任更多。不需要额外传感器模型，协方差本身编码了信息贡献；配合创新点 1 的协方差一致性修正，该指标在退化场景下真实反映"LiDAR 没提供信息"。

## 15. 新增 ROS 参数（均有默认值，不改 config 也能跑）

```yaml
adaptive:
  degeneracy_aware_en: true    # 创新点1开关
  degeneracy_alpha: 10.0       # sigmoid 陡度
  degeneracy_tau: 0.05         # 退化特征值比阈值
  vio_quality_en: true         # 创新点2开关（关闭则回退原始固定协方差）
  min_gen_quality: 0.15        # 视觉点生成门控阈值
  min_ref_update_quality: 0.20 # 参考patch更新门控阈值
  motion_vel_max: 5.0          # m/s
  motion_gyr_max: 2.0          # rad/s
  motion_noise_gain: 2.0       # 过程噪声膨胀增益
```

两个总开关同时置 false 即可退化回近似原版行为（消融实验用）。

## 16. 诊断话题扩展

`/fast_livo2/lio_diag` 新增：

| Index | 字段 | 含义 |
|---:|---|---|
| `[8]` | `degeneracy_factor` | 方向性退化因子，0=完全退化，1=健康 |
| `[9]` | `normal_anisotropy` | 约束法向各向异性，0=单方向 |
| `[10]` | `motion_degrade` | 运动激励风险 0~1 |
| `[11]` | `vio_trust_factor` | LIO 信息增益比→VIO 信任因子 |

`/fast_livo2/vio_diag` 新增：

| Index | 字段 | 含义 |
|---:|---|---|
| `[6]` | `illum_quality` | 图像光照/纹理质量 0~1 |
| `[7]` | `visual_quality` | 含运动模糊风险的综合视觉质量 |
| `[8]` | `effective_img_cov` | 本帧实际使用的视觉观测协方差 |
| `[9]` | `motion_degrade` | 运动激励风险 0~1 |

原有 `[0]–[7]` / `[0]–[5]` 字段含义不变，health_monitor 无需改动即可继续工作；后续升级可直接消费新字段。

## 17. 第二阶段影响范围与验证建议

| 文件 | 改动 | 性质 |
|---|---|---|
| `include/voxel_map.h` | +6 成员 | 配置/诊断 |
| `src/voxel_map.cpp` | 法向散布分析、分块特征值分析、增量重映射、G 修正 | **算法** |
| `include/vio.h` | +9 成员 +1 函数声明 | 配置/诊断 |
| `src/vio.cpp` | `assessImageQuality()`、自适应协方差、双处 EKF 替换、门控+饥饿保护 | **算法** |
| `include/IMU_Processing.h` | +4 成员 | 配置 |
| `src/IMU_Processing.cpp` | 运动激励计算、过程噪声膨胀 | **算法** |
| `include/LIVMapper.h` | +12 成员 | 配置/状态 |
| `src/LIVMapper.cpp` | 信息增益计算、参数读取下发、指标传递、诊断扩展 | 调度/**算法** |

消融实验建议（论文对比）：

1. 原版（两开关关）vs 全开，HILTI / NTU_VIRAL / MARS_LVIG 数据集跑 ATE/RPE + 建图完整度。
2. 单独开创新点 1（几何退化场景，如走廊/空旷地面序列）。
3. 单独开创新点 2（弱光/曝光变化序列）。
4. 观察 `/fast_livo2/lio_diag[8][9]` 与退化路段的时间对齐，作图佐证退化检测有效性。

编译验证同第 8 节（需在 Ubuntu/ROS 环境）：`catkin_make --pkg fast_livo`。

## 18. Innovation 10: LIO residual robust kernel

This round replaces the old hard 5-sigma residual downweighting in `src/voxel_map.cpp`
with a configurable continuous robust kernel on normalized point-to-plane residuals.

- Config keys: `adaptive/robust_kernel_en`, `adaptive/robust_kernel_type`,
  `adaptive/robust_kernel_delta`, `adaptive/robust_kernel_min_weight`.
- Supported kernel types: `cauchy` and `huber`.
- Effect path: residual -> robust weight -> `R_inv(i)` -> `H^T R^-1 H` and EKF increment.
- Diagnostic: `/fast_livo2/lio_diag[19] = robust_kernel_mean_weight`.

Motivation: dynamic objects, bad voxel planes, local scan distortion, vegetation motion,
and fast-UAV motion can all create large residuals. A continuous robust kernel reduces
their influence smoothly instead of abruptly switching constraints to near-zero weight.

## 19. Innovation 11: scan-distortion-aware LIO weighting

This round adds an IMU-derived intra-scan distortion risk for UAV fast flight, sharp
turns and vibration-like motion.

- `ImuProcess` estimates scan start/end rotation and translation deltas after forward
  propagation, then publishes `scan_distortion_risk`, `scan_distortion_rot_risk`,
  `scan_distortion_trans_risk`, and `scan_time_span`.
- `VoxelMapManager` carries each LiDAR point's scan offset time into `PointToPlane`
  and downweights early-scan constraints more strongly under high scan risk.
- Effect path: intra-scan pose delta -> scan risk -> point-time weight -> `R_inv(i)`
  -> `H^T R^-1 H` and EKF increment.
- Config keys: `adaptive/scan_distortion_weight_en`,
  `adaptive/scan_distortion_rot_ref`, `adaptive/scan_distortion_trans_ref`,
  `adaptive/scan_distortion_noise_gain`, `adaptive/scan_distortion_min_weight`.
- Diagnostics:
  `/fast_livo2/lio_diag[20] = scan_distortion_risk`,
  `[21] = scan_distortion_mean_weight`,
  `[22] = scan_distortion_rot_risk`,
  `[23] = scan_distortion_trans_risk`,
  `[24] = scan_time_span`.

The adaptive LiDAR preprocessor now also uses `max(motion_degrade,
scan_distortion_risk)` when choosing the next frame's point-filter step, so aggressive
motion keeps more points instead of coarsening the cloud during degradation.

## 20. Innovation 12: LiDAR return-quality weighting

This round adds a point-level LiDAR return-quality model for weak reflectivity,
saturation/specular returns, long range and sparse local plane support.

- `pointWithVar` now carries per-point `intensity`, `range` and scan offset metadata
  into the point-to-plane residual path.
- `PointToPlane` stores `intensity_`, `range_` and a local `density_score_` derived
  from voxel plane support count.
- `VoxelMapManager` converts return quality into a continuous precision shrink before
  `H^T R^-1 H` is formed.
- Effect path: intensity/range/local density -> return-quality weight -> `R_inv(i)`
  -> `H^T R^-1 H` and EKF increment.
- Config keys: `adaptive/lidar_return_quality_en`, `adaptive/lidar_intensity_ref`,
  `adaptive/lidar_saturation_ref`, `adaptive/lidar_range_ref`,
  `adaptive/lidar_density_ref`, `adaptive/lidar_return_min_weight`.
- Diagnostics:
  `/fast_livo2/lio_diag[25] = lidar_return_quality_mean_weight`,
  `[26] = lidar_return_intensity_score`,
  `[27] = lidar_return_density_score`.

Motivation: UAV mapping can degrade even when compute is sufficient because weak,
over-saturated, far or sparsely supported LiDAR returns produce unstable plane
constraints. This change makes those constraints contribute less information to the
main LIO update instead of only reporting them after the map has already been updated.

## 21. Innovation 13: degradation-aware voxel map-write gate

This round closes a map-pollution gap: previous weighting innovations reduced the
effect of bad constraints during pose estimation, but the same degraded points could
still be inserted into the long-term voxel map.

- `pointWithVar` now carries map-match status, normalized residual, plane quality,
  local density, return-quality score and final map-write score.
- `build_single_residual()` writes the best residual consistency evidence back to
  the corresponding source point.
- `UpdateVoxelMap()` gates insertion/update before calling `UpdateOctoTree()`.
- Matched points are scored by residual consistency, return quality, voxel-plane
  quality and local density.
- Unmatched points are treated differently depending on whether they conflict with
  an already stable voxel plane or expand into a new/uninitialized area; high UAV
  motion/scan-distortion risk makes unmatched writes more conservative.
- Effect path: residual/return/structure/motion risk -> map-write confidence ->
  skip or accept `UpdateOctoTree()` -> cleaner future voxel planes and residuals.
- Config keys: `adaptive/map_write_gate_en`,
  `adaptive/map_write_residual_sigma`, `adaptive/map_write_min_score`,
  `adaptive/map_write_unmatched_risk_thresh`, `adaptive/map_write_min_return`.
- Diagnostics:
  `/fast_livo2/lio_diag[28] = map_write_accept_ratio`,
  `[29] = map_write_reject_num`,
  `[30] = map_write_mean_score`,
  `[31] = map_write_motion_risk`.

Motivation: UAV maps are easily polluted by moving objects, vegetation, dust, weak
returns and scan-distorted points. Rejecting those writes protects the map itself,
not just the current-frame pose estimate.

## 22. Innovation 14: view-angle and warp-aware VIO weighting

This round adds point-level visual observation weighting for aerial view-angle
degradation and strong patch deformation.

- `SubSparseMap` now stores one `point_weight` per tracked visual map point.
- `retrieveFromVisualSparseMap()` computes each visual observation weight from:
  current/reference surface incidence angle, affine warp determinant deformation,
  NCC quality and visual point lifecycle quality.
- `updateState()` and `updateStateInverse()` apply `sqrt(point_weight)` to both
  photometric residual rows and Jacobian rows, turning the normal equations into
  a weighted visual EKF update.
- `updateVisualMapPoints()` uses the newest observation (`obs_.front()`), and new
  long-term observations must pass current/reference view cosine, depth ratio,
  affine determinant, incidence and NCC admission checks when the same view-weight
  switch is enabled. Patch memory is allocated only after an update is needed, and
  an old observation is evicted only when its replacement is ready.
- Effect path: view angle / warp deformation / NCC / lifecycle -> point weight ->
  weighted `H^T H` and `H^T z` -> VIO state increment.
- Config keys: `adaptive/visual_view_weight_en`,
  `adaptive/visual_view_min_weight`, `adaptive/visual_view_min_cos`,
  `adaptive/visual_affine_det_ref`.
- Diagnostics:
  `/fast_livo2/vio_diag[17] = visual_view_weight_mean`,
  `[18] = visual_view_incidence_mean`,
  `[19] = visual_affine_score_mean`,
  `[20] = visual_low_view_points`.

Motivation: UAVs often observe ground/facades from rapidly changing, oblique
viewpoints. A patch can pass coarse retrieval but still be geometrically weak due
to grazing incidence or large affine deformation. This change reduces its impact
inside the VIO update instead of waiting for a hard photometric failure.

Small fix in Innovation 13: map-write return-quality gating now uses the raw
return-quality score, while LIO measurement weighting still uses the protected
minimum weight. This makes `adaptive/map_write_min_return` an actual pollution
filter instead of being masked by `lidar_return_min_weight`.

## 23. Innovation 15: cross-modal degradation arbitration

This round adds a joint LIO/VIO degradation gate for cases where both modalities
are weak at the same time, such as fast UAV motion through a low-texture/dark
area with sparse or geometrically degenerate LiDAR constraints.

- `LIVMapper` now computes a fused 0~1 LIO health score from directional
  degeneracy, normal anisotropy, incidence-angle weight, robust residual weight,
  scan-distortion weight, LiDAR return quality, LIO information gain and effective
  constraint count.
- `VIOManager` combines this LIO health with current visual quality, point-level
  view/warp weight and tracked-point count to form `cross_modal_degrade`.
- When both LIO and VIO are degraded, VIO measurement covariance is inflated by
  `cross_modal_cov_scale`, so the EKF trusts the photometric update less.
- Visual map writes (`generateVisualMapPoints`, `updateVisualMapPoints`,
  `updateReferencePatch`) are blocked when joint degradation exceeds the configured
  risk threshold, preventing bad frames from becoming long-term references.
- Historical note: this revision originally fed the latest joint degradation
  risk into LiDAR map writing. Section 38 supersedes that path: current LiDAR
  writes use current-frame LIO pose health, while joint risk remains a VIO policy.
- Config keys: `adaptive/cross_modal_gate_en`,
  `adaptive/cross_modal_cov_gain`, `adaptive/cross_modal_map_risk_thresh`,
  `adaptive/cross_modal_min_lio_points`,
  `adaptive/cross_modal_min_track_points`.
- Diagnostics:
  `/fast_livo2/vio_diag[21] = lio_health_score`,
  `[22] = cross_modal_visual_health`,
  `[23] = cross_modal_degrade`,
  `[24] = cross_modal_cov_scale`,
  `[25] = cross_modal_map_write_blocked`,
  `/fast_livo2/lio_diag[32] = cross_modal_lio_health`,
  `[33] = latest cross_modal_degrade`.

Motivation: previous changes made each sensor more degradation-aware, but a UAV
often fails when both streams are only marginally usable. This gate is a conservative
arbiter: a single healthy modality can still contribute, while simultaneous LIO/VIO
weakness reduces state-update confidence and blocks long-term map pollution.

## 24. Innovation 16: dynamic/transient map-pollution suppression

This round targets moving objects and transient structures that conflict with an
already stable voxel plane but may otherwise be reinserted as "new" map structure.

- `pointWithVar` now carries `map_dynamic_score`, and `PointToPlane` carries
  `dynamic_score_`.
- `build_single_residual()` computes a dynamic conflict score from normalized
  point-to-plane residual, plane quality and local support density whenever a point
  is evaluated against a stable voxel plane.
- Healthy low-residual constraints get zero dynamic penalty; the score rises only
  after a residual free-band, avoiding blanket downweighting.
- `StateEstimation()` converts this dynamic score into a precision weight on
  `R_inv(i)`, so suspected dynamic/transient points contribute less to
  `H^T R^-1 H`.
- `UpdateVoxelMap()` rejects map writes whose dynamic score exceeds the configured
  threshold, especially for unmatched points that conflict with an existing stable
  plane.
- High scan/self-motion no longer amplifies the same residual into stronger dynamic
  evidence. `dynamic_motion_gain` now attenuates single-frame classification
  confidence, while scan-distortion and map-motion gates independently protect the
  state and map. This avoids labelling deskew/timing error as object motion twice.
- Config keys: `adaptive/dynamic_object_filter_en`,
  `adaptive/dynamic_residual_sigma`, `adaptive/dynamic_min_weight`,
  `adaptive/dynamic_map_reject_score`, `adaptive/dynamic_motion_gain`.
- Diagnostics:
  `/fast_livo2/lio_diag[34] = dynamic_mean_score`,
  `[35] = dynamic_mean_weight`,
  `[36] = dynamic_reject_num`.

Motivation: UAV mapping is easily polluted by people, cars, vegetation, dust and
short-lived reflective objects. Robust residual kernels protect the current pose
update, but they do not by themselves prevent those points from entering the map.
This change uses temporal consistency with stable voxel planes to protect both the
current LIO update and future map quality.

## 25. Innovation 17: temporal/exposure-motion degradation gate

This round adds a timing and exposure consistency gate for UAV cases where image
timestamps, LIO/VIO pairing, exposure jumps and aggressive motion jointly degrade
direct visual tracking.

- `handleVIO()` now feeds the real VIO image timestamp into `processFrame()` and
  records the paired LIO/VIO time gap. Previously the unused `img_time` argument was
  filled from `last_lio_update_time`, which made frame-time reasoning impossible.
- `processFrame()` now safely returns on empty images instead of continuing into
  `resize()`/photometric processing.
- `VIOManager` estimates frame-interval jitter from the image timestamp stream,
  paired LIO/VIO time-gap risk, inverse-exposure jump risk and a high-motion
  amplification term.
- The fused `temporal_degrade` directly scales `current_frame_img_cov`, affecting
  the visual EKF normal equations through the existing covariance path.
- High temporal risk blocks visual map writes (`generateVisualMapPoints`,
  `updateVisualMapPoints`, `updateReferencePatch`) so timestamp/exposure-corrupted
  frames cannot become long-term references.
- Temporal risk gates VIO covariance and visual-map writes. It is retained in
  LIO diagnostics for correlation analysis but no longer directly gates the
  next LiDAR map write from a stale visual timestamp.
- Config keys: `adaptive/temporal_degrade_en`,
  `adaptive/temporal_jitter_ref`, `adaptive/temporal_lio_vio_gap_ref`,
  `adaptive/temporal_exposure_jump_ref`, `adaptive/temporal_cov_gain`,
  `adaptive/temporal_motion_gain`, `adaptive/temporal_map_risk_thresh`.
- Diagnostics:
  `/fast_livo2/vio_diag[26] = temporal_degrade`,
  `[27] = temporal_jitter_score`,
  `[28] = temporal_lio_vio_gap_score`,
  `[29] = temporal_exposure_jump_score`,
  `[30] = temporal_cov_scale`,
  `[31] = temporal_map_write_blocked`,
  `[32] = lio_vio_time_gap`,
  `/fast_livo2/lio_diag[37] = latest temporal_degrade`.

Motivation: fast UAV flight turns small timestamp skew, frame-rate jitter and
exposure discontinuities into spatial residual bias. This gate makes those effects
visible and, more importantly, reduces their influence on both VIO state updates
and future map references.

## 26. Innovation 18: quality/age-aware reference patch selection

This round targets a VIO failure mode that is especially common during UAV mapping:
an old or low-quality reference patch can stay attached to a visual point even after
the current view angle, exposure, texture or motion conditions have changed. The
photometric residual may then pull the EKF toward a stale appearance model and
pollute future visual-map updates.

- `Feature` construction now initializes `id_`, `score_`, `mean_` and
  `inv_expo_time_`, so reference aging, deletion by score and exposure scoring do
  not read uninitialized values.
- `VIOManager::computeReferencePatchScore()` computes a 0~1 reference quality
  score from frame age, current/reference view direction, surface incidence angle,
  inverse-exposure jump, patch contrast, stored feature score and visual point
  lifecycle quality.
- `VIOManager::selectReferencePatch()` chooses the best reference with hysteresis,
  keeping the current reference unless a candidate is clearly better, and marking
  stale/low-score references for diagnostics.
- `retrieveFromVisualSparseMap()` now rejects points whose best reference patch is
  below `adaptive/ref_patch_min_score`, and multiplies the VIO residual weight by
  the selected reference quality. This changes the main EKF measurement chain
  rather than only monitoring bad references.
- `updateReferencePatch()` now fixes the original NCC scoring robustness problems:
  cached means are initialized before use, each pairwise NCC denominator is
  protected, non-finite values are ignored, and the adaptive reference quality is
  blended into the reference score when enabled.
- The residual weighting path in both inverse-composition and forward VIO updates
  now respects reference-patch weights even if the older view/warp weighting switch
  is disabled.
- Config keys: `adaptive/ref_patch_adaptive_en`,
  `adaptive/ref_patch_max_age`, `adaptive/ref_patch_min_score`,
  `adaptive/ref_patch_min_weight`, `adaptive/ref_patch_view_cos_min`,
  `adaptive/ref_patch_exposure_ref`, `adaptive/ref_patch_switch_margin`.
- Diagnostics:
  `/fast_livo2/vio_diag[33] = ref_patch_mean_score`,
  `[34] = ref_patch_switch_count`,
  `[35] = ref_patch_stale_count`,
  `[36] = ref_patch_low_score_count`.

Motivation: light changes, fast yaw, oblique ground/facade observation and motion
blur can make a previously good patch become a bad reference. This change makes
reference-patch choice a degradation-aware part of the VIO front-end, reducing
stale visual constraints before they enter the EKF and before they become long-term
map references.

## 27. Stabilization pass after external review

This round does not add another major algorithm branch. It tightens the existing
degradation-aware pipeline so the current innovations are more stable, reproducible
and easier to defend experimentally.

- `Preprocess::computePostDispatchStats()` now computes `last_output_ratio` and
  uses the estimated pre-downsample valid count, not the post-sampling output
  count, for the LiDAR quality density term. This reduces self-feedback where a
  larger `point_filter_num` would make the system think LiDAR quality had dropped.
- `Preprocess::decideNextFilterNum()` now has bad/good hysteresis counters:
  degraded scenes must persist before the sampler keeps more points, while healthy
  scenes must persist longer before the sampler coarsens for efficiency.
- `LIVMapper` now reads `imu/b_gyr_cov` and `imu/b_acc_cov` from YAML and actually
  forwards them to `ImuProcess`; these parameters were previously present in YAML
  but hard-coded in source.
- `motion_vel_max`, `motion_gyr_max` and `motion_noise_gain` are clamped before
  use, and `ImuProcess` keeps a second local guard against division by zero.
- Directional degeneracy suppression now clamps the sigmoid exponent and applies
  `adaptive/degeneracy_min_factor`, preventing a pose direction from being almost
  completely removed by numerical saturation.
- LIO precision weighting now applies `adaptive/total_lio_min_weight` after all
  per-factor weights are multiplied, avoiding constraint starvation from an overly
  long weight chain.
- `/fast_livo2/lio_diag` now exposes:
  `[38] = total_lio_effective_weight_mean`,
  `[39] = total_lio_low_weight_ratio`,
  `[40] = point_filter_num`,
  `[41] = last_raw_count`,
  `[42] = last_valid_count`,
  `[43] = last_output_count`,
  `[44] = last_valid_ratio`,
  `[45] = last_output_ratio`,
  `[46] = last_lidar_quality`,
  `[47] = last_near_ratio`,
  `[48] = last_far_ratio`,
  `[49] = last_intensity_mean`.

YAML note: sensor-specific thresholds such as LiDAR intensity/range/density,
cross-modal map-write thresholds and dataset-specific preprocessing limits were
not blindly changed, because the project contains real lab flight parameters.
Only source-backed safety parameters were added:
`adaptive/degeneracy_min_factor`, `adaptive/total_lio_min_weight`,
`adaptive/preprocess_bad_hysteresis_frames` and
`adaptive/preprocess_good_hysteresis_frames`.

## 28. Current review fixes

- `VoxelMapManager::StateEstimation()` now resets
  `total_lio_effective_weight_mean_` and `total_lio_low_weight_ratio_` at the
  beginning of each ESIKF iteration, so a no-constraint or skipped-weight path
  cannot leak stale final-weight diagnostics from the previous frame.
- `Preprocess::decideNextFilterNum()` formatting was cleaned around the
  hysteresis branch; behavior is unchanged.
- Diagnostic note: the early baseline sections only document the original 8 LIO
  and 6 VIO diagnostic fields. Intermediate sections retain their historical
  layouts; the authoritative current contract is the 88/45 layout at the top of
  this file and in `src/LIVMapper.cpp`.

## 29. Innovation 19: high-frequency IMU vibration degradation

UAV vibration is different from ordinary high-speed motion: velocity and yaw rate
may be moderate, but propeller/body vibration can still corrupt LiDAR deskewing
and visual patch alignment. This innovation adds a lightweight high-pass IMU
vibration model inside the main propagation chain.

- `ImuProcess` now maintains low-pass acceleration/gyro states and computes
  high-frequency residual risks:
  `vibration_acc_risk`, `vibration_gyr_risk`, and smoothed
  `vibration_degrade`.
- `adaptive/vibration_degrade_en` provides a true ablation switch. When disabled,
  the original motion-risk weighting and scan-distortion composition are restored.
- The low-pass and risk smoothing factors are derived from IMU `dt` and
  `adaptive/vibration_lpf_tau`, so behavior is stable across different IMU rates.
- IMU process noise inflation now uses both motion excitation and vibration:
  `Q *= 1 + motion_noise_gain * motion_degrade^2 +
  vibration_noise_gain * vibration_degrade^2`.
- Scan-distortion risk now includes a vibration component, so LIO residual
  weighting reacts to high-frequency shake even when net scan rotation/translation
  is not large.
- VIO receives `max(motion_degrade, vibration_degrade)` for adaptive covariance,
  visual map-write gates and reference-patch protection.
- Adaptive LiDAR preprocessing and voxel-map write risk also use the vibration
  risk, so the system keeps more points and writes more conservatively under
  strong vibration.
- Config keys:
  `adaptive/vibration_degrade_en`, `adaptive/vibration_acc_ref`,
  `adaptive/vibration_gyr_ref`, `adaptive/vibration_noise_gain`,
  `adaptive/vibration_lpf_tau`.
- Diagnostics:
  `/fast_livo2/lio_diag[50] = vibration_degrade`,
  `[51] = vibration_acc_risk`,
  `[52] = vibration_gyr_risk`.

## 30. Innovation 20: altitude/range-distribution adaptive LiDAR weighting

A fixed LiDAR range penalty can be harmful for aerial mapping. During
high-altitude downward-looking flight, most valid ground returns may lie beyond
the configured `lidar_range_ref`; uniformly downweighting them can starve the
LIO update exactly when geometric diversity is already weak.

- `VoxelMapManager::updateAdaptiveLidarRangeReference()` computes a robust range
  quantile from the configured candidate or matched constraint source.
- The configured `adaptive/lidar_range_ref` remains the minimum/base reference.
  The adaptive logic can only raise it, never reduce or overwrite the
  dataset/lab-flight baseline.
- The target is bounded by `adaptive/lidar_range_ref_max` and updated once per
  LIO frame with EMA, avoiding abrupt weight changes and dependence on the number
  of ESIKF iterations.
- Fewer than 20 valid samples in the active source do not update the reference.
- `computeLidarReturnWeight()` uses the effective reference in the core LIO
  precision and voxel-map write paths.
- Config keys:
  `adaptive/lidar_range_adaptive_en`, `adaptive/lidar_range_candidate_source_en`,
  `adaptive/lidar_range_ref_max`,
  `adaptive/lidar_range_quantile`, `adaptive/lidar_range_ref_ema_alpha`.
- Diagnostics:
  `/fast_livo2/lio_diag[53] = lidar_range_ref_effective`,
  `[54] = lidar_range_quantile_value` (last active-source quantile accepted by EMA).

## 31. Motion-degradation model stabilization

The existing Innovation 3 motion model was also tightened during this review:

- `adaptive/motion_degrade_en` is now a true ablation switch for the
  velocity/gyro/jerk degradation model and its process-noise inflation.
- The previously hard-coded `50 m/s^3` jerk saturation value is exposed as
  `adaptive/motion_jerk_ref`.
- Motion-risk smoothing now uses IMU `dt` and
  `adaptive/motion_risk_tau` instead of fixed `0.9/0.1` coefficients, keeping the
  response approximately invariant across IMU sample rates.
- The no-IMU path explicitly clears `motion_degrade`, preventing stale risk from
  leaking into VIO covariance, preprocessing or map-write gates.
- Scan-distortion composition respects both the motion and vibration ablation
  switches, so disabled innovations do not continue affecting the main chain
  indirectly.

## 32. Static-safety fixes from core-source review

- `ImuProcess::IMU_mean_acc_norm`, `first_lidar_time`, `unbiased_gyr` and
  `lidar_type` now have deterministic defaults.
- `LIVMapper::initializeComponents()` explicitly aligns
  `ImuProcess::lidar_type` with `Preprocess::lidar_type`; the no-IMU
  undistortion path previously read an unassigned value.
- `VoxelMapManager` timing and feature-count diagnostics are initialized to zero.
- The invalid configuration-less `VoxelMapManager()` default constructor is
  disabled; all instances must provide `VoxelMapConfig` and the voxel-map owner.
- `VoxelOctoTree` pointer/leaf/scalar members now have safe defaults, and copying
  is disabled to prevent accidental double deletion of owned child/plane
  pointers.
- `VIOManager` pointers, dimensions, flags and timing fields now have
  deterministic defaults; copying is disabled because it owns `visual_submap`,
  warp and feature-map allocations.
- `initializeVIO()` safely replaces an existing submap and validates the camera.
  Grid dimensions now use floating-point division before `ceil`, fixing the
  previous truncated final image row/column.
- VIO iteration count and `img_point_cov` are clamped to valid positive ranges
  before component initialization.

## 33. Innovation 21: correlated-risk-aware VIO covariance fusion

Image quality, motion, temporal consistency and cross-modal health are not
independent. The previous implementation multiplied the temporal and
cross-modal covariance scales after image/motion inflation, even though both
auxiliary risks reuse motion, exposure or visual-quality evidence. In the worst
case this could inflate visual covariance by roughly three orders of magnitude
and effectively remove VIO from a double-degraded update.

- `base_frame_img_cov` preserves the covariance produced by image quality,
  motion/vibration and LIO information arbitration.
- With `adaptive/vio_cov_fusion_en=true`, temporal and cross-modal auxiliary
  scales are fused with `max(scale_temporal, scale_cross_modal)` instead of
  multiplication. This keeps the strongest warning without counting shared
  evidence twice.
- The final covariance is bounded by
  `adaptive/vio_cov_scale_max * img_point_cov`, preventing complete visual
  measurement starvation while still allowing strong downweighting.
- Setting `adaptive/vio_cov_fusion_en=false` restores the legacy multiplicative
  path for ablation.
- Diagnostics:
  `/fast_livo2/vio_diag[37] = base_frame_img_cov`,
  `[38] = fused_cov_scale`,
  `[39] = vio_cov_clamped`.

## 34. 本轮代码审查修复

本轮先修复了会影响退化判断、消融可信度或数值安全的明确问题，再引入
Innovation 22：

- `/fast_livo2/vio_diag[0]` 改为发布当前配对图像的真实 `vio_time`，不再复用
  `last_lio_update_time`。VIO 诊断、时间抖动和 LIO/VIO 间隔指标现在使用一致的
  图像时间轴。
- 所有相关体素索引使用 `floor(world_coordinate / voxel_size)`。这修复了负坐标
  非整数点和负向体素边界处由截断加手工减一造成的错格问题。
- LIO 邻格搜索改为用世界坐标点与 `voxel_center_ +/- quater_length_` 比较；不再
  将无量纲体素索引与以米为单位的中心和长度直接比较。
- 残差递归在解引用前检查 `current_octo` 和 `plane_ptr_`，并拒绝非有限或非正的
  残差方差，避免稀疏/未初始化叶节点触发空指针或无效概率计算。
- `calcBodyCov()` 现在保护非有限和近零长度点，规范化 range/angular 噪声输入，
  使用稳定的正交切平面基，并显式对称化输出协方差，避免除零和 NaN/Inf 进入
  ESIKF。
- `adaptive/normal_anisotropy_weight_en` 和
  `adaptive/plane_quality_weight_en` 分别控制法向各向异性噪声膨胀和拟合平面质量
  精度缩放；关闭后对应因子严格为 1，补齐此前缺失的消融路径。
- 点世界协方差现在包含 LiDAR-IMU 外参旋转、当前世界姿态、旋转-平移先验
  交叉协方差；点到平面噪声和入射角也与当前迭代状态在同一线性化点计算。
- 视觉子图少于 30 点时，跨模态健康度不再使用由地图自身缺失造成的
  `track_health`。恢复仍须通过图像质量、视角质量、时序门控和绝对最低质量，
  但不会因“无点 -> 禁止写点 -> 继续无点”形成自维持饥饿。
- `adaptive/cross_modal_gate_en` 的协方差作用不再依赖
  `adaptive/vio_quality_en`；关闭图像质量自适应时，独立跨模态消融仍会进入 VIO
  EKF，而地图门控语义保持不变。
- `VisualPoint::ref_patch` 显式初始化为空；关闭法向模式时释放 `warp_map` 所有
  `Warp` 后再清容器；空/过小标准点云不再入队，ONLY_LIO/ONLY_LO 会消费历史
  坏帧且每周期只保留当前 `MeasureGroup`，避免坏首帧卡死和长期消息堆积。
- 任一 LiDAR、IMU 或图像时间回退都会请求协调 reset；主线程统一清空状态、两类
  地图、IMU/VIO 时序、风险状态、高频传播状态和全部传感器队列，避免跨 rosbag
  会话混入旧地图、旧 carry 或旧传播时间水位。
- 对称矩阵正则化只容忍数值舍入量级的负特征值；明显非正定输入会触发失败，
  不再被静默抬成极小正数并制造伪精度。任一 ESIKF 迭代出现致命数值失败时，
  状态和协方差原子回滚到本帧传播先验，且该帧禁止写入 voxel map。
- Historical note: the first fix disabled `-ffast-math` only for `lio`. The
  current CMake contract removes the global ARM flag and applies
  `-fno-fast-math` to every guarded core target.
- 参数读取新增有限性和范围保护。重点包括退化 sigmoid 参数、先验特征值地板、
  VIO 质量阈值和协方差上限、运动安全阈值，以及满足 `near_dist < far_dist` 的
  预处理距离约束。非有限输入回退到源码默认值，不修改 YAML 中已有的真实试飞
  标定和阈值。

当前诊断总长度为：

- Historical layout at this section: LIO 67 (`[0..66]`), VIO 40 (`[0..39]`).
  The current authoritative layout is 77/45 as stated at the top of this file.

## 35. Innovation 22: prior-whitened coupled 6DoF degeneracy update

旧退化检测分别分解旋转和平移两个 `3x3` 信息块，无法看到完整信息矩阵
`I = H^T R^-1 H` 的旋转-平移非对角块。因此，两个对角块都看似满秩时，仍可能
存在形如 `[delta_theta; delta_t]` 的耦合不可观方向。直接分解原始 `6x6` 又会受
弧度和米的量纲差异影响。

Innovation 22 使用位姿先验协方差构造无量纲完整 `6x6` 分析：

```text
P6 = L L^T
J  = L^T I L
g  = L^T b,    b = H^T R^-1 z
J  = V diag(lambda_i) V^T
```

`P6` 在分解前对称化，并用
`adaptive/degeneracy_prior_eigen_ratio_floor` 对相对特征值设置 SPD 地板。
`J` 保留完整旋转-平移耦合，同时通过先验尺度消除 rad/m 直接比较造成的偏置。
每个白化方向同时计算相对谱健康度（归一化有界 sigmoid）与绝对先验信息增益
`q_i = lambda_i / (1 + lambda_i)`，取两者较大值后映射到
`[adaptive/degeneracy_min_factor, 1]`。这样严格零模态落到配置下限，而绝对信息
很强但相对最大特征值较小的方向不会被误杀。该可靠性 `w_i` 同步作用于信息和
score：

```text
J_eff = V diag(w_i * lambda_i) V^T
g_eff = V diag(w_i) V^T g
```

随后将 `J_eff/g_eff` 变换回位姿坐标得到同一有效似然 `I_eff/b_eff`。该有效似然
嵌入完整状态信息系统后统一求解状态增量、全状态 Kalman 增益和后验协方差；速度、
偏置、重力等状态通过先验交叉协方差一致更新。不再只缩放六维位姿增量，也不再
对已求出的增益局部事后缩放。

消融和失败回退如下：

- `adaptive/degeneracy_aware_en=false`：不施加方向退化权重。
- `adaptive/degeneracy_aware_en=true` 且
  `adaptive/coupled_degeneracy_en=false`：恢复 Innovation 1 的旋转/平移两个
  `3x3` 分块检测路径，用于直接消融对比。
- `adaptive/coupled_degeneracy_en=true`：启用先验白化完整 `6x6` 耦合路径。
- 先验或白化特征分解失败、矩阵非有限或有效约束不足时，设置数值回退标志并走
  受保护的旧路径，不把无效矩阵送入状态或协方差更新。

新增 YAML 参数均有源码默认值和边界保护：

- `adaptive/coupled_degeneracy_en`，默认 `true`。
- `adaptive/degeneracy_prior_eigen_ratio_floor`，默认 `1.0e-9`，限制在
  `[1.0e-12, 1.0e-3]`。
- `adaptive/normal_anisotropy_weight_en`，默认 `true`。
- `adaptive/plane_quality_weight_en`，默认 `true`。
- `adaptive/degeneracy_alpha` 限制在 `[0.1, 100]`，
  `adaptive/degeneracy_tau` 限制在 `[1.0e-6, 1]`，
  `adaptive/degeneracy_min_factor` 限制在 `[0, 1]`。

`/fast_livo2/lio_diag` 新增以下连续字段：

| Index | Field | Meaning |
|---:|---|---|
| `[55]` | `coupled_active` | 本帧实际使用完整白化 `6x6` 耦合路径 |
| `[56]` | `whitened_min_eigenvalue` | `J` 的最小特征值，无量纲 |
| `[57]` | `whitened_max_eigenvalue` | `J` 的最大特征值，无量纲 |
| `[58]` | `whitened_condition_ratio` | `lambda_min / lambda_max`，越低表示方向退化越强 |
| `[59]` | `rot_trans_coupling` | 旋转-平移非对角信息块的归一化耦合强度 |
| `[60]` | `mean_direction_factor` | 六个白化方向可靠性因子的均值 |
| `[61]` | `numerical_fallback` | 本帧任一轮触发白化回退或致命数值回滚时为 `1` |

原有 `[0..54]` 字段顺序和语义保持不变，便于旧实验脚本继续读取固定前缀。

数值基准检查覆盖了三类关键性质：`H=[I I]` 的三个耦合零模态、位置单位从米
切换为厘米时白化谱不变，以及旧分块消融路径 Joseph 协方差的对称正定性。

## 36. SO(3) covariance reset after state injection

`StatesGroup` represents attitude updates and errors on the right:
`R_new = R * Exp(delta_theta)` and `error = Log(R_ref^T * R)`. The posterior
covariance produced at the final ESIKF linearization therefore cannot be copied
unchanged after the mean injection. It must be transported to the new tangent
coordinates with the SO(3) right Jacobian:

```text
T_reset = diag(J_r(delta_theta), I_16)
P_new   = T_reset P_candidate T_reset^T
```

The propagated covariance is frozen once per LiDAR frame. Before every IEKF
relinearization it is transported, without incorporating the measurement again,
from the propagated tangent into the current iterate tangent:

```text
delta_iter = current_state - propagated_state
P_iter     = T(delta_iter) P_propagated T(delta_iter)^T
```

`P_iter` is used consistently by point-world covariance, coupled-`6x6` prior
whitening, the posterior information solve and the legacy Joseph branch. The
final reset is applied to the full `19x19` covariance, so rotation-position,
rotation-velocity, bias and gravity cross-covariances remain consistent. It is
shared by the unweighted baseline, legacy split-`3x3`, and coupled-`6x6` update
paths because this is a manifold-coordinate correctness fix, not an optional
degradation model. Non-finite reset results trigger the existing atomic frame
rollback before any map write.

`test/so3_covariance_reset_test.cpp` verifies the right-Jacobian sign by finite
difference, the inverse prior-residual Jacobian relation, full cross-covariance
transport and SPD preservation. The three `Exp()` overloads now share a stable
Rodrigues/Taylor implementation; sub-`1e-5` state increments are no longer
silently discarded while the covariance is transported as if they were applied.
This innovation extended the then-current LIO layout to 71 fields (`[0..70]`);
later convergence work extends the authoritative layout to 77 fields:

| Index | Field | Meaning |
|---:|---|---|
| `[67]` | `covariance_reset_angle` | final injected rotation used by the reset (rad) |
| `[68]` | `covariance_min_eigenvalue` | posterior minimum eigenvalue after reset and regularization |
| `[69]` | `covariance_symmetry_error` | relative asymmetry before final regularization |
| `[70]` | `covariance_relinearization_angle` | current-vs-propagated rotation used for prior transport (rad) |

## 37. LiDAR intensity contract and feature-output safety

- Every handler now keeps the driver-provided LiDAR intensity/reflectivity in
  its native numeric unit and sanitizes non-finite/negative values. Pandar128 no
  longer divides its byte intensity by 255 before the same value reaches the
  native-unit `80/240` return model.
- Feature extraction value-initializes averaged points and copies/averages
  intensity together with XYZ and point time. This removes zero, indeterminate
  or stale return values from the LIO and map-write paths.
- L515 explicitly emits zero intensity and marks return intensity unavailable.
  Both preprocessing and voxel-map return quality then redistribute the score
  over the available range/density evidence instead of treating a missing
  channel as a weak return.
- Preprocess reuses the existing bounded `adaptive/lidar_intensity_ref` and
  `adaptive/lidar_saturation_ref` values. The independent
  `adaptive/preprocess_intensity_quality_en` switch controls only whether this
  evidence affects adaptive sampling; diagnostics remain available, and
  `adaptive/preprocess_adaptive_en=false` still restores fixed sampling.

## 38. Final-pose map metadata and current-frame map risk

- `RefreshMapWriteMetadata()` clears the previous linearization's match state
  and performs one correspondence-only query at the final accepted IEKF pose.
  It does not solve again or alter filter mean, covariance, information, or LIO
  diagnostics. `MapWriteMetadata.RefreshClearsRejectedLinearizationStateWithoutChangingFilterState`
  tests that separation.
- LiDAR map-write quality risk is now `1 - current_lio_health`. It is evaluated
  after the current LIO update and before the current cloud is inserted. The
  previous joint VIO risk was both stale and unable to represent a bad current
  LIO pose when the previous visual frame was healthy.
- Disabling `adaptive/normal_anisotropy_weight_en` supplies neutral normal health
  to the cross-modal policy, so the switch no longer changes VIO/map behavior
  through an undocumented side path.
- LIO diagnostic suffix:

| Index | Field |
|---:|---|
| `[71]` | current-frame map-write pose-quality risk |
| `[72]` | consecutive low-write frames |
| `[73]` | frontier-recovery active flag |
| `[74]` | points admitted by the recovery budget |
| `[75]` | severe residual conflicts rejected before the weight floor |
| `[76]` | fused robust/dynamic residual-consistency weight |

## 39. Fixed-support VIO acceptance and visual-map transaction

- All direct image accesses validate the exact support implied by patch size,
  pyramid level and affine search level. Raw pointer addressing uses
  `cv::Mat::step`, not the camera width, and invalid affine/interpolation inputs
  are rejected before dereference.
- Each pyramid optimization freezes the point support from its first valid
  evaluation. A candidate that moves any required point out of positive depth or
  safe image support is rejected rather than receiving a lower mean residual by
  dropping difficult patches.
- The loop permits at most `max_iterations` solves, followed by one residual-only
  validation of the last injected state. Worsening final candidates restore the
  accepted mean, covariance candidate, anchor, residuals and support mask.
- Lifecycle counters, bad-point state and reference-patch pointers are
  snapshotted before retrieval. Destructive observation cleanup is delayed until
  covariance success; failure or no valid visual linearization restores the
  snapshot. Existing-point observation and reference writes require an accepted
  visual update, while an empty visual map may still bootstrap new points from a
  healthy LIO pose.
- VIO diagnostic suffix:

| Index | Field |
|---:|---|
| `[40]` | final SO(3) covariance-reset angle |
| `[41]` | committed covariance minimum eigenvalue |
| `[42]` | covariance symmetry error before regularization |
| `[43]` | numerical mean/covariance rollback flag |
| `[44]` | accepted fixed-support visual update flag |

`test/vio_warp_test.cpp` now verifies complete finite sampling and explicit
rejection of singular and partially out-of-frame affine warps. Transactional
visual-map rollback still relies on source review and integration coverage; it
does not yet have a dedicated ownership-fault-injection unit test.

## 40. Parameter and experiment contract closure

- All 72 adaptive floating-point parameters use finite fallback plus explicit
  lower/upper bounds. Existing finite YAML values are unchanged.
- `scripts/validate_adaptive_config.py` rejects duplicate YAML keys, non-finite
  values and any mismatch between the 109 source reads and the six mapping YAMLs.
- Voxel size/topology/layer-vector constraints and 3/9/3/9 extrinsic vector
  lengths are validated before use. Pandar timestamps are filtered before sort.
- ARM builds no longer use global `-ffast-math`; all estimator targets explicitly
  retain IEEE finite-value semantics. CMake honors an explicit build type so the
  strict `RelWithDebInfo` ASan script is meaningful.
- Replay contract v3 records SHA-256 fingerprints for the input bag, mapping
  config, camera config, parameter dump and the executable/shared-library
  runtime bundle. It uses simulated time and `rosbag --clock`. The analyzer
  rejects mode/config/hash differences, incomplete replay coverage, schema
  mismatches, NaN/Inf, altered manifests and altered parameter dumps.
- Optional analyzer mode subsets ignore other valid mode directories instead of
  falsely marking them invalid. Historical contract-v1/v2 runs remain readable
  but are intentionally not accepted as contract-v3 evidence.

Registered tests are exactly:

- `test/so3_covariance_reset_test.cpp`: five SO(3)/covariance tests plus one
  Eigen/PCL ABI contract test.
- `test/voxel_map_recovery_test.cpp`: three voxel-spectrum tests, two
  recovery-policy tests and one final-pose map-metadata isolation test.
- `test/vio_warp_test.cpp`: one complete-warp acceptance test plus singular and
  partially out-of-frame rejection tests.

## 41. Cold core-source audit and compile closure

This pass added no degradation score and did not run rosbag. It re-read the
current worktree and fixed confirmed correctness or engineering-contract faults
in the existing implementation:

- VIO no longer caches affine warps by `Feature::id_` (a frame id shared by all
  observations created in that image). Every visual point builds its own warp.
  Affine construction and sampling return `bool`; singular, non-finite, partial
  and out-of-frame patches are rejected instead of entering the EKF as valid
  zero intensity. New level-0 patches are stored with level 0, projection is
  checked before floating-to-integer conversion, and an empty image clears the
  frame transaction, support mask, lifecycle counters and accepted-update flag.
- `Feature` is non-copyable because it uniquely owns `patch_`. The obsolete raw
  `Warp` cache and its manual allocation/deletion path were removed.
- Voxel plane fitting uses `SelfAdjointEigenSolver` with deterministic sorted
  eigenvectors, finite checks and a minimum spectral gap. Repeated points and
  collinear distributions are no longer misclassified as planes or allowed to
  divide by a repeated eigenvalue. Plane covariance is explicitly symmetrized.
- Gravity-frame alignment now applies the matching full `19x19` covariance
  congruence to world-frame position, velocity and gravity blocks. The mean and
  covariance are committed transactionally only after finite validation.
- An invalid propagated LIO mean/covariance is distinguished from an ordinary
  update failure. It requests the existing coordinated estimator reset instead
  of repeatedly rolling back to the same invalid prior.
- LiDAR feature preprocessing initializes all geometric scratch values and
  never computes edge intersections from an uninitialized/zero-length neighbor.
  XT32 point times use finite frame-relative milliseconds in both feature modes;
  Robosense chooses a finite frame-time origin. `lidar_time_offset` now applies
  consistently to standard and Livox callbacks.
- Non-finite time offsets, covariance parameters, blind distances and voxel
  filter sizes fall back to their existing defaults. Finite laboratory YAML
  values are unchanged. Coordinated reset restores the configured preprocess
  sampling stride and HILTI frame-decimation phase.
- The ablation analyzer requires full `88/45` diagnostic widths before indexed
  access, rehashes the declared input bag, and validates trajectory start, end
  and duration. The replay runner computes the bag SHA-256 for each formal run
  instead of trusting a metadata-only cache.
- CMake uses the Sophus package result, explicitly links OpenMP to each core
  library, and records the direct `vio -> lio` shared-library dependency.

Verification for this pass:

- No rosbag, mapping node or dataset replay was run, by request.
- Noetic Release build succeeded in
  `/home/fastlivo/codex_catkin_ws_20260710`.
- Fifteen gtest cases passed; `catkin_test_results` reported 30 records with
  zero errors/failures/skips. This includes three affine-warp tests and three
  repeated/collinear/planar voxel-spectrum tests.
- `RelWithDebInfo` ASan build succeeded in
  `/home/fastlivo/codex_asan_ws_20260710`; all tests passed with address-error
  detection enabled. LeakSanitizer separately reports 368 bytes rooted only in
  VTK/PCL library initialization, so the address test was rerun with leak
  detection disabled to isolate project memory-access faults.
- Cppcheck 2.19 warning/portability, `git diff --check`, Python byte-compilation,
  Bash syntax, YAML duplicate/non-finite checks and all 109 adaptive-key mappings
  passed. The VM still emits an RPATH ordering warning because identical Sophus
  binaries exist in `/usr/local/lib` and the third-party build directory; their
  SHA-256 values match and the built executable resolves all runtime libraries.
