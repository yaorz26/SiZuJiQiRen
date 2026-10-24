# Go2 server-side single-robot SLAM

This package runs motion-compensated cloud accumulation, odometry TF, and `slam_toolbox` on the
computer receiving Go2 data. It publishes no robot movement requests.

The input is `/utlidar/cloud_base` in `base_link` and `/utlidar/robot_odom` describing
`odom -> base_link`. A time bridge estimates one common clock offset from odometry
and applies it to both streams on separate output topics. Original input topics and
recorded bags are unchanged.

## Live input

Connect the robot through the verified interface and source its environment:

```bash
source /home/yaorz/unitree_ros2/setup.sh
source /home/yaorz/unitree_go2_ws/install/setup.bash
ros2 launch go2_slam_bringup go2_slam.launch.py
```

Keep the robot stationary for the initial check. Physical movement is performed
by the user with the remote controller after the assistant confirms readiness.

Outputs:

- `/go2/slam/cloud_base`, `/go2/slam/odom`: timestamps shifted by the same offset.
- `/go2/slam/time_diagnostics`: calibrated offset and cloud gap observations.
- `/scan`: 2D scan in `base_link`.
- `/go2/slam/scan_diagnostics`: accumulation coverage, dropped frames and gap resets.
- `/tf`: `map -> odom -> base_link`.
- `/map`: occupancy grid containing free, occupied, and unknown cells.

`use_sim_time` defaults to `false` for live input. `alignment_mode` supports `auto`,
`fixed` (with `fixed_offset_seconds`), and `passthrough`. Never hard-code the measured
2173-second offset from one session into a later session.

## Replay with map saving and reload verification

```bash
source /opt/ros/humble/setup.bash
source /home/yaorz/unitree_go2_ws/install/setup.bash
python3 /home/yaorz/unitree_go2_ws/scripts/run_slam_replay.py /absolute/path/to/run/bag
```

The runner selects a local loopback DDS domain before importing ROS libraries. It
replays only the recorded body cloud and odometry, enables simulation time for the
pipeline, saves raster and posegraph files, and loads the raster into a separate
map server to compare the cells. Run one replay at a time; each run starts fresh
SLAM nodes. The original sensor timestamps are preserved and aligned in memory.

Replay defaults to the original speed (`--rate 1`). Accelerated replay is optional
and its rate is recorded. Nonzero player exits, unavailable/failed posegraph saves,
and cell/resolution/origin mismatches fail the run. The saver YAML is preserved as
`map.yaml.saver_original`; the active YAML retains full-precision origin and
resolution to avoid the installed saver's limited-precision origin rounding.

Results default to `/media/yaorz/Elements/unitree_go2/slam_validation/2026-10-10`.
Use `--output-root` to choose a dated external-drive directory for later sessions.
Drive mount, free space, and temporary writes are checked before running.

The run directory contains `config/`, `metadata.yaml`, logs, `metrics.json`, and:

```text
maps/map.pgm
maps/map.yaml
maps/posegraph.posegraph
maps/posegraph.data
estimated_trajectory.csv
```

Map saving uses `--free 0.196 --occ 0.65`. The installed saver writes unknown PGM
pixels as 205; its default free threshold of 0.25 caused those pixels to reload as
free. The chosen thresholds preserve the observed free/occupied/unknown classes.

The estimated trajectory samples odometry and composes it with the latest received
map transform, recording the timestamp difference. It is an online estimate, not
a final globally optimized trajectory or independent ground truth.

## Offline quality assessment

```bash
python3 scripts/evaluate_slam_run.py /absolute/path/to/replay_output
python3 -m pytest scripts/test/test_slam_quality.py -q
```

This produces source-continuity checks, observed areas, free-cell connectivity,
unmodified occupancy previews and a machine-readable report. Ten-minute input
quality, saved-map consistency and independent geometry are separate gates. The
saved extent is not a surveyed room boundary, and connectivity does not account
for the robot footprint. No independent measurements means geometry stays pending.
Historical reports without original map origin cannot prove its preservation.

Use `--reference` with one map and an independently measured length file to check
selected distances. The empty template is `scripts/config/map_reference_template.json`.
Measurement uncertainty, the operator and method are required; unknown map
endpoints remain pending. Passing a few lengths does not establish navigation
readiness. Follow the single project plan at `../../docs/PLAN.md` for the next
physical test; archived guidance is not normative.

## Initial indoor profile and limits

The recorded robot body origin is about 0.31 m above the floor. The height slice
uses `[-0.2, 1.0]` m in `base_link`, excludes the measured floor band near -0.30 m,
and retains more low obstacles than the previous lower bound of zero. Verify the
height slice for a different robot posture or terrain.

The default converter accumulates one second of real observations. It interpolates
the recorded body poses at individual point times, transforms each observation to
the world, and projects the accumulated points into the latest body frame. It does
not invent returns in unobserved directions. A cloud gap exceeding 0.4 seconds
clears the accumulation window. `--single-frame-scan` on the replay runner, or
`accumulate_scan:=false` on the launch file, selects the original single-frame path.

The scan publisher sends updates for at least 0.05 m translation or 0.05 rad
rotation, with a minimum interval of 0.2 seconds. Three bootstrap updates precede
these motion gates, and a stationary refresh is sent every ten seconds. These
gates avoid repeatedly matching almost identical sparse scans while stationary.

The Humble SLAM implementation checks translation before admitting subsequent
scans. Therefore `minimum_travel_distance` is zero and `minimum_time_interval` is
0.5 seconds; the converter applies the explicit motion gates. Otherwise a turn
in place may leave the graph at one vertex even though `/scan` and `/map` continue
publishing. `min_pass_through` is two, and unvalidated loop closure is disabled for
these short open trajectories. Validate a real closed route before enabling it.

The corrected old turn run now has 17 graph vertices and an estimated yaw change
of 72.28 degrees, close to the recorded odometry's 72.29 degrees. A separate gyro
integration provides supporting evidence of about 71 degrees; it is not a
surveyed map reference. The straight run's net displacement magnitude differs from odometry by
about 0.12 m, so the chain and map saving checks do not establish metric accuracy.

### Stationary drift and angle prior

Two 641 s stationary bags (charged A/B) showed odometry yaw drift of -3.46 and
-3.76 degrees while a multi-start 2D ICP on `cloud_base` measured less than 0.1
degree of physical rotation and 6-23 mm translation. With the earlier angle prior
(`angle_variance_penalty=0.005`, `minimum_angle_penalty=0.1`), `map->odom` yaw
stayed at zero and the map frame shifted by 9 cm over the run. Relaxing the prior
to 0.1/0.3 corrected 2.4 of 3.8 degrees, reduced the map-frame shift to 5.9 cm,
and kept turn yaw within 0.55/1.22 degrees of odometry on the new and old turn
bags. The residual 1.39 degrees is still above the 1 degree target, and the turn
differences have no independent angle reference.

Use [analyze_stationary_drift.py](../../scripts/analyze_stationary_drift.py) and
[analyze_stationary_icp.py](../../scripts/analyze_stationary_icp.py) on finished
bags after sourcing ROS. Raw gyro integration is not a reliable long-baseline
reference: the same A/B bags integrated to +106.7/+21.5 degrees before bias
removal. Full evidence and limitations are in
[docs/offline_analysis_2026-10-24.md](../../docs/offline_analysis_2026-10-24.md).

The recorded cloud gaps are retained and reported; no missing scans are fabricated.
Short straight/turn recordings produce partial maps, not a complete laboratory
map or a loop-closure accuracy result. Do not merge independently recorded runs
without verifying their pose continuity and initialization.

The raw recorder now defaults to the light profile: original raw/body clouds,
odometry, IMU, robot states, and TF. The padded deskewed topic contains about ten
thousand zero points per frame in the observed firmware and is omitted by default.
Use `GO2_RECORD_PROFILE=full` only when that stream is specifically needed.

Recording uses the resilient SQLite preset, WAL journaling, and a five-second
busy timeout. Never inspect a recorder's live SQLite file: use ROS subscriptions
for live checks and wait for a clean stop before reading the bag. The offline
motion checker refuses to inspect a directory still used by an active recorder.
After an abort, preserve the original database and reindex a separate recovery
copy. Successful recovery proves the saved prefix is readable, not that the
intended run finished or that independent backup exists.

For a stationary continuity check without touching a live SQLite database:

```bash
python3 scripts/check_go2_continuity.py --seconds 300 --phase receive_only \
  --output /media/yaorz/Elements/unitree_go2/experiments/unique_continuity_run
```

The destination must be a new directory. It stores per-minute source gaps, frame
rates, raw lidar diagnostics, host UDP/interface counter changes, source snapshots
and a compact frame timing CSV. Match the topics and duration across comparison
phases. UDP counters cover the host, and receiving gaps do not alone identify the
faulty component. Also compare the point times already encoded within each cloud.

The checker now refuses to start without Ethernet carrier and finishes after a
three-second link loss. Reports retain first/last receive times so disconnected
or partial windows can be excluded. Add `--power-state-topic /lf/lowstate` to
record raw battery, voltage, temperature and fan fields; verify units and field
validity before interpreting them. For rosbag, `GO2_RECORD_POWER_STATE=1` adds
the same low-frequency state stream. Match this option across comparison runs.

New recordings use today's date and the robot ID for their experiment directory.
`GO2_SESSION_ID` and `GO2_ROBOT_ID` override these explicitly; historical D1
directories retain their original identity. Replay and quality outputs also use
the current Asia/Shanghai date by default.

For saving a live map after the continuity test:

```bash
python3 scripts/save_live_slam.py --output-root /absolute/external-drive/run_directory
```

It freezes one received occupancy grid on a separate save topic, preserves full
geometry and verifies loading against that exact snapshot, so ongoing map updates
cannot produce a false mismatch. The graph is serialized after grid capture;
these files are not an atomic combined checkpoint. No motion publisher is used.
