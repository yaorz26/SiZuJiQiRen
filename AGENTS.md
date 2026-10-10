# Project instructions

## Large output storage

User preference confirmed on 2026-10-10: future large outputs should preferentially
be written to the attached Elements drive under `/media/yaorz/Elements/unitree_go2`.
The device label is `Elements` (the user referred to it as `Element`).

- Put raw rosbag recordings, dense CSV/PLY point clouds, large maps, videos,
  simulation datasets, and other large generated artifacts under this directory.
- Keep source code, scripts, documentation, small configuration files, and data
  indexes in `/home/yaorz/unitree_go2_ws`.
- Confirm that `/media/yaorz/Elements` is mounted and the destination is writable
  before creating large outputs. Do not silently fall back to the internal disk
  when the external drive is unavailable.
- Use separate experiment/run directories and preserve metadata/configuration
  snapshots linking outputs to their source code and input data.
- A second copy on the same drive is not an independent backup.

Filesystem sandbox permissions still apply. If writing to the external drive is
outside the allowed roots, use the supported command approval flow; do not work
around it with symlinks or alternate tools.

## Physical robot tests

User instruction confirmed on 2026-10-10: while the robot is connected by cable,
inform the user before any movement test and let the user start the movement.
Do not independently publish motion/action requests or launch programs that make
the robot move. Passive network, sensor, state and recording checks may proceed.
