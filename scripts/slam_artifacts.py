"""Preserve numeric geometry when saving occupancy map metadata."""

import math
import csv
import shutil

import yaml


def trajectory_timing_summary(path):
    """Flag estimates composed with old map transforms instead of treating them as truth."""
    ages = []
    total = 0
    with path.open() as handle:
        for row in csv.DictReader(handle):
            total += 1
            value = row.get('odom_stamp_minus_map_tf_stamp_s')
            if value:
                age = float(value)
                if math.isfinite(age):
                    ages.append(age)
    supported = sum(-.2 <= age <= .5 for age in ages)
    return {'samples': total, 'samples_with_tf_time_delta': len(ages),
            'samples_with_recent_tf': supported, 'samples_outside_tf_time_gate': total - supported,
            'tf_time_delta_min_s': min(ages) if ages else None,
            'tf_time_delta_max_s': max(ages) if ages else None,
            'time_gate_s': [-.2, .5],
            'status': 'timing_caveat' if supported != total else 'passed',
            'interpretation': 'Latest map alignment is held between scan updates. Old transform stamps do not prove a wrong pose, but these samples are not independently time-verified.'}


def preserve_map_geometry(path, resolution, origin_xy_yaw):
    """Keep the saver output and write full-precision geometry before reloading."""
    values = [float(value) for value in origin_xy_yaw]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('Map origin must contain finite x, y and yaw')
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution must be finite and positive')
    config = yaml.safe_load(path.read_text())
    previous = {'resolution': config['resolution'], 'origin': config['origin']}
    backup = path.with_name(path.name + '.saver_original')
    if not backup.exists():
        shutil.copy2(path, backup)
    config['resolution'] = float(resolution)
    config['origin'] = values
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    return {'original_saver_geometry': previous, 'saved_resolution': resolution,
            'saved_origin_xy_yaw': values, 'original_saver_file': str(backup)}
