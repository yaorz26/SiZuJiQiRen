#!/usr/bin/env python3
"""Evaluate a saved SLAM run without confusing pipeline checks with accuracy."""

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from zoneinfo import ZoneInfo

import numpy as np
from PIL import Image
from scipy.ndimage import label
import yaml


def load_map(path):
    """Decode a trinary map, preserving unknown cells and its world geometry."""
    config = yaml.safe_load(path.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary occupancy maps are supported')
    resolution = float(config['resolution'])
    origin = np.asarray(config['origin'], dtype=float)
    free, occupied = float(config['free_thresh']), float(config['occupied_thresh'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution must be finite and positive')
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('Map origin must contain finite x, y and yaw')
    if not 0 <= free < occupied <= 1:
        raise ValueError('Map thresholds must satisfy 0 <= free < occupied <= 1')
    image_path = (path.parent / config['image']).resolve()
    with Image.open(image_path) as image:
        if image.mode != 'L':
            raise ValueError('Expected an 8-bit grayscale occupancy image')
        pixels = np.asarray(image).copy()
    probability = pixels / 255. if config.get('negate', 0) else (255. - pixels) / 255.
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    grid[probability < free] = 0
    grid[probability > occupied] = 100
    return np.flipud(grid), {'resolution_m': resolution, 'origin_xy_yaw': origin.tolist(),
                             'image_path': str(image_path), 'width': grid.shape[1],
                             'height': grid.shape[0]}


def map_shape_metrics(grid, resolution):
    """Describe observed cells; do not infer room coverage or robot traversability."""
    free = grid == 0
    components, count = label(free)  # Four-neighbor connectivity, without smoothing.
    sizes = np.bincount(components.ravel())[1:]
    cells = {'unknown': int((grid == -1).sum()), 'free': int(free.sum()),
             'occupied': int((grid == 100).sum())}
    largest = int(sizes.max()) if count else 0
    return {'cells': cells, 'area_m2': {key: value * resolution ** 2
                                      for key, value in cells.items()},
            'known_fraction_of_saved_extent': (cells['free'] + cells['occupied']) / grid.size,
            'free_components_four_neighbors': int(count),
            'largest_free_component_cells': largest,
            'largest_free_component_area_m2': largest * resolution ** 2,
            'limitations': ['Saved extent is not a surveyed environment boundary',
                            'Free-cell connectivity does not apply a robot footprint or clearance']}


def continuity_assessment(motion, minimum_hz=12., maximum_gap_s=1.,
                          maximum_gap_fraction=.01, minimum_duration_s=600.):
    """Assess source observations separately from the required duration."""
    cloud = motion.get('streams', {}).get('/utlidar/cloud_base', {})
    timing = cloud.get('source_interval_s', {})
    required = (cloud.get('messages'), cloud.get('received_hz'), timing.get('maximum'),
                timing.get('gaps_over_025s'), timing.get('nonincreasing'),
                cloud.get('empty_clouds'))
    thresholds = {'minimum_hz': minimum_hz, 'maximum_gap_s': maximum_gap_s,
                  'maximum_fraction_of_intervals_over_025s': maximum_gap_fraction,
                  'minimum_observation_duration_s': minimum_duration_s,
                  'meaning': 'Provisional project gates, not vendor specifications'}
    if any(value is None for value in required) or cloud.get('messages', 0) < 2:
        return {'status': 'pending', 'reason': 'Missing complete source continuity statistics',
                'thresholds': thresholds}
    gap_fraction = timing['gaps_over_025s'] / (cloud['messages'] - 1)
    checks = {'frequency': cloud['received_hz'] >= minimum_hz,
              'maximum_source_gap': timing['maximum'] <= maximum_gap_s,
              'gap_fraction': gap_fraction < maximum_gap_fraction,
              'strictly_increasing_source_time': timing['nonincreasing'] == 0,
              'nonempty_clouds': cloud['empty_clouds'] == 0}
    enough_time = motion.get('duration_s', 0) >= minimum_duration_s
    status = 'failed' if not all(checks.values()) else ('passed' if enough_time else 'pending')
    return {'status': status, 'observed_sample': 'passed' if all(checks.values()) else 'failed',
            'duration_verified': enough_time, 'duration_s': motion.get('duration_s'),
            'received_hz': cloud['received_hz'], 'maximum_source_gap_s': timing['maximum'],
            'interval_fraction_over_025s': gap_fraction, 'checks': checks,
            'thresholds': thresholds}


def world_cell(point, geometry):
    """Convert world x/y to a bottom-up map cell, including rotated origins."""
    x, y = point
    ox, oy, yaw = geometry['origin_xy_yaw']
    dx, dy = x - ox, y - oy
    r = geometry['resolution_m']
    return (math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / r),
            math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / r))


def measured_distance_assessment(reference, grid, geometry):
    """Compare explicitly surveyed lengths; odometry is not an independent reference."""
    results = []
    for measurement in reference.get('measurements', []):
        result = {'id': measurement['id']}
        if measurement.get('independent') is not True:
            results.append({**result, 'status': 'pending', 'reason': 'Independent measurement not confirmed'})
            continue
        if not measurement.get('measured_by') or not measurement.get('method'):
            raise ValueError('Independent measurements require measured_by and method')
        actual = float(measurement['measured_length_m'])
        uncertainty = float(measurement['uncertainty_m'])
        tolerance = float(measurement.get('allowed_absolute_error_m', .10))
        points = np.asarray(measurement['map_points_m'], dtype=float)
        if (points.shape != (2, 2) or not np.isfinite(points).all()
                or not all(math.isfinite(v) for v in (actual, uncertainty, tolerance))
                or actual <= 0 or uncertainty < 0 or tolerance <= 0):
            raise ValueError('Lengths, uncertainty, tolerance and two map endpoints must be valid')
        cells = [world_cell(point, geometry) for point in points]
        observed = all(0 <= x < geometry['width'] and 0 <= y < geometry['height']
                       and grid[y, x] != -1 for x, y in cells)
        if not observed:
            results.append({**result, 'status': 'pending', 'reason': 'An endpoint is outside observed map cells'})
            continue
        estimated = float(np.linalg.norm(points[1] - points[0]))
        error = abs(estimated - actual)
        if uncertainty > tolerance:
            status = 'pending'
        elif error > tolerance + uncertainty:
            status = 'failed'
        elif error + uncertainty <= tolerance:
            status = 'passed'
        else:
            status = 'pending'  # Measurement uncertainty crosses the tolerance boundary.
        results.append({**result, 'status': status, 'measured_length_m': actual,
                        'map_length_m': estimated, 'absolute_difference_m': error,
                        'reference_uncertainty_m': uncertainty, 'allowed_absolute_error_m': tolerance,
                        'method': measurement['method'], 'measured_by': measurement['measured_by']})
    status = ('failed' if any(r['status'] == 'failed' for r in results) else
              'partially_checked' if results and all(r['status'] == 'passed' for r in results) else 'pending')
    return {'status': status, 'measurements': results,
            'limitations': ['Endpoints are manually selected and must represent the same physical feature',
                            'A few lengths do not establish full-map geometry, loop accuracy or navigation readiness']}


def evaluate_run(run, reference=None):
    """Combine saved artifacts with original source-quality evidence."""
    metrics = json.loads((run / 'metrics.json').read_text())
    metadata = yaml.safe_load((run / 'metadata.yaml').read_text())
    source_bag = Path(metadata['source_bag'])
    motion_path = source_bag.parent / 'motion_validation.json'
    motion = json.loads(motion_path.read_text()) if motion_path.exists() else {}
    grid, geometry = load_map(run / 'maps/map.yaml')
    shape = map_shape_metrics(grid, geometry['resolution_m'])
    expected = {key: int(metrics.get('map', {}).get('cells', {}).get(str(value), -2))
                for key, value in (('unknown', -1), ('free', 0), ('occupied', 100))}
    pipeline = {
        'replay_report_success': metrics.get('status') == 'success',
        'map_saved': metrics.get('map_save_exit_code') == 0,
        'posegraph_serialized': metrics.get('posegraph_save_result') == 0,
        'posegraph_files_present': all((run / 'maps' / f'posegraph.{ext}').is_file()
                                     and (run / 'maps' / f'posegraph.{ext}').stat().st_size > 0
                                     for ext in ('data', 'posegraph')),
        'reloaded_cells_match': metrics.get('map_reload', {}).get('exact_cell_match') is True,
        'saved_classes_match_report': shape['cells'] == expected,
        'saved_dimensions_match_report': (geometry['width'], geometry['height']) == (
            metrics.get('map', {}).get('width'), metrics.get('map', {}).get('height')),
        'saved_resolution_matches_report': math.isclose(geometry['resolution_m'],
            metrics.get('map', {}).get('resolution_m', -1), abs_tol=1e-8, rel_tol=0)}
    origin = metrics.get('map', {}).get('origin')
    geometry_metadata = {'status': 'pending', 'reason': 'Original live origin absent in this historical report'}
    if origin is not None:
        expected_yaw = math.atan2(2*(origin[6]*origin[5]+origin[3]*origin[4]),
                                  1-2*(origin[4]*origin[4]+origin[5]*origin[5]))
        matches = bool(np.allclose(geometry['origin_xy_yaw'],
                                  [origin[0], origin[1], expected_yaw], atol=1e-5, rtol=0))
        pipeline['saved_origin_matches_report'] = matches
        geometry_metadata = {'status': 'passed' if matches else 'failed',
                             'original_origin_xy_yaw': [origin[0], origin[1], expected_yaw],
                             'saved_origin_xy_yaw': geometry['origin_xy_yaw']}
    graph_count = metrics.get('pose_graph_vertices')
    if graph_count is None:
        graph_count = sum(m['namespace'] == 'slam_toolbox' and m['type'] == 2
                          for m in metrics.get('pose_graph_markers', []))
    reference_motion = motion.get('odometry_motion', {})
    slam_motion = metrics.get('slam_pose_change', {})
    comparison = {'reference': 'Input odometry, not independent ground truth'}
    if 'net_yaw_deg' in slam_motion and 'net_yaw_change_deg' in reference_motion:
        comparison['yaw_difference_deg'] = abs(math.remainder(
            slam_motion['net_yaw_deg'] - reference_motion['net_yaw_change_deg'], 360))
        comparison['net_displacement_magnitude_difference_m'] = abs(
            slam_motion['net_xy_m'] - reference_motion['net_xy_displacement_m'])
    distances = measured_distance_assessment(reference or {}, grid, geometry)
    continuity = continuity_assessment(motion)
    return {'run_path': str(run), 'source_bag': str(source_bag),
            'source_quality_report': str(motion_path) if motion_path.exists() else None,
            'pipeline': {'status': 'passed' if all(pipeline.values()) else 'failed', 'checks': pipeline},
            'source_continuity': continuity, 'map_geometry': geometry,
            'saved_geometry_verification': geometry_metadata, 'observed_map': shape,
            'graph_vertices': graph_count, 'motion_consistency': comparison,
            'independent_distances': distances,
            'acceptance': 'pipeline_failed' if not all(pipeline.values()) else
                'source_quality_failed' if continuity['status'] == 'failed' else 'validation_incomplete',
            'navigation_readiness': 'not_verified',
            'remaining': ['Ten-minute continuous input validation', 'Independent map and closed-route measurements',
                          'Robot footprint, clearance and navigation tests']}


def render_map(run, output):
    """Render actual occupancy cells in map coordinates, without inferring structure."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    from matplotlib.transforms import Affine2D
    grid, geometry = load_map(run / 'maps/map.yaml')
    r = geometry['resolution_m']
    width, height = geometry['width'] * r, geometry['height'] * r
    ox, oy, yaw = geometry['origin_xy_yaw']
    transform = Affine2D().rotate(yaw).translate(ox, oy)
    corners = transform.transform([[0, 0], [width, 0], [width, height], [0, height]])
    fig, ax = plt.subplots(figsize=(7, 6.5))
    classes = np.where(grid < 0, 0, np.where(grid == 0, 1, 2))
    ax.imshow(classes, origin='lower', extent=[0, width, 0, height],
              transform=transform + ax.transData, interpolation='nearest', vmin=0, vmax=2,
              cmap=ListedColormap(['#cdcdcd', '#ffffff', '#111111']))
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    ax.set_facecolor('#cdcdcd')
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    ax.set_title(f'Actual occupancy cells, {r:g} m resolution\n{run.name}', fontsize=9)
    ax.legend(handles=[Patch(facecolor='#cdcdcd', label='Unknown'),
                       Patch(facecolor='white', edgecolor='#999999', label='Free'),
                       Patch(facecolor='#111111', label='Occupied')], loc='lower right')
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--reference', type=Path, help='Independent length measurements for one run only')
    parser.add_argument('--output-root', type=Path, default=Path(
        '/media/yaorz/Elements/unitree_go2/slam_quality') / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d'))
    args = parser.parse_args()
    if args.reference and len(args.runs) != 1:
        parser.error('A reference file is tied to one map; evaluate one run with --reference')
    workspace = Path(__file__).resolve().parents[1]
    subprocess.run(['python3', str(workspace / 'scripts/check_external_storage.py'),
                    '--root', str(args.output_root), '--probe'], check=True)
    out = args.output_root / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H%M%S_%f')
    out.mkdir(parents=True)
    shutil.copy2(__file__, out / 'evaluate_slam_run.py')
    reference = json.loads(args.reference.read_text()) if args.reference else None
    if args.reference:
        shutil.copy2(args.reference, out / 'independent_reference.json')
    reports = [evaluate_run(run.resolve(), reference) for run in args.runs]
    summary = {'evaluated_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
               'evaluator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               'reports': reports}
    (out / 'quality.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    lines = ['# 单机 SLAM 离线质量评估', '',
             '保存重载通过、源输入通过和独立几何通过是不同的验收项。未知区域保持未知。', '',
             '| 运行 | 保存链路 | 输入连续性 | 已观测面积 m² | 最大自由连通面积 m² | 独立距离 |',
             '|---|---|---|---:|---:|---|']
    for report in reports:
        area = report['observed_map']['area_m2']
        lines.append(f"| {Path(report['run_path']).name} | {report['pipeline']['status']} | "
                     f"{report['source_continuity']['status']} | {area['free'] + area['occupied']:.2f} | "
                     f"{report['observed_map']['largest_free_component_area_m2']:.2f} | "
                     f"{report['independent_distances']['status']} |")
    lines += ['', '已观测面积是当前保存图中的单元面积，不是完整场地覆盖率。自由连通未计机身尺寸，不能用于直接判断可通行。',
              '', '没有独立实测数据时，几何和导航保持待验证；里程计比较只检查内部一致性。', '']
    (out / 'quality.md').write_text('\n'.join(lines))
    for index, run in enumerate(args.runs):
        render_map(run.resolve(), out / f'map_{index + 1}.png')
    print('Quality report:', out)
    for report in reports:
        print(Path(report['run_path']).name, report['acceptance'], report['independent_distances']['status'])


if __name__ == '__main__':
    main()
