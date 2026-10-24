"""Check occupancy semantics, geometry references and incomplete observation gates."""

import importlib.util
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

SPEC = importlib.util.spec_from_file_location('quality', Path(__file__).parents[1] / 'evaluate_slam_run.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_saved_unknown_pixel_and_image_orientation(tmp_path):
    Image.fromarray(np.array([[205, 254], [0, 205]], dtype=np.uint8)).save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': .03,
                                  'origin': [0, 0, 0], 'negate': 0,
                                  'free_thresh': .196, 'occupied_thresh': .65}))
    grid, _ = MODULE.load_map(path)
    np.testing.assert_array_equal(grid, [[100, -1], [-1, 0]])


def test_small_good_sample_does_not_pass_ten_minute_gate():
    data = {'duration_s': 92, 'streams': {'/utlidar/cloud_base': {
        'messages': 1340, 'received_hz': 14.5, 'empty_clouds': 0,
        'source_interval_s': {'maximum': .33, 'gaps_over_025s': 5, 'nonincreasing': 0}}}}
    result = MODULE.continuity_assessment(data)
    assert result['observed_sample'] == 'passed'
    assert result['status'] == 'pending'
    assert result['duration_verified'] is False
    data['duration_s'] = 600
    data['streams']['/utlidar/cloud_base']['source_interval_s']['maximum'] = 6
    assert MODULE.continuity_assessment(data)['status'] == 'failed'


def test_free_area_does_not_bridge_unknown_cells():
    grid = np.array([[0, -1, 0], [0, -1, 0]])
    result = MODULE.map_shape_metrics(grid, .5)
    assert result['free_components_four_neighbors'] == 2
    assert result['largest_free_component_area_m2'] == .5
    assert result['area_m2']['unknown'] == .5


def test_rotated_origin_is_used_for_reference_cells():
    geometry = {'origin_xy_yaw': [10, 20, math.pi / 2], 'resolution_m': 1}
    assert MODULE.world_cell([9.5, 20.5], geometry) == (0, 0)


def test_independent_reference_and_uncertainty_cannot_be_assumed():
    grid = np.zeros((5, 5), dtype=np.int8)
    geometry = {'origin_xy_yaw': [0, 0, 0], 'resolution_m': 1, 'width': 5, 'height': 5}
    reference = {'measurements': [{'id': 'wall', 'independent': False}]}
    assert MODULE.measured_distance_assessment(reference, grid, geometry)['status'] == 'pending'
    m = reference['measurements'][0]
    m.update({'independent': True, 'measured_by': 'surveyor', 'method': 'tape',
              'measured_length_m': 2, 'uncertainty_m': .01, 'allowed_absolute_error_m': .1,
              'map_points_m': [[.5, .5], [2.5, .5]]})
    assert MODULE.measured_distance_assessment(reference, grid, geometry)['status'] == 'partially_checked'
    m['measured_length_m'] = 1
    assert MODULE.measured_distance_assessment(reference, grid, geometry)['status'] == 'failed'
    m['measured_length_m'] = 1.9
    assert MODULE.measured_distance_assessment(reference, grid, geometry)['status'] == 'pending'
    grid[0, 0] = -1
    assert MODULE.measured_distance_assessment(reference, grid, geometry)['status'] == 'pending'


def test_map_saver_rounding_is_repaired_and_original_is_preserved(tmp_path):
    spec = importlib.util.spec_from_file_location('artifacts', Path(__file__).parents[1] / 'slam_artifacts.py')
    artifacts = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(artifacts)
    p = tmp_path / 'map.yaml'
    p.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': .03,
                               'origin': [-2.57, -4.11, 0], 'free_thresh': .196}))
    original = p.read_text()
    precise = [-2.573251926434363, -4.113837227399753, 0.]
    artifacts.preserve_map_geometry(p, .029999999329447746, precise)
    assert yaml.safe_load(p.read_text())['origin'] == precise
    assert yaml.safe_load(p.read_text())['free_thresh'] == .196
    assert (tmp_path / 'map.yaml.saver_original').read_text() == original


def test_old_transform_samples_are_not_claimed_to_be_time_verified(tmp_path):
    spec = importlib.util.spec_from_file_location('artifacts', Path(__file__).parents[1] / 'slam_artifacts.py')
    artifacts = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(artifacts)
    p = tmp_path / 'trajectory.csv'
    p.write_text('odom_stamp_minus_map_tf_stamp_s\n0.1\n5.0\n-0.1\n')
    result = artifacts.trajectory_timing_summary(p)
    assert result['status'] == 'timing_caveat'
    assert result['samples_with_recent_tf'] == 2
    assert result['samples_outside_tf_time_gate'] == 1
