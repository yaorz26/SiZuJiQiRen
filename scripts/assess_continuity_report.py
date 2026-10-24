#!/usr/bin/env python3
"""Assess a finished passive observation, including silent edges and raw power fields."""

import argparse
import csv
import json
from pathlib import Path
import statistics
import subprocess

from evaluate_slam_run import continuity_assessment


def assess(directory):
    report = json.loads((directory / 'report.json').read_text())
    metadata = json.loads((directory / 'metadata.json').read_text())
    with (directory / 'cloud_timing.csv').open() as stream:
        clouds = list(csv.DictReader(stream))
    prefix = directory / 'validated_online_prefix.json'
    stop = report.get('stop_reason')
    known_partial = prefix.is_file() or stop == 'robot_ethernet_link_down'
    duration = report['duration_s']
    if known_partial and clouds:
        coverage = report.get('topic_receive_coverage', {})
        duration = max([float(clouds[-1]['receive_elapsed_s'])]
                       + [record['last_receive_elapsed_s'] for record in coverage.values()])
    times = [float(row['receive_elapsed_s']) for row in clouds]
    gaps = [float(row['source_gap_s']) for row in clouds if row['source_gap_s']]
    silence = ([times[0], max(0., duration - times[-1])]
               + [b - a for a, b in zip(times, times[1:])]) if times else [duration]
    complete = (not known_partial and stop in (None, 'requested_duration_completed')
                and duration >= metadata['requested_duration_s'])
    motion = {'duration_s': duration, 'streams': {'/utlidar/cloud_base': {
        'messages': len(clouds), 'received_hz': len(clouds) / duration if duration else 0,
        'empty_clouds': sum(int(row['point_count']) == 0 for row in clouds),
        'source_interval_s': {'maximum': max(gaps) if gaps else None,
                              'gaps_over_025s': sum(g > .25 for g in gaps),
                              'nonincreasing': sum(g <= 0 for g in gaps)}}}}
    quality = continuity_assessment(motion)
    if max(silence) > 1:
        quality['status'] = 'failed'
        quality.setdefault('checks', {})['no_long_receive_silence_including_edges'] = False
    else:
        quality.setdefault('checks', {})['no_long_receive_silence_including_edges'] = True
    if not complete and quality['status'] == 'passed':
        quality['status'] = 'pending'
    power = {}
    power_path = directory / 'power_state.csv'
    if power_path.is_file():
        with power_path.open() as stream:
            rows = list(csv.DictReader(stream))
        for key in ('bms_soc_raw', 'power_v_raw', 'power_a_raw', 'bms_current_raw',
                    'board_temperature_ntc1_raw', 'board_temperature_ntc2_raw'):
            values = [float(row[key]) for row in rows]
            if values:
                power[key] = {'first': values[0], 'last': values[-1],
                              'min': min(values), 'median': statistics.median(values), 'max': max(values)}
        power['messages'] = len(rows)
        power['interpretation'] = 'Raw LowState fields; units/calibration and zero-value validity are not independently verified'
    return {'observation_directory': str(directory), 'phase': metadata['phase'],
            'requested_duration_s': metadata['requested_duration_s'], 'assessed_duration_s': duration,
            'complete_requested_observation': complete, 'stop_reason': stop or 'legacy_not_recorded',
            'known_disconnected_windows_excluded': known_partial, 'continuity': quality,
            'maximum_receive_silence_s': max(silence), 'raw_power_summary': power,
            'host_udp_receive_errors': report['network_delta']['udp_RcvbufErrors'],
            'ethernet_receive_errors': report['network_delta']['enp2s0_rx_errors'],
            'limitations': ['Sequential tests do not isolate all reboot, charge and thermal effects',
                            'Passing source quality does not verify map geometry or navigation']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    if not (args.directory / 'report.json').is_file():
        parser.error('Observation must finish and write report.json before assessment')
    subprocess.run(['python3', str(Path(__file__).with_name('check_external_storage.py')),
                    '--root', str(args.directory), '--probe'], check=True)
    result = assess(args.directory.resolve())
    (args.directory / 'assessment.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
