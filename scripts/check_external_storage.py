#!/usr/bin/env python3
"""Check the requested output location; optionally test a small temporary write."""

import argparse
import os
from pathlib import Path
import shutil
import tempfile


DEFAULT_MOUNT = Path('/media/yaorz/Elements')
DEFAULT_ROOT = DEFAULT_MOUNT / 'unitree_go2'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--probe', action='store_true', help='Create the destination if needed and test a temporary write.')
    args = parser.parse_args()
    root = args.root.expanduser().absolute()

    if root == DEFAULT_ROOT or DEFAULT_MOUNT in root.parents:
        if not DEFAULT_MOUNT.is_mount():
            parser.exit(1, f'External drive is not mounted at {DEFAULT_MOUNT}. Recording stopped.\n')
    try:
        existing = root
        while not existing.exists():
            existing = existing.parent
        free_gib = shutil.disk_usage(existing).free / 1024**3
        if free_gib < 5:
            parser.exit(1, f'Only {free_gib:.1f} GiB free at {existing}. Recording stopped.\n')
        if args.probe:
            root.mkdir(parents=True, exist_ok=True)
            descriptor, probe_name = tempfile.mkstemp(prefix='.go2_write_probe_', dir=root)
            try:
                with os.fdopen(descriptor, 'wb') as probe:
                    probe.write(b'go2 external storage write check\n')
                    probe.flush()
                    os.fsync(probe.fileno())
            finally:
                Path(probe_name).unlink(missing_ok=True)
        print(f'Output root: {root}')
        print(f'Available space: {free_gib:.1f} GiB')
        if args.probe:
            print('Temporary write and flush: OK; probe removed.')
    except OSError as error:
        parser.exit(1, f'Output storage unavailable at {root}: {error}\n')


if __name__ == '__main__':
    main()
