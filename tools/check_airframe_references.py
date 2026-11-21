#!/usr/bin/env python3
"""Compare packaged airframe references with a PX4 checkout without modifying either."""
import argparse
import hashlib
import json
from pathlib import Path
import re

PROFILES = ('5001_xplane_cessna172', '5002_xplane_tb2', '5010_xplane_ehang184',
            '5020_xplane_alia250', '5021_xplane_qtailsitter')


def check(px4, references):
    result = []
    for name in PROFILES:
        reference = references / name
        source = px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes' / name
        data = reference.read_bytes()
        names = re.findall(rb'^param set-default ([A-Z0-9_]+) ', data, re.M)
        result.append({'file': name, 'sha256': hashlib.sha256(data).hexdigest(),
                       'default_count': len(names), 'duplicate_defaults': len(names) != len(set(names)),
                       'matches_px4': data == source.read_bytes()})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-path', type=Path, required=True)
    parser.add_argument('--references', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config/px4_params')
    args = parser.parse_args()
    try:
        result = check(args.px4_path.expanduser(), args.references)
    except OSError as error:
        parser.exit(2, str(error) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if all(r['matches_px4'] and not r['duplicate_defaults'] for r in result) else 1


if __name__ == '__main__':
    raise SystemExit(main())
