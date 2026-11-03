#!/usr/bin/env python3
"""Screen recorded braking-release states with the pinned PX4 smoother.

This executes the real VelocitySmoothing implementation with a fixed stop
target and no aircraft, guidance, sensor, or position-feedback model. It is not
a flight replay or proof that a jerk setting stabilizes the complete vehicle.
Requires an existing PX4 checkout and g++; never changes that checkout.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile

PIN = '9cfe130a8d8e0c25f88e044141c523247ca53d48'
SOURCE = 'src/lib/motion_planning/VelocitySmoothing.cpp'
HEADER = 'src/lib/motion_planning/VelocitySmoothing.hpp'
JERKS = (.7, 2., 4., 8.)


def releases(events):
    result = []
    for event in events:
        if event['active'] != 0 or not 125 < event['t'] < 221:
            continue
        a = event['smooth_accel']
        v = event['smooth_velocity']
        if len(a) != 3 or len(v) != 3 or not all(math.isfinite(x) for x in (*a, *v)):
            raise ValueError('Release requires three finite acceleration/velocity components')
        result.append({'boot_s': event['t'], 'acceleration': a, 'velocity': v})
    if not result:
        raise ValueError('No pre-edit braking-release states found')
    return result


def replay(repo, events):
    if not shutil.which('g++'):
        raise RuntimeError('g++ is required')
    def source(path):
        return subprocess.check_output(['git', '-C', str(repo), 'show', PIN+':'+path], text=True)
    cpp, header = source(SOURCE), source(HEADER)
    driver = r'''
#include "VelocitySmoothing.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
int main() {
  double stamp, jerk, a[3], v[3];
  while (scanf("%lf %lf %lf %lf %lf %lf %lf %lf", &stamp, &jerk,
         &a[0], &a[1], &a[2], &v[0], &v[1], &v[2]) == 8) {
    VelocitySmoothing s[3];
    for (int i=0; i<3; ++i) {
      s[i].reset(a[i], v[i], 0.f);
      s[i].setMaxJerk(jerk);
      s[i].setMaxAccel(i<2 ? .7f : 1.5f);
      s[i].setMaxVel(i<2 ? 4.f : 1.4f);
      s[i].updateDurations(0.f);
    }
    VelocitySmoothing::timeSynchronization(s, 3);
    float peak=std::hypot(v[0],v[1]), crossing=-1, accel_low=-1;
    for (int k=1;k<=3000;++k) {
      for (auto &axis:s) axis.updateTraj(.01f);
      float speed=std::hypot(s[0].getCurrentVelocity(),s[1].getCurrentVelocity());
      float accel=std::hypot(s[0].getCurrentAcceleration(),s[1].getCurrentAcceleration());
      peak=std::max(peak,speed);
      if (speed>5.2f && crossing<0) crossing=k*.01f;
      if (accel<.7f && accel_low<0) accel_low=k*.01f;
      for (auto &axis:s) axis.updateDurations(0.f);
      VelocitySmoothing::timeSynchronization(s,3);
    }
    printf("%.9g %.9g %.9g %.9g %.9g\n",stamp,jerk,peak,crossing,accel_low);
  }
}
'''
    with tempfile.TemporaryDirectory(prefix='ref6-smoothing-') as directory:
        root = Path(directory)
        (root/'px4_platform_common').mkdir()
        (root/'px4_platform_common/defines.h').write_text(
            '#pragma once\n#include <cmath>\n#define PX4_ISFINITE(x) std::isfinite(x)\n')
        (root/'VelocitySmoothing.cpp').write_text(cpp)
        (root/'VelocitySmoothing.hpp').write_text(header)
        (root/'driver.cpp').write_text(driver)
        subprocess.run(['g++', '-std=c++17', '-O2', '-I'+str(root),
            '-I'+str(repo/'src/lib'), '-I'+str(repo/'src/lib/matrix'),
            str(root/'VelocitySmoothing.cpp'), str(root/'driver.cpp'), '-o', str(root/'replay')], check=True)
        lines = [' '.join(map(str, (e['boot_s'], j, *e['acceleration'], *e['velocity'])))
                 for e in events for j in JERKS]
        output = subprocess.check_output([str(root/'replay')], input='\n'.join(lines)+'\n', text=True)
    rows = []
    for line in output.splitlines():
        stamp, jerk, peak, cross, low = map(float, line.split())
        rows.append(dict(release_boot_s=stamp, jerk_mps3=jerk,
            peak_virtual_horizontal_speed_mps=peak,
            first_5p2_crossing_s=None if cross < 0 else cross,
            first_acceleration_below_0p7_s=None if low < 0 else low))
    return dict(px4_pin=PIN, cpp_sha256=hashlib.sha256(cpp.encode()).hexdigest(),
        header_sha256=hashlib.sha256(header.encode()).hexdigest(), release_states=events, rows=rows,
        limits=dict(horizontal_acceleration_mps2=.7, horizontal_velocity_mps=4,
                    vertical_acceleration_mps2=1.5, vertical_velocity_mps=1.4),
        assumptions='Fixed zero velocity target on all axes, time stretch=1, 10ms steps, '
                    '30s horizon, synchronized XYZ. No braking reactivation or aircraft feedback. '
                    'Recorded release samples are diagnostic-rate observations, not exact switching states.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path, help='braking_preedit.json from the R11 controls audit')
    parser.add_argument('--px4-repo', type=Path, default=Path.home()/'PX4-Autopilot-Me')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay(args.px4_repo, releases(json.loads(args.events.read_text())))
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for jerk in JERKS:
        rows = [r for r in result['rows'] if r['jerk_mps3'] == jerk]
        print(f'jerk={jerk:g}: peak virtual speed {min(r["peak_virtual_horizontal_speed_mps"] for r in rows):.2f}'
              f'..{max(r["peak_virtual_horizontal_speed_mps"] for r in rows):.2f} m/s; '
              f'{sum(r["first_5p2_crossing_s"] is not None for r in rows)}/{len(rows)} cross 5.2 m/s')
