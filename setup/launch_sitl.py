#!/usr/bin/env python3
"""Launch a normal X-Plane PX4 target with isolated state and an explicit GCS IP."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

API_VERSION = 1
AIRFRAMES = {'xplane_cessna172': 5001, 'xplane_tb2': 5002,
             'xplane_ehang184': 5010, 'xplane_alia250': 5020, 'xplane_qtailsitter': 5021}
GCS_START = 'mavlink start -x -u $udp_gcs_port_local -r 4000000 -f $mavlink_network_interface_arg\n'


def ipv4(value):
    if value == 'localhost':
        return '127.0.0.1'
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or int(address) == 0xffffffff:
        raise ValueError('Expected a unicast IPv4 address')
    return str(address)


def prepare_startup(stock, mavlink, qgc):
    anchor = '. px4-rc.mavlink\n'
    if stock.count(anchor) != 1 or mavlink.count(GCS_START) != 1:
        raise ValueError('PX4 startup layout changed; update the launcher before running this source')
    # Only the GCS endpoint changes. Parameters, controllers and other links use stock startup.
    return (stock.replace(anchor, '. ../mavlink.sh\n', 1),
            mavlink.replace(GCS_START, GCS_START.rstrip('\n') + f' -t {ipv4(qgc)} -o 14550\n', 1))


def reset_parameters(root):
    if root.is_symlink() or (root / 'fs').is_symlink():
        raise ValueError('Refusing to reset a symlinked runtime or parameter directory')
    files = [root / name for name in ('parameters.bson', 'parameters_backup.bson',
                                     'fs/parameters.bson', 'fs/parameters_backup.bson')]
    for file in files:
        if file.is_symlink():
            raise ValueError(f'Refusing to reset symlinked parameter store: {file}')
    existing = [file for file in files if file.is_file()]
    if not existing:
        return None
    backup = root.parent / 'backups' / str(time.time_ns())
    # Copy every store before removing any, so a failed backup leaves the originals intact.
    for file in existing:
        target = backup / file.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, target)
    for file in existing:
        file.unlink()
    return backup


def state_path(checkout, target, label, base):
    identity = hashlib.sha256((str(checkout) + '\n' + label).encode()).hexdigest()[:16]
    return base / identity / target


def run(args):
    px4 = Path(args.checkout).expanduser().resolve()
    if args.target not in AIRFRAMES:
        raise ValueError('Unsupported X-Plane target')
    host, qgc = ipv4(args.host), ipv4(args.qgc_host)
    base = Path(args.state_root).expanduser().resolve()
    state = state_path(px4, args.target, args.source_label, base)
    if state.is_symlink() or (state / 'rootfs').is_symlink():
        raise ValueError('Runtime state must not be symlinked into another installation')
    build = px4 / 'build/px4_sitl_default'
    source = px4 / 'ROMFS/px4fmu_common/init.d-posix'
    stock = (source / 'rcS').read_text()
    if (build / 'etc/init.d-posix/rcS').read_text() != stock:
        raise ValueError('Built PX4 startup differs from checkout; rebuild before launching')
    startup, mavlink = prepare_startup(stock, (source / 'px4-rc.mavlink').read_text(), qgc)
    binary = build / 'bin/px4'
    if not binary.is_file():
        raise ValueError('PX4 executable missing; run the build first')
    # Do not compete for PX4's instance-0 socket or modify state beneath another run.
    process = subprocess.run(['pgrep', '-x', 'px4'], capture_output=True, text=True)
    if process.returncode == 0:
        raise ValueError('PX4 is already running; stop it before launching another instance')
    if process.returncode != 1:
        raise ValueError('Could not check for an existing PX4 process')
    revision = subprocess.check_output(['git', '-C', str(px4), 'rev-parse', 'HEAD'], text=True).strip()
    profile = source / 'airframes' / f'{AIRFRAMES[args.target]}_{args.target}'
    fingerprint = hashlib.sha256(profile.read_bytes()).hexdigest()
    state.mkdir(parents=True, exist_ok=True)
    with (state / '.runner.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        root = state / 'rootfs'
        root.mkdir(exist_ok=True)
        record = state / 'last-launch.json'
        previous = json.loads(record.read_text()) if record.exists() else {}
        if not isinstance(previous, dict):
            raise ValueError('Runtime metadata is not an object; inspect it before retrying')
        loaded_defaults = previous.get('loaded_defaults_sha256', previous.get('airframe_sha256', fingerprint))
        if loaded_defaults != fingerprint and not args.reset_params:
            print('Airframe defaults changed; saved tuning is PRESERVED. Use --reset-params '
                  'after reviewing the change to back up and reload defaults.')
        if args.reset_params:
            backup = reset_parameters(root)
            print(f'Selected airframe parameter backup: {backup or "no saved parameters"}')
        (state / 'startup.sh').write_text(startup)
        (state / 'mavlink.sh').write_text(mavlink)
        metadata = {'source_commit': revision, 'source_label': args.source_label,
                    'target': args.target, 'sys_autostart': AIRFRAMES[args.target],
                    'airframe_sha256': fingerprint, 'xplane_host': host, 'qgc_host': qgc,
                    'loaded_defaults_sha256': fingerprint if args.reset_params else loaded_defaults,
                    'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
        record.write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'PX4 source: {revision}\nAirframe: {args.target} (ID {AIRFRAMES[args.target]})\n'
              f'Runtime: {root}\nX-Plane: TCP {host}:4560\nQGC target: UDP {qgc}:14550')
        print('QGC reception is NOT verified. Check the vehicle/heartbeat in QGC.\n'
              'In X-Plane select the matching airframe and Connect to SITL.\n'
              'At first build timeout, click Connect to SITL again. Use shutdown to exit PX4.', flush=True)
        env = {key: value for key, value in os.environ.items()
               if not key.startswith('PX4_PARAM_') and key not in {'replay_mode', 'R', 'px4_instance'}}
        env.update(PX4_SIMULATOR='xplane', PX4_SYS_AUTOSTART=str(AIRFRAMES[args.target]),
                   PX4_SIM_MODEL=args.target, PX4_SIM_HOSTNAME=host)
        # Preserve the interactive PX4 console and unmodified official parameter startup.
        return subprocess.run([str(binary), '-w', str(root), '-s', '../startup.sh',
                               str(build / 'etc')], cwd=root, env=env).returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api-version', action='store_true')
    parser.add_argument('--checkout')
    parser.add_argument('--target', choices=AIRFRAMES)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--qgc-host', default='127.0.0.1')
    parser.add_argument('--source-label', default='official')
    parser.add_argument('--reset-params', action='store_true')
    parser.add_argument('--state-root', default=str(Path(os.environ.get('XDG_STATE_HOME',
                        str(Path.home() / '.local/state'))) / 'px4xplane/sitl'))
    args = parser.parse_args(argv)
    if args.api_version:
        print(API_VERSION)
        return 0
    if not args.checkout or not args.target:
        parser.error('--checkout and --target are required')
    try:
        return run(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
