#!/usr/bin/env python3
"""Ref6 installation and evidence gates. No PX4 or plugin source edits."""
import argparse
import fcntl
import configparser
import csv
import hashlib
import io
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

from locate_inputs import path

HERE = Path(__file__).resolve().parent
NAME = 'QuadTailsitterRef6'
BASE = '649929706fb34bf0b06dbd2df24e04d5d452d616'
TARGET = '58794523a151fa33a28a564454454b8c9fbfcdd5'
VERSION = '2026-11-03.ref6.13'
BUNDLE_REF = 'refs/heads/validation/qtailsitter-upstream-20261103'
SCENARIOS = ('baseline', 'recovery-jerk', 'braking-isolation')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def print_source_info():
    print('Source:', VERSION, '| official PX4 BASE:', BASE, '| log-only TARGET:', TARGET)
    print('Includes all official upstream changes since the Sep 21 base, including #28350 '
          '(braking feedforward velocity-limit fix). No new PX4 controller patch.')


def scenario_overrides(mode, scenario):
    require(scenario in SCENARIOS, 'Unknown scenario: '+str(scenario))
    require(mode == 'flight' or scenario == 'baseline', 'Nonbaseline --scenario is only valid for mode flight')
    if mode == 'hover':
        # Pinned FlightTaskAuto.hpp: 5 holds yaw; 4 turns to the waypoint first.
        return {'MPC_YAW_MODE': 5}
    if scenario == 'recovery-jerk':
        return {'MPC_JERK_AUTO': 4.0}
    if scenario == 'braking-isolation':
        return {'MPC_XY_VEL_MAX': 12.0}
    return {}


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def read_config(text):
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    parser.optionxform = str
    try:
        parser.read_string('[global]\n' + text.lstrip('\ufeff'))
    except configparser.Error as error:
        raise ValueError('Invalid/ambiguous plugin config: '+str(error)) from error
    return parser


def config_semantics(text):
    parser = read_config(text)
    return {section: dict(parser[section]) for section in parser.sections()}


def comments(text):
    return sorted(line.strip() for line in text.splitlines() if line.lstrip().startswith((';', '#')))


def bundle_check(bundle=HERE):
    required = ['profile.sh', 'parameters.json', 'design.json', 'diagnostics.bundle', 'diagnostic-sha.txt',
                f'aircraft/{NAME}/{NAME}.acf']
    for relative in required:
        require((bundle/relative).is_file(), 'Missing bundle file: ' + relative)
    require((bundle/'diagnostic-sha.txt').read_text().strip() == TARGET, 'Diagnostic pin mismatch')
    design = json.loads((bundle/'design.json').read_text())
    require(design.get('name') == NAME and design.get('acf_sha256') ==
            digest(bundle/f'aircraft/{NAME}/{NAME}.acf'), 'Design name/ACF hash mismatch')
    expected = json.loads((bundle/'parameters.json').read_text())['parameters']
    actual = profile_params(bundle/'profile.sh')
    require(actual.keys() == expected.keys() and all(math.isclose(float(actual[k]), float(expected[k]),
            rel_tol=1e-8, abs_tol=1e-9) for k in actual), 'Profile/parameters.json mismatch')
    for index in range(4):
        ct = float(expected.get(f'CA_ROTOR{index}_CT', 0))
        km = float(expected.get(f'CA_ROTOR{index}_KM', 0))
        require(math.isfinite(ct * km) and abs(ct * km) > .05,
                'Ref6 yaw/FW-roll allocation would be removed by the weak-axis cutoff; reject this profile')
    source = bundle/'aircraft'/NAME
    require(not any(p.is_symlink() for p in source.rglob('*')), 'Aircraft symlinks refused')
    acf_text = (source/(NAME+'.acf')).read_text(encoding='utf-8')
    require(re.findall(r'^P acf/_num_engn\s+(\S+)\s*$', acf_text, re.MULTILINE) == ['4'],
            'Ref6 structural check requires exactly four engines')
    return source


def profile_params(profile):
    values = {}
    for line in profile.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if re.fullmatch(r'\s*param show [A-Z][A-Z0-9_]*\s*', line):
            continue
        match = re.fullmatch(r'\s*param set(?:-default)? ([A-Z][A-Z0-9_]*) ([-+0-9.eE]+)\s*', line)
        require(match is not None, 'Profile must contain literal param set commands only: ' + line)
        values[match[1]] = match[2]
    require(bool(values), 'Empty profile')
    require('SYS_AUTOSTART' not in values or float(values['SYS_AUTOSTART']) == 5021,
            'Ref6 must retain airframe 5021')
    return values


def stopped():
    # Match executable names, NOT arguments: this installer itself receives
    # an X-Plane path and must not mistake that path for a running simulator.
    # Linux detection alone is insufficient for Windows X-Plane under WSL.
    result = subprocess.run(['pgrep', '-x', r'X-Plane(-x86_64|\.exe)?'],
                            capture_output=True, text=True)
    require(result.returncode in (0, 1), 'Cannot inspect local processes')
    require(result.returncode != 0, 'X-Plane is running; quit before installation/rollback')
    if shutil.which('tasklist.exe'):
        windows = subprocess.run(['tasklist.exe', '/FO', 'CSV', '/NH'], capture_output=True, text=True)
        require(windows.returncode == 0, 'Windows process detection failed; quit X-Plane and retry')
        names = {row[0].casefold() for row in csv.reader(io.StringIO(windows.stdout)) if row}
        require('x-plane.exe' not in names, 'Windows X-Plane is running; quit first')
    else:
        require(input('Cannot detect Windows X-Plane. Quit it, then type XPLANE CLOSED: ').strip()
                == 'XPLANE CLOSED', 'Installation/rollback cancelled')


def atomic(file, data):
    fd, raw = tempfile.mkstemp(prefix='.ref6-', dir=file.parent)
    temporary = Path(raw)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if file.exists():
            shutil.copymode(file, temporary)
        os.replace(temporary, file)
    finally:
        temporary.unlink(missing_ok=True)


def install(root, config, state, bundle=HERE):
    source = bundle_check(bundle)
    require(config.is_file() and not config.is_symlink(), 'Missing or symlink config.ini')
    require((root/'Aircraft').is_dir(), 'Missing X-Plane Aircraft directory')
    destination = root/'Aircraft'/NAME
    if (state/'receipt.json').exists():
        receipt = json.loads((state/'receipt.json').read_text())
        require(not receipt.get('restored'), 'Installation was restored; use a NEW state directory')
        require(receipt['config'] == str(config) and receipt['aircraft'] == str(destination),
                'State belongs to another installation')
        verify_install(receipt, bundle)
        print('Already installed; unchanged')
        return
    require(not destination.exists(), 'Refusing existing Ref6 aircraft folder; nothing overwritten')
    require(not state.exists(), 'State directory already exists; inspect partial transaction first')
    original = config.read_bytes()
    parser = read_config(original.decode('utf-8-sig'))
    require(NAME not in parser, 'Ref6 config section already exists')
    require('QuadTailsitter' in parser, 'Missing source QuadTailsitter config section')
    # Validate with a real INI parser, but serialize ONLY the added section.
    # Preserve the user's other sections, comments and global formatting.
    addition = configparser.ConfigParser(interpolation=None)
    addition.optionxform = str
    addition[NAME] = dict(parser['QuadTailsitter'])
    addition[NAME]['aircraftMatch'] = NAME
    output = io.StringIO()
    addition.write(output)
    text = original.decode('utf-8-sig')
    first_section = re.search(r'^\s*\[', text, re.MULTILINE)
    require(first_section is not None, 'Missing airframe sections')
    header, sections = text[:first_section.start()], text[first_section.start():]
    header, count = re.subn(r'^(\s*config_name\s*=)[^\r\n]*',
                           lambda match: match[1]+' '+NAME, header, flags=re.MULTILINE)
    require(count == 1, 'Require exactly one global config_name')
    newline = '\r\n' if b'\r\n' in original else '\n'
    suffix = output.getvalue().replace('\n', newline)
    updated = (header+sections+newline+suffix).encode('utf-8')
    if original.startswith(b'\xef\xbb\xbf'):
        updated = b'\xef\xbb\xbf'+updated
    state.mkdir(parents=True)
    (state/'config.original').write_bytes(original)
    receipt = {'version': VERSION, 'config': str(config), 'aircraft': str(destination),
               'original_sha256': hashlib.sha256(original).hexdigest(),
               'config_sha256': hashlib.sha256(updated).hexdigest(),
               'acf_sha256': digest(source/(NAME+'.acf')), 'restored': False,
               'config_semantics': config_semantics(updated.decode('utf-8-sig')),
               'config_comments': comments(updated.decode('utf-8-sig'))}
    (state/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    created = False
    try:
        destination.mkdir()
        created = True
        shutil.copytree(source, destination, dirs_exist_ok=True)
        require(config.read_bytes() == original, 'Config changed during copy; refusing overwrite')
        atomic(config, updated)
    except BaseException:
        if digest(config) == receipt['config_sha256']:
            atomic(config, original)
        if created:
            shutil.rmtree(destination)
        raise
    print('Installed separately:', destination, '\nRollback receipt:', state/'receipt.json')


def verify_install(receipt, bundle=HERE):
    acf = Path(receipt['aircraft'])/(NAME+'.acf')
    config = Path(receipt['config'])
    if digest(config) != receipt['config_sha256']:
        require(config_semantics(config.read_text(encoding='utf-8-sig')) == receipt.get('config_semantics'),
                'Config values changed since install; select Ref6 and inspect the config')
    require(digest(acf) == receipt['acf_sha256'] == digest(bundle/'aircraft'/NAME/acf.name),
            'Installed/bundle ACF hash mismatch')
    return acf


def rollback(state):
    receipt_file = state/'receipt.json'
    receipt = json.loads(receipt_file.read_text())
    config = Path(receipt['config'])
    original = (state/'config.original').read_bytes()
    require(hashlib.sha256(original).hexdigest() == receipt['original_sha256'], 'Backup hash mismatch')
    current = digest(config)
    unchanged = current in (receipt['original_sha256'], receipt['config_sha256'])
    if not unchanged:
        text = config.read_text(encoding='utf-8-sig')
        try:
            unchanged = (config_semantics(text) == receipt.get('config_semantics') and
                         comments(text) == receipt.get('config_comments'))
        except ValueError:
            unchanged = False
    require(unchanged, 'Config edited after install; refusing to discard edits. Merge backup manually')
    if current != receipt['original_sha256']:
        atomic(config, original)
    receipt['restored'] = True
    atomic(receipt_file, (json.dumps(receipt, indent=2)+'\n').encode())
    print('Config restored. Ref6 aircraft retained; original aircraft never modified.')


def loaded_truth(session, receipt, bundle=HERE):
    acf = verify_install(receipt, bundle).resolve()
    manifest = json.loads((session/'manifest.json').read_text(encoding='utf-8-sig'))
    require(manifest.get('plugin') == 'XPlaneTruthCapture', 'Wrong truth plugin')
    version = manifest.get('version', '')
    require(re.fullmatch(r'0\.1\.[0-9]+', version) is not None and int(version.split('.')[2]) >= 9,
            'Require compatible TruthCapture 0.1.9 or newer 0.1.x')
    require(manifest.get('aircraft_name') == NAME+'.acf', 'Truth manifest is not the new named ACF')
    require(path(manifest.get('aircraft_path', '')).resolve() == acf,
            'Truth manifest loaded aircraft path mismatch')
    require(manifest.get('aircraft_sha256', digest(acf)) == digest(acf), 'Truth manifest ACF hash mismatch')
    return {'manifest': manifest, 'acf_path': str(acf), 'acf_sha256': digest(acf),
            'hash_scope': 'On-disk ACF at recorder-reported loaded path; not simulator memory'}


def ipv4(value):
    address = ipaddress.IPv4Address(value)
    require(not (address.is_unspecified or address.is_multicast or str(address) == '255.255.255.255'),
            'Require explicit unicast IPv4')
    return str(address)


def recovery_truth(run, source):
    identity = json.loads((run/'identity.json').read_text())
    require(source.resolve() == path((run/'truth-source.txt').read_text()).resolve(),
            'Recovery must use the original flight capture session')
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8-sig'))
    require(manifest == identity['manifest'], 'Recovery capture manifest changed since preflight')
    require(digest(Path(identity['acf_path'])) == identity['acf_sha256'], 'Recovery ACF hash changed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['check', 'install', 'bench', 'hover', 'A', 'flight', 'prepare-truth', 'restore-truth',
                                       'collect', 'restore', 'update', 'configure-upload',
                                       'disable-upload', 'retry-uploads'])
    parser.add_argument('--xplane', type=path)
    parser.add_argument('--config', type=path)
    parser.add_argument('--state', type=path, default=HERE/'installation')
    parser.add_argument('--repo', type=path, default=Path.home()/'PX4-Autopilot-Me')
    parser.add_argument('--run', type=path)
    parser.add_argument('--truth', type=path)
    parser.add_argument('--host', type=ipv4)
    parser.add_argument('--qgc', type=ipv4)
    parser.add_argument('--bench-measurements', type=path)
    parser.add_argument('--scenario', choices=SCENARIOS, default='baseline')
    parser.add_argument('--no-upload', action='store_true', help='Disable upload for this run only')
    parser.add_argument('--yes-debug', action='store_true', help='Explicitly accept the unvalidated SITL experiment without a prompt')
    parser.add_argument('--manifest-url', default='https://de2.joomtalk.ir/OuZsF0TGp1L7DRBjnf_nsGSDc_4wW2gy/ref6-debug-current.json')
    parser.add_argument('--upload-url', default='https://de2.joomtalk.ir/px4xplane-debug/upload')
    args = parser.parse_args()
    scenario_overrides(args.mode, args.scenario)
    # Serialize this bundle's install, rollback, flight and evidence operations.
    with (HERE/'.ref6.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        dispatch(args)


def dispatch(args):
    scenario = getattr(args, 'scenario', 'baseline')
    overrides = scenario_overrides(args.mode, scenario)
    args.state = args.state.resolve()
    if args.mode == 'update':
        from update_debug import update
        print('Updated SAME debug folder to', update(HERE, args.manifest_url))
        print('Next: bash run.sh prepare-truth, then follow QUICKSTART.md for the latest-upstream baseline flight.')
    elif args.mode == 'configure-upload':
        from upload_debug import configure
        configure(url=args.upload_url)
    elif args.mode in ('prepare-truth', 'restore-truth'):
        from attach_truth import configure_diagnostics
        from flight import no_px4
        no_px4()
        configure_diagnostics(args.state, restore=args.mode == 'restore-truth')
    elif args.mode == 'disable-upload':
        from upload_debug import disable
        disable()
    elif args.mode == 'retry-uploads':
        from upload_debug import retry_pending
        retry_pending(HERE/'runs')
    elif args.mode == 'check':
        bundle_check()
        subprocess.run(['git', '-C', str(args.repo), 'bundle', 'verify', str(HERE/'diagnostics.bundle')], check=True)
        print_source_info()
        print('Ref6', VERSION, 'checks passed; no installation or flight performed')
    elif args.mode == 'install':
        require(args.xplane and args.config, '--xplane and --config required')
        stopped()
        install(args.xplane.resolve(), args.config.resolve(), args.state)
    elif args.mode == 'restore':
        if args.run:
            from flight import no_px4
            from source_state import restore
            no_px4()
            restore(args.repo.resolve(), args.run/'source-original.json')
        else:
            stopped()
            rollback(args.state)
    elif args.mode == 'bench':
        from gates import report
        bundle_check()
        report(args.bench_measurements)
    elif args.mode in ('flight', 'hover', 'A'):
        from gates import report
        bundle_check()
        report(args.bench_measurements)
        print('Mode', args.mode, '; SAME pinned diagnostic firmware.')
        print_source_info()
        print('Scenario:', scenario, '| scenario_overrides:', overrides)
        print('UNVALIDATED SITL ONLY. Calibration and hover results are pending. No readiness claim.')
        print('Observe the hover checkpoint before any transition. No EKF overrides.')
        if args.mode != 'flight':
            print('HOVER ONLY: do not transition in this run.')
            if args.mode == 'hover':
                print('MC lateral scenario: 50m console takeoff; fixed heading MPC_YAW_MODE=5 applied and checked automatically.')
        else:
            print('R13: baseline is the recommended latest-upstream SITL test; reusable profile uses '
                  'MPC_JERK_AUTO=4, MPC_ACC_HOR=2, MPC_ACC_HOR_MAX=2. PID/ACF unchanged. '
                  'Follow QUICKSTART.md; recovery remains experimental.')
            if scenario == 'braking-isolation':
                print('WARNING: braking-isolation sets MPC_XY_VEL_MAX=12.0, increases the 1.3*bound threshold to 15.6 m/s, '
                      'and suppresses initial 9.3 m/s braking too. Explicit diagnostic only; NOT recommended, NOT a production fix.')
            elif scenario == 'recovery-jerk':
                print('WARNING: legacy recovery-jerk sets MPC_JERK_AUTO=4.0, now the same as baseline; '
                      'diagnostics only; no full-flight promise.')
        if not args.yes_debug and args.mode != 'A':
            require(input('Type SITL EXPERIMENT to capture this unvalidated simulation test: ').strip()
                    == 'SITL EXPERIMENT', 'Experiment cancelled')
        from flight import flight
        if args.host is None:
            release = Path('/proc/sys/kernel/osrelease').read_text().lower()
            if 'microsoft' in release:
                route = subprocess.check_output(['ip', '-4', 'route', 'show', 'default'], text=True).split()
                require('via' in route, 'Cannot detect Windows gateway; pass --host explicitly')
                args.host = ipv4(route[route.index('via')+1])
            else:
                args.host = '127.0.0.1'
        args.qgc = args.qgc or args.host
        if args.truth is None:
            receipt = json.loads((args.state/'receipt.json').read_text())
            args.truth = Path(receipt['aircraft']).parents[1]/'Output/XPlaneTruthCapture'
        print('SIM host:', args.host, '| QGC host:', args.qgc, '| Truth:', args.truth)
        flight(args)
    else:
        from flight import no_px4
        from collect import collect
        from attach_truth import attach, snapshot_live
        no_px4()
        require(args.run and args.run.is_dir() and (args.run/'identity.json').is_file(), '--run must identify a Ref6 run')
        if args.truth:
            recovery_truth(args.run.resolve(), args.truth.resolve())
            initial = json.loads((args.run/'truth-preflight.json').read_text())
            if initial.get('version') == 1:
                snapshot_live(args.truth.resolve(), args.run.resolve(), initial)
            else:
                attach(args.truth.resolve(), args.run.resolve())
        archive = collect(args.run.resolve())
        if not args.no_upload:
            from upload_debug import upload
            upload(archive)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError, EOFError) as error:
        sys.exit('ERROR: '+str(error))
