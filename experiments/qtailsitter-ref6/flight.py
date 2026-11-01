"""Interactive SITL session; operator retains all flight decisions."""
import fcntl
import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import tempfile

from attach_truth import active_session, begin_snapshot, snapshot_live
from collect import collect
from upload_debug import settings, upload
from ref6 import HERE, BASE, TARGET, VERSION, bundle_check, digest, loaded_truth, profile_params, require
from locate_inputs import path
from source_state import git, save, restore


def no_px4():
    result = subprocess.run(['pgrep', '-x', 'px4'], capture_output=True)
    require(result.returncode == 1, 'PX4 may still be running; stop it before source restore or collection')


def startup(profile, altitude=50):
    params = {'SYS_AUTOSTART': '5021', **profile_params(profile), 'MIS_TAKEOFF_ALT': str(altitude)}
    text = f'. etc/init.d-posix/rcS\n. "$REF6_PROFILE"\nparam set MIS_TAKEOFF_ALT {altitude}\n'
    for name, value in params.items():
        text += (f'param show {name}\n'
                 f'if ! param compare {name} {value}; then\n'
                 f'  echo "REF6 PARAMETER GATE FAILED: {name}"\n'
                 '  shutdown\n  exit 1\nfi\n')
    text += ('param save\n'
             'mavlink start -u 18571 -o 14550 -t "$REF6_QGC" -r 200000 -f\n'
             'mavlink boot_complete\nmavlink status\nver all\n'
             'echo "REF6 PARAMETER GATE PASSED; confirm QGC heartbeat before arming"\n')
    return text


def flight(args):
    bundle_check()
    require(args.host and args.qgc and args.truth, '--host, --qgc and --truth required (explicit unicast IPs)')
    for tool in ('git', 'make', 'script', 'pgrep'):
        require(shutil.which(tool), 'Missing tool: '+tool)
    repo = args.repo.resolve()
    upload_enabled = not getattr(args, 'no_upload', False) and settings() is not None
    require(upload_enabled or getattr(args, 'no_upload', False),
            'Configure debug upload once with configure-upload, or explicitly use --no-upload for an offline run')
    print('DEBUG AUTO-UPLOAD:', 'ON (private server)' if upload_enabled else 'OFF; configure-upload once to enable')
    no_px4()
    key = __import__('hashlib').sha256(str(repo).encode()).hexdigest()[:16]
    with open(Path(tempfile.gettempdir())/('px4-qtail-'+key+'.lock'), 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require(not git(repo, 'status', '--porcelain').stdout, 'PX4 checkout is dirty; nothing discarded')
        git(repo, 'cat-file', '-e', BASE+'^{commit}')
        receipt = json.loads((args.state/'receipt.json').read_text())
        require(not receipt.get('restored'), 'Installation has been restored; reinstall with a new state')
        truth = active_session(args.truth.resolve())
        identity = loaded_truth(truth, receipt)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('0.0.0.0', 18571))
        results = HERE/'runs'
        results.mkdir(exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix='flight-', dir=results))
        (run/'rootfs').mkdir()
        (run/'inputs').mkdir()
        save(repo, run/'source-original.json')
        identity.update({'runner': VERSION, 'base': BASE, 'source': TARGET,
                         'mode': args.mode, 'scope': 'unvalidated full SITL scenario' if args.mode == 'flight' else 'hover-only',
                         'repo': str(repo), 'host': args.host, 'qgc': args.qgc,
                         'profile_sha256': digest(HERE/'profile.sh'),
                         'design_sha256': digest(HERE/'design.json')})
        identity['debug_upload_enabled'] = upload_enabled
        (run/'identity.json').write_text(json.dumps(identity, indent=2)+'\n')
        (run/'truth-source.txt').write_text(str(truth)+'\n')
        shutil.copy2(HERE/'profile.sh', run/'profile.sh')
        shutil.copy2(HERE/'design.json', run/'inputs/design.json')
        shutil.copy2(HERE/'parameters.json', run/'inputs/parameters.json')
        shutil.copy2(HERE/'bench-report.json', run/'inputs/bench-report.json')
        if args.bench_measurements:
            shutil.copy2(args.bench_measurements, run/'inputs/bench-measurements.json')
        xplog = path(identity['manifest']['system_path'])/'Log.txt'
        require(xplog.is_file(), 'Missing X-Plane Log.txt at manifest system_path')
        shutil.copy2(xplog, run/'inputs/Log-before.txt')
        shutil.copy2(receipt['config'], run/'inputs/config.ini')
        shutil.copy2(identity['acf_path'], run/'inputs'/Path(identity['acf_path']).name)
        (run/'startup.sh').write_text(startup(run/'profile.sh', 180 if args.mode == 'flight' else 50))
        changed = False
        initial = None
        try:
            git(repo, 'bundle', 'verify', str(HERE/'diagnostics.bundle'))
            git(repo, '-c', 'fetch.recurseSubmodules=false', 'fetch', '--no-tags',
                str(HERE/'diagnostics.bundle'), 'refs/heads/validation/qtailsitter-diagnostics-20261029')
            require(git(repo, 'rev-parse', 'FETCH_HEAD').stdout.strip() == TARGET, 'Diagnostic identity mismatch')
            changed = True
            git(repo, 'switch', '--detach', TARGET)
            (run/'source.patch').write_text(git(repo, 'diff', BASE, TARGET).stdout)
            with (run/'build.txt').open('w') as output:
                subprocess.run(['make', '-C', str(repo), '-j', '2', 'px4_sitl_default'],
                               stdout=output, stderr=subprocess.STDOUT, check=True)
            build = repo/'build/px4_sitl_default'
            (run/'binary.sha256').write_text(digest(build/'bin/px4')+'\n')
            # A build can outlast the original recording. Recheck identity and growth.
            require(active_session(truth) == truth, 'Recording changed during build')
            loaded_truth(truth, receipt)
            initial = begin_snapshot(truth)
            (run/'truth-preflight.json').write_text(json.dumps(initial, indent=2)+'\n')
            env = os.environ.copy()
            for key in ('PX4_NET_INTERFACE', 'PX4_TEST_TUNING', 'PX4_PARAM_FILE'):
                env.pop(key, None)
            env.update(PX4_SIM_HOSTNAME=args.host, PX4_SIMULATOR='xplane', PX4_SYS_AUTOSTART='5021',
                       REF6_PROFILE=str(run/'profile.sh'), REF6_QGC=args.qgc)
            command = shlex.join([str(build/'bin/px4'), '-w', str(run/'rootfs'),
                                  '-s', str(run/'startup.sh'), str(build/'etc')])
            print('Run:', run, '\nFollow COMMANDS.md. QGC: Windows UDP 14550; no router.', flush=True)
            result = subprocess.run(['script', '-q', '-e', '-c', command, str(run/'console.txt')], env=env)
            (run/'session.json').write_text(json.dumps({'px4_exit_code': result.returncode,
                'parameter_gate_passed': 'REF6 PARAMETER GATE PASSED' in
                    (run/'console.txt').read_text(errors='replace')})+'\n')
            no_px4()
            require(result.returncode == 0, 'PX4 session exited unsuccessfully; evidence retained')
            require('REF6 PARAMETER GATE PASSED' in (run/'console.txt').read_text(errors='replace'),
                    'Startup parameter gate not observed; do not treat as a valid flight')
        finally:
            print('Evidence retained:', run, flush=True)
            try:
                if xplog.is_file():
                    shutil.copy2(xplog, run/'inputs/Log-after.txt')
            except OSError as error:
                print('Could not copy final X-Plane log:', error)
            no_px4()
            if initial is not None and not (run/'truth').exists():
                try:
                    loaded_truth(truth, receipt)
                    snapshot_live(truth, run, initial)
                    print('Truth snapshot complete; recorder may keep running. No manual stop needed.')
                except (OSError, ValueError) as error:
                    (run/'truth-snapshot-error.json').write_text(json.dumps({'error': str(error)})+'\n')
                    print('Truth snapshot failed; incomplete evidence retained:', error)
            try:
                if changed:
                    restore(repo, run/'source-original.json')
            finally:
                archive = collect(run)
                if upload_enabled:
                    try:
                        upload(archive)
                    except (OSError, ValueError) as error:
                        print('Auto-upload failed; local archive retained:', error)
