"""Exercise terminal progress and cancellation without building or launching PX4."""
import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SOURCE = (Path(__file__).resolve().parents[1] / 'setup/setup_px4_sitl.sh').read_text()


def function(name):
    return name + '() {' + SOURCE.split(name + '() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'


@unittest.skipUnless(sys.platform.startswith('linux'), 'Ubuntu/WSL terminal integration')
class LauncherProgress(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        commands = self.root / 'commands'
        commands.mkdir()
        make = commands / 'make'
        make.write_text('''#!/usr/bin/python3
import json, os, signal, subprocess, sys, time
from pathlib import Path
if os.environ.get('IGNORE_TERM'):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
if os.environ.get('PID_FILE'):
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    Path(os.environ['PID_FILE']).write_text(json.dumps([os.getpid(), child.pid]))
print('FULL_BUILD_DETAIL', flush=True)
time.sleep(float(os.environ.get('BUILD_WAIT', '0.25')))
sys.exit(int(os.environ.get('BUILD_RESULT', '0')))
''')
        make.chmod(0o755)
        self.env = {**os.environ, 'HOME': str(self.root), 'XDG_STATE_HOME': str(self.root / 'state'),
                    'PATH': str(commands) + ':/usr/bin:/bin', 'TERM': 'xterm',
                    'BUILD_WAIT': '0.25', 'BUILD_RESULT': '0'}
        self.script = ('BUILD_PID=""\nMAVLINK_ROUTER_PID=""\n' +
                       ''.join(function(name) for name in ('highlight', 'info', 'warning',
                                                          'build_sitl', 'cleanup')) +
                       "trap cleanup EXIT\ntrap 'exit 130' INT\ntrap 'exit 143' TERM\nbuild_sitl\n")

    def drain(self, fd, process, timeout=8):
        output = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([fd], [], [], 0.1)[0]:
                try:
                    data = os.read(fd, 8192)
                except OSError as error:
                    if error.errno == errno.EIO:
                        break
                    raise
                if not data:
                    break
                output.extend(data)
            elif process.poll() is not None:
                break
        return output.decode()

    def run_terminal(self, **extra):
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen(['/bin/bash', '-c', self.script], env={**self.env, **extra},
                                       stdin=subprocess.DEVNULL, stdout=slave, stderr=slave)
            os.close(slave)
            slave = None
            try:
                output = self.drain(master, process)
                code = process.wait(timeout=3)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=3)
            return code, output
        finally:
            os.close(master)
            if slave is not None:
                os.close(slave)

    def test_terminal_progress_keeps_build_details_in_plain_log(self):
        code, output = self.run_terminal(VERBOSE='false')
        self.assertEqual(code, 0, output)
        self.assertIn('[|] Building PX4', output)
        self.assertNotIn('FULL_BUILD_DETAIL', output)
        log = (self.root / 'state/px4xplane/build/latest.log').read_text()
        self.assertIn('FULL_BUILD_DETAIL', log)
        self.assertNotIn('\x1b', log)

    def test_verbose_dumb_and_nonterminal_show_full_output(self):
        for extra in ({'VERBOSE': 'true'}, {'TERM': 'dumb'}):
            code, output = self.run_terminal(**extra)
            self.assertEqual(code, 0, output)
            self.assertIn('FULL_BUILD_DETAIL', output)
            self.assertNotIn('[|] Building PX4', output)
        result = subprocess.run(['/bin/bash', '-c', self.script], env=self.env,
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('FULL_BUILD_DETAIL', result.stdout)
        self.assertNotIn('\r', result.stdout)

    def test_failed_build_shows_error_tail_and_returns_failure(self):
        code, output = self.run_terminal(BUILD_RESULT='17')
        self.assertEqual(code, 1, output)
        self.assertIn('FULL_BUILD_DETAIL', output)
        self.assertIn('No PX4 launch performed', output)

    def cancel_build(self, terminal, interrupt, ignore_term=False):
        pid_file = self.root / 'pids.json'
        guard = subprocess.Popen(['sleep', '60'])
        master, slave = pty.openpty()
        process = subprocess.Popen(['/bin/bash', '-c', self.script],
                                   env={**self.env, 'PID_FILE': str(pid_file), 'BUILD_WAIT': '60',
                                        'IGNORE_TERM': '1' if ignore_term else ''},
                                   stdin=subprocess.DEVNULL,
                                   stdout=slave if terminal else subprocess.PIPE, stderr=slave)
        os.close(slave)
        try:
            deadline = time.monotonic() + 5
            while not pid_file.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(pid_file.exists(), self.drain(master, process, 0.1))
            owned = json.loads(pid_file.read_text())
            process.send_signal(interrupt)
            output = self.drain(master, process)
            self.assertEqual(process.wait(timeout=3), 128 + interrupt, output)
            self.assertIsNone(guard.poll())
            for pid in owned:
                status = Path(f'/proc/{pid}/stat')
                if status.exists():
                    self.assertEqual(status.read_text().split()[2], 'Z', f'Build process {pid} still running')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
            guard.terminate()
            guard.wait(timeout=3)
            if process.stdout is not None:
                process.stdout.close()
            os.close(master)

    def test_interrupt_stops_owned_build_children_not_other_processes(self):
        self.cancel_build(True, signal.SIGINT)

    def test_redirected_output_cancels_and_escalates_ignored_termination(self):
        self.cancel_build(False, signal.SIGTERM, ignore_term=True)


if __name__ == '__main__':
    unittest.main()
