"""Run with: python3 -m unittest discover -s experiments/qtailsitter-ref6 -p test_update_debug.py"""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import stat
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch
import zipfile

import update_debug as updater


ACF = 'aircraft/QuadTailsitterRef6/QuadTailsitterRef6.acf'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def sums(files):
    return ''.join(f'{sha(data)}  {name}\n' for name, data in sorted(files.items())).encode()


def archive(files, extra=None, checksum=None):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as zipped:
        zipped.writestr('release/', b'')
        for name, data in files.items():
            zipped.writestr('release/' + name, data)
        zipped.writestr('release/SHA256SUMS', sums(files) if checksum is None else checksum)
        if extra:
            zipped.writestr(*extra)
    return stream.getvalue()


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.parent = Path(self.temp.name)
        self.bundle = self.parent / 'existing-bundle'
        self.bundle.mkdir()
        self.old = {'ref6.py': b'old code\n', 'SOURCE.md': b'original source\n',
                    'VERSION.txt': b'v1\n', ACF: b'unchanged aircraft\n',
                    'flight.py': b'obsolete code\n'}
        self.new = {**self.old, 'ref6.py': b'new code\n', 'VERSION.txt': b'v2\n',
                    'SOURCE.md': b'new source\n', 'update_debug.py': b'new updater\n'}
        del self.new['flight.py']
        for name, data in {**self.old, 'SHA256SUMS': sums(self.old)}.items():
            path = self.bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        (self.bundle / 'ref6.py').chmod(0o755)
        self.user = {'installation/receipt.json': b'installed', 'runs/session/log': b'flight log',
                     '.ref6.lock': b'lock', 'user-notes.txt': b'notes'}
        for name, data in self.user.items():
            path = self.bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        self.credential = self.parent / '.config/px4xplane-debug/upload.json'
        self.credential.parent.mkdir(parents=True)
        self.credential.write_bytes(b'secret')
        self.pgrep = patch.object(updater.subprocess, 'run',
                                 return_value=subprocess.CompletedProcess([], 1))
        self.run = self.pgrep.start()
        self.addCleanup(self.pgrep.stop)

    def invoke(self, files=None, zipped=None, expected_hash=None, version='v2'):
        blob = zipped if zipped is not None else archive(self.new if files is None else files)
        manifest = json.dumps({'version': version, 'url': 'https://example.test/release.zip',
                               'sha256': expected_hash or sha(blob)}).encode()

        def download(url, target, limit, allow_loopback_http=False):
            data = blob if url.endswith('.zip') else manifest
            if len(data) > limit:
                raise ValueError('Download exceeds size limit')
            self.assertEqual(target.parent.parent, self.parent)
            target.write_bytes(data)

        with patch.object(updater, 'download', side_effect=download):
            return updater.update(self.bundle, 'https://example.test/manifest.json')

    def assert_original(self):
        for name, data in {**self.old, 'SHA256SUMS': sums(self.old)}.items():
            self.assertEqual((self.bundle / name).read_bytes(), data, name)
        self.assertFalse((self.bundle / 'update_debug.py').exists())
        self.assertEqual(stat.S_IMODE((self.bundle / 'ref6.py').stat().st_mode), 0o755)
        self.assert_preserved()

    def assert_preserved(self):
        for name, data in self.user.items():
            self.assertEqual((self.bundle / name).read_bytes(), data)
        self.assertEqual(self.credential.read_bytes(), b'secret')
        self.assertFalse(list(self.parent.glob('.ref6-update-*')))

    def test_update_preserves_state_and_original_backup_and_is_idempotent(self):
        inode = self.bundle.stat().st_ino
        lock_inode = (self.bundle / '.ref6.lock').stat().st_ino
        self.assertEqual(self.invoke(), 'v2')
        for name, data in self.new.items():
            self.assertEqual((self.bundle / name).read_bytes(), data)
        self.assertFalse((self.bundle / 'flight.py').exists())
        backup = self.bundle / '.previous-managed'
        for name, data in {**self.old, 'SHA256SUMS': sums(self.old)}.items():
            self.assertEqual((backup / 'files' / name).read_bytes(), data)
        self.assertFalse((backup / 'files/installation').exists())
        self.assertEqual(self.invoke(), 'v2')
        self.assertEqual((backup / 'files/SOURCE.md').read_bytes(), self.old['SOURCE.md'])
        self.assertEqual(self.bundle.stat().st_ino, inode)
        self.assertEqual((self.bundle / '.ref6.lock').stat().st_ino, lock_inode)
        self.run.assert_called_with(['pgrep', '-x', 'px4'], capture_output=True, text=True)
        self.assert_preserved()

    def test_archive_hash_mismatch(self):
        with self.assertRaisesRegex(ValueError, 'Archive hash mismatch'):
            self.invoke(expected_hash='0' * 64)
        self.assert_original()

    def test_second_update_replaces_only_previous_managed_backup(self):
        self.invoke()
        first = dict(self.new)
        self.new = {**first, 'VERSION.txt': b'v3\n', 'ref6.py': b'third release\n'}
        self.invoke(version='v3')
        for name, data in first.items():
            self.assertEqual((self.bundle / '.previous-managed/files' / name).read_bytes(), data)
        self.assert_preserved()

    def test_real_loopback_http_update(self):
        blob = archive(self.new)
        payloads = {'/release.zip': blob}

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(payloads[self.path])

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f'http://127.0.0.1:{server.server_port}'
            payloads['/manifest.json'] = json.dumps({'version': 'v2', 'url': base + '/release.zip',
                                                    'sha256': sha(blob)}).encode()
            with self.assertRaisesRegex(ValueError, 'HTTPS'):
                updater.update(self.bundle, base + '/manifest.json')
            self.assertEqual(updater.update(self.bundle, base + '/manifest.json',
                                           allow_loopback_http=True), 'v2')
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
        self.assert_preserved()

    def test_internal_hash_mismatch_and_incomplete_sums(self):
        for checksum in (sums({**self.new, 'ref6.py': b'wrong'}),
                         sums({k: v for k, v in self.new.items() if k != 'ref6.py'}),
                         sums({**self.new, 'ghost.py': b'missing'})):
            with self.subTest(checksum=checksum):
                with self.assertRaises(ValueError):
                    self.invoke(zipped=archive(self.new, checksum=checksum))
                self.assert_original()

    def test_duplicate_and_unsafe_checksum_entries(self):
        for checksum in (sums(self.new) + sums({'ref6.py': self.new['ref6.py']}),
                         sums({**self.new, '../outside': b'escape'}),
                         sums(self.new) + b'invalid checksum\n'):
            with self.assertRaises(ValueError):
                self.invoke(zipped=archive(self.new, checksum=checksum))
            self.assert_original()

    def test_flat_archive(self):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as zipped:
            for name, data in {**self.new, 'SHA256SUMS': sums(self.new)}.items():
                zipped.writestr(name, data)
        self.invoke(zipped=stream.getvalue())
        self.assert_preserved()

    def test_malicious_zip_names_and_symlink(self):
        link = zipfile.ZipInfo('release/link')
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        extras = [(name, b'evil') for name in ('../escape', '/absolute', 'release/../escape',
                   'release/./ref6.py', 'release/a\\b', 'C:/escape', 'other/ref6.py',
                   'release/ref6.py')]
        extras.append((link, b'../../escape'))
        for extra in extras:
            with self.subTest(extra=extra[0]):
                with self.assertRaises(ValueError):
                    self.invoke(zipped=archive(self.new, extra))
                self.assert_original()
        self.assertFalse((self.parent / 'escape').exists())

    def test_local_edit_and_missing_managed_file(self):
        path = self.bundle / 'ref6.py'
        path.write_bytes(b'user edit')
        with self.assertRaisesRegex(ValueError, 'Local edits'):
            self.invoke()
        self.assertEqual(path.read_bytes(), b'user edit')
        path.unlink()
        with self.assertRaisesRegex(ValueError, 'Local edits'):
            self.invoke()

    def test_only_managed_checksums_checked_and_obsolete_owned_files_removed(self):
        current = {**self.old, 'runs/old.txt': b'original run', 'custom.txt': b'original note'}
        (self.bundle / 'SHA256SUMS').write_bytes(sums(current))
        (self.bundle / 'runs/old.txt').write_bytes(b'changed run')
        (self.bundle / 'custom.txt').write_bytes(b'changed note')
        self.invoke()
        self.assertEqual((self.bundle / 'runs/old.txt').read_bytes(), b'changed run')
        self.assertEqual((self.bundle / 'custom.txt').read_bytes(), b'changed note')
        self.assertFalse((self.bundle / 'flight.py').exists())

    def test_unowned_collision_and_unmanaged_payload(self):
        (self.bundle / 'update_debug.py').write_bytes(b'user file')
        with self.assertRaisesRegex(ValueError, 'Unowned file collision'):
            self.invoke()
        (self.bundle / 'update_debug.py').unlink()
        for name in ('runs/data', 'installation/receipt.json', '.ref6.lock',
                     '.config/px4xplane-debug/upload.json', 'aircraft/New/New.acf'):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, 'Unmanaged incoming'):
                    self.invoke(files={**self.new, name: b'bad'})
        self.assert_original()

    def test_acf_changes_additions_and_removal_require_asset_install(self):
        for files in ({k: v for k, v in self.new.items() if k != ACF},
                      {**self.new, updater.AIRCRAFT + 'second.acf': b'new'}):
            with self.assertRaisesRegex(ValueError, 'asset install'):
                self.invoke(files=files)
            self.assert_original()

    def asset_fixture(self):
        installed = self.parent / 'X-Plane 12/Aircraft/QuadTailsitterRef6'
        installed.mkdir(parents=True)
        acf = installed / 'QuadTailsitterRef6.acf'
        acf.write_bytes(self.old[ACF])
        receipt = self.bundle/'installation/receipt.json'
        receipt.write_text(json.dumps({'aircraft': str(installed), 'acf_sha256': sha(self.old[ACF]),
                                      'version': 'v1', 'restored': False, 'config': 'untouched'}))
        return acf, receipt

    def test_asset_update_same_location_and_receipt(self):
        acf, receipt = self.asset_fixture()
        with patch.object(updater, 'assets_stopped') as stopped:
            self.invoke(files={**self.new, ACF: b'compact aircraft'})
        stopped.assert_called_once()
        self.assertEqual(acf.read_bytes(), b'compact aircraft')
        revised = json.loads(receipt.read_text())
        self.assertEqual(revised['acf_sha256'], sha(b'compact aircraft'))
        self.assertEqual(revised['version'], 'v2')
        self.assertEqual(revised['config'], 'untouched')
        self.assertEqual((self.bundle/'.previous-managed/installed/0').read_bytes(), self.old[ACF])

    def test_asset_failure_rolls_back_model_receipt_and_managed_files(self):
        acf, receipt = self.asset_fixture()
        original_receipt = receipt.read_bytes()
        original = updater.atomic_file
        def fail(source, target, stage):
            if source.name == 'ref6.py' and 'payload' in source.parts:
                raise OSError('injected write failure')
            return original(source, target, stage)
        with patch.object(updater, 'assets_stopped'), patch.object(updater, 'atomic_file', side_effect=fail):
            with self.assertRaisesRegex(OSError, 'injected'):
                self.invoke(files={**self.new, ACF: b'compact aircraft'})
        self.assertEqual(acf.read_bytes(), self.old[ACF])
        self.assertEqual(receipt.read_bytes(), original_receipt)
        self.assertEqual((self.bundle/'ref6.py').read_bytes(), self.old['ref6.py'])

    def test_edited_installed_aircraft_refused(self):
        acf, receipt = self.asset_fixture()
        acf.write_bytes(b'user edit')
        with self.assertRaisesRegex(ValueError, 'local edits'):
            self.invoke(files={**self.new, ACF: b'compact aircraft'})
        self.assertEqual(acf.read_bytes(), b'user edit')
        self.assertEqual((self.bundle/'ref6.py').read_bytes(), self.old['ref6.py'])

    def test_restart_recovers_installed_and_managed_pending_update(self):
        acf, receipt = self.asset_fixture()
        original_receipt = receipt.read_bytes()
        with patch.object(updater, 'assets_stopped'):
            self.invoke(files={**self.new, ACF: b'compact aircraft'})
            journal_path = self.bundle/'.previous-managed/journal.json'
            journal = json.loads(journal_path.read_text())
            journal['state'] = 'pending'
            journal_path.write_text(json.dumps(journal))
            with tempfile.TemporaryDirectory(dir=self.parent) as stage:
                updater.recover(self.bundle, Path(stage))
        self.assertEqual(acf.read_bytes(), self.old[ACF])
        self.assertEqual(receipt.read_bytes(), original_receipt)
        self.assertEqual((self.bundle/'ref6.py').read_bytes(), self.old['ref6.py'])
        self.assertEqual(json.loads(journal_path.read_text())['state'], 'rolled-back')

    def test_uninstalled_bundle_can_update_aircraft_without_external_writes(self):
        (self.bundle/'installation/receipt.json').unlink()
        self.invoke(files={**self.new, ACF: b'compact aircraft'})
        self.assertEqual((self.bundle/ACF).read_bytes(), b'compact aircraft')

    def test_symlink_in_current_bundle_and_checksum_refused(self):
        path = self.bundle / 'ref6.py'
        outside = self.parent / 'outside.py'
        outside.write_bytes(self.old['ref6.py'])
        path.unlink()
        path.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            self.invoke()
        path.unlink()
        path.write_bytes(self.old['ref6.py'])
        checksum = self.bundle / 'SHA256SUMS'
        outside.write_bytes(checksum.read_bytes())
        checksum.unlink()
        checksum.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            self.invoke()

    def test_px4_running_or_detection_failure_refused(self):
        for code in (0, 2):
            self.run.return_value = subprocess.CompletedProcess([], code)
            with self.assertRaises(ValueError):
                self.invoke()
            self.assert_original()

    def test_new_file_symlink_parent_cannot_escape_bundle(self):
        outside = self.parent / 'outside'
        outside.mkdir()
        (self.bundle / 'build_tools').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            self.invoke(files={**self.new, 'build_tools/build_qtailsitter_reference.py': b'bad'})
        self.assertEqual(list(outside.iterdir()), [])
        self.assert_original()

    def test_write_failure_and_keyboard_interrupt_roll_back(self):
        real = updater.atomic_file
        for error in (OSError('disk failure'), KeyboardInterrupt()):
            fired = False

            def fail(source, destination, stage):
                nonlocal fired
                if destination == self.bundle / 'ref6.py' and not fired:
                    fired = True
                    raise error
                return real(source, destination, stage)

            with patch.object(updater, 'atomic_file', side_effect=fail):
                with self.assertRaises(type(error)):
                    self.invoke()
            self.assertTrue(fired)
            self.assert_original()

    def test_failure_after_obsolete_removal_rolls_back(self):
        real = updater.journal_write

        def fail(backup, journal, stage):
            if journal['state'] == 'complete':
                raise OSError('commit failure')
            return real(backup, journal, stage)

        with patch.object(updater, 'journal_write', side_effect=fail):
            with self.assertRaises(OSError):
                self.invoke()
        self.assert_original()

    def test_abrupt_interruption_recovered_before_next_download(self):
        real = updater.atomic_file

        def fail(source, destination, stage):
            if destination == self.bundle / 'ref6.py':
                raise KeyboardInterrupt()
            return real(source, destination, stage)

        # Simulate process death: suppress the in-process exception recovery.
        recover = updater.recover
        with patch.object(updater, 'atomic_file', side_effect=fail), \
                patch.object(updater, 'recover', side_effect=[None, SystemExit()]):
            with self.assertRaises(SystemExit):
                self.invoke()
        with tempfile.TemporaryDirectory(dir=self.parent) as stage:
            recover(self.bundle, Path(stage))
        self.assert_original()
        self.invoke()
        self.assert_preserved()

    def test_recovery_refuses_post_crash_user_edit(self):
        with patch.object(updater, 'journal_write', side_effect=OSError('stop')):
            # Failure before publication cannot affect the bundle.
            with self.assertRaises(OSError):
                self.invoke()
        self.assert_original()
        self.invoke()
        backup = self.bundle / '.previous-managed'
        journal = json.loads((backup / 'journal.json').read_text())
        journal['state'] = 'pending'
        (backup / 'journal.json').write_text(json.dumps(journal))
        (self.bundle / 'ref6.py').write_bytes(b'post-crash edit')
        with self.assertRaisesRegex(ValueError, 'Edited file blocks recovery'):
            self.invoke()
        self.assertEqual((self.bundle / 'ref6.py').read_bytes(), b'post-crash edit')

    def test_limits_and_version(self):
        for limit in ('MAX_ARCHIVE', 'MAX_EXPANDED', 'MAX_MEMBERS'):
            with patch.object(updater, limit, 1):
                with self.assertRaises(ValueError):
                    self.invoke()
            self.assert_original()
        with self.assertRaisesRegex(ValueError, 'version mismatch'):
            self.invoke(version='v3')
        self.assert_original()


class DownloadTests(unittest.TestCase):
    def test_url_policy(self):
        for url in ('http://example.com/a', 'file:///tmp/a', 'ftp://example.com/a',
                    'https://user:pass@example.com/a', 'http://127.0.0.1/a'):
            with self.assertRaises(ValueError):
                updater.check_url(url)
        updater.check_url('https://example.com/a')
        updater.check_url('http://127.0.0.1/a', True)
        updater.check_url('http://[::1]/a', True)
        with self.assertRaises(ValueError):
            updater.check_url('http://example.com/a', True)

    def test_redirect_cannot_downgrade_https(self):
        redirect = updater.SafeRedirect(False)
        with self.assertRaises(ValueError):
            redirect.redirect_request(None, None, 302, '', {}, 'http://example.com/zip')

    def test_download_bound_and_tls_context(self):
        class Response(io.BytesIO):
            def geturl(self):
                return 'https://example.com/a'

        with tempfile.TemporaryDirectory() as raw:
            for data, limit in ((b'abc', 3), (b'abcd', 3)):
                with patch.object(updater.urllib.request, 'build_opener') as build:
                    build.return_value.open.return_value = Response(data)
                    if len(data) > limit:
                        with self.assertRaisesRegex(ValueError, 'size limit'):
                            updater.download('https://example.com/a', Path(raw) / 'data', limit)
                    else:
                        updater.download('https://example.com/a', Path(raw) / 'data', limit)
                        self.assertEqual((Path(raw) / 'data').read_bytes(), data)
                    handler = build.call_args.args[1]
                    self.assertEqual(handler._context.verify_mode, updater.ssl.CERT_REQUIRED)
                    self.assertTrue(handler._context.check_hostname)


if __name__ == '__main__':
    unittest.main()
