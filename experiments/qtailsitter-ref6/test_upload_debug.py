import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import upload_debug as uploads


class UploadTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.archive = self.root/'flight.zip'
        self.archive.write_bytes(b'closed evidence')
        self.config = self.root/'private/upload.json'
        uploads.save_json(self.config, {'enabled': True, 'url': uploads.DEFAULT_URL,
                                       'token': 'fixture-only-'*4}, private=True)
        self.receipt = {'ok': True, 'sha256': uploads.file_hash(self.archive),
                        'bytes': self.archive.stat().st_size}

    def test_verified_receipt_and_no_credentials_in_sidecars(self):
        with patch('upload_debug.send_once', return_value=self.receipt):
            self.assertTrue(uploads.upload(self.archive, self.config))
        sidecar = self.archive.with_suffix('.zip.upload.json')
        self.assertEqual(json.loads(sidecar.read_text()), self.receipt)
        self.assertNotIn('token', sidecar.read_text())
        self.assertFalse(self.archive.with_suffix('.zip.pending.json').exists())

    def test_auth_error_retains_pending_and_archive(self):
        with patch('upload_debug.send_once', side_effect=ValueError('Upload HTTP 401')) as send:
            self.assertFalse(uploads.upload(self.archive, self.config))
        self.assertEqual(send.call_count, 1)
        self.assertTrue(self.archive.exists())
        self.assertTrue(self.archive.with_suffix('.zip.pending.json').exists())
        with patch('upload_debug.send_once', return_value=self.receipt):
            uploads.retry_pending(self.root, self.config)
        self.assertFalse(self.archive.with_suffix('.zip.pending.json').exists())

    def test_transient_error_retries(self):
        with patch('upload_debug.send_once', side_effect=[OSError('network unavailable'), self.receipt]) as send, \
                patch('upload_debug.time.sleep'):
            self.assertTrue(uploads.upload(self.archive, self.config))
        self.assertEqual(send.call_count, 2)

    def test_disabled_and_private_file_requirement(self):
        uploads.disable(self.config)
        with patch('upload_debug.send_once') as send:
            self.assertFalse(uploads.upload(self.archive, self.config))
            send.assert_not_called()
        self.config.chmod(0o644)
        with self.assertRaisesRegex(ValueError, 'private'):
            uploads.settings(self.config)

    def test_size_and_url_limits(self):
        with patch('upload_debug.MAX_BYTES', 2), self.assertRaisesRegex(ValueError, '256'):
            uploads.upload(self.archive, self.config)
        for url in ('http://example.com/upload', 'https://user:pass@example.com/',
                    'https://example.com/?token=secret', 'https://example.com/#fragment'):
            with self.assertRaises(ValueError):
                uploads.endpoint(url)

    def test_receipt_validation_uses_actual_http_response(self):
        class Response:
            status = 200
            def read(self, _):
                return b'{"ok":true,"sha256":"wrong","bytes":15}'
        with patch('upload_debug.http.client.HTTPSConnection') as connection:
            connection.return_value.getresponse.return_value = Response()
            with self.assertRaisesRegex(ValueError, 'receipt'):
                uploads.send_once(self.archive, uploads.settings(self.config), self.receipt['sha256'],
                                  self.receipt['bytes'])


if __name__ == '__main__':
    unittest.main()
