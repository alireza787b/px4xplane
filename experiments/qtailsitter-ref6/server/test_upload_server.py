import hashlib
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer
from upload_server import app, Settings


class IntakeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.settings = Settings(self.root, 'fixture-private-token-'*3, time.time()+3600,
                                 max_bytes=1024, quota_bytes=4096, free_floor=0)
        self.client = TestClient(TestServer(app(self.settings)))
        await self.client.start_server()
        self.addAsyncCleanup(self.client.close)
        self.data = b'closed archive fixture'
        self.sha = hashlib.sha256(self.data).hexdigest()
        self.headers = {'Authorization': 'Bearer '+self.settings.token, 'X-Content-SHA256': self.sha}

    async def test_checksum_acknowledgement_and_idempotency(self):
        for _ in range(2):
            response = await self.client.post('/px4xplane-debug/upload', data=self.data, headers=self.headers)
            self.assertEqual(response.status, 200)
            self.assertEqual((await response.json())['sha256'], self.sha)
        self.assertEqual((self.root/(self.sha+'.zip')).read_bytes(), self.data)
        self.assertEqual(len(list(self.root.glob('*.zip'))), 1)
        self.assertFalse(list(self.root.glob('*.partial')))

    async def test_auth_and_expiry(self):
        response = await self.client.post('/px4xplane-debug/upload', data=self.data)
        self.assertEqual(response.status, 401)
        self.settings.expires_at = 0
        response = await self.client.post('/px4xplane-debug/upload', data=self.data, headers=self.headers)
        self.assertEqual(response.status, 401)
        self.assertFalse(list(self.root.iterdir()))

    async def test_wrong_checksum_rejected_without_partial(self):
        response = await self.client.post('/px4xplane-debug/upload', data=b'wrong', headers=self.headers)
        self.assertEqual(response.status, 400)
        self.assertFalse(list(self.root.iterdir()))

    async def test_size_quota_and_disk_bounds(self):
        response = await self.client.post('/px4xplane-debug/upload', data=b'x'*1025, headers=self.headers)
        self.assertEqual(response.status, 413)
        self.settings.quota_bytes = 1
        response = await self.client.post('/px4xplane-debug/upload', data=self.data, headers=self.headers)
        self.assertEqual(response.status, 507)
        self.settings.quota_bytes = 4096
        self.settings.free_floor = 999
        with patch('upload_server.shutil.disk_usage', return_value=type('Disk', (), {'free': 1})()):
            response = await self.client.post('/px4xplane-debug/upload', data=self.data, headers=self.headers)
        self.assertEqual(response.status, 507)
        self.assertFalse(list(self.root.iterdir()))

    async def test_no_download_and_no_path_traversal(self):
        response = await self.client.get('/px4xplane-debug/upload')
        self.assertEqual(response.status, 405)
        headers = {**self.headers, 'X-Content-SHA256': '../../escape'}
        response = await self.client.post('/px4xplane-debug/upload', data=self.data, headers=headers)
        self.assertEqual(response.status, 400)


if __name__ == '__main__':
    unittest.main()
