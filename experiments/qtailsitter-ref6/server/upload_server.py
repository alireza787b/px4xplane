"""Bounded, upload-only private debug intake behind the existing HTTPS proxy."""
import asyncio
from dataclasses import dataclass
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import time

from aiohttp import web


@dataclass
class Settings:
    root: Path
    token: str
    expires_at: float
    max_bytes: int = 256*1024*1024
    quota_bytes: int = 512*1024*1024
    free_floor: int = 384*1024*1024


def app(settings):
    settings.root.mkdir(parents=True, exist_ok=True)
    lock = asyncio.Lock()

    async def preflight(request):
        # Check admission before a large body is sent. Early POST rejection can
        # otherwise surface as a TLS EOF in a streaming client behind a proxy.
        if time.time() >= settings.expires_at or not hmac.compare_digest(
                request.headers.get('Authorization', ''), 'Bearer '+settings.token):
            raise web.HTTPUnauthorized(text='Invalid or expired debug upload credentials')
        try:
            size = int(request.headers.get('X-Upload-Bytes', '0'))
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid upload size')
        digest = request.headers.get('X-Content-SHA256', '')
        if not 0 < size <= settings.max_bytes:
            raise web.HTTPRequestEntityTooLarge(max_size=settings.max_bytes, actual_size=size)
        if not re.fullmatch('[a-f0-9]{64}', digest):
            raise web.HTTPBadRequest(text='Missing SHA256')
        async with lock:
            final = settings.root/(digest+'.zip')
            if final.is_file():
                if final.stat().st_size != size:
                    raise web.HTTPConflict(text='Existing object size mismatch')
            else:
                used = sum(p.stat().st_size for p in settings.root.iterdir() if p.is_file())
                if used+size > settings.quota_bytes or shutil.disk_usage(settings.root).free-size < settings.free_floor:
                    raise web.HTTPInsufficientStorage(text='Debug quota/free-space limit; keep local evidence')
        # This is not a reservation; POST checks the bounds again under lock.
        return web.Response(status=204)

    async def receive(request):
        auth = request.headers.get('Authorization', '')
        if time.time() >= settings.expires_at or not hmac.compare_digest(auth, 'Bearer '+settings.token):
            raise web.HTTPUnauthorized(text='Invalid or expired debug upload credentials')
        size = request.content_length
        digest = request.headers.get('X-Content-SHA256', '')
        if size is None or not 0 < size <= settings.max_bytes:
            raise web.HTTPRequestEntityTooLarge(max_size=settings.max_bytes, actual_size=size or 0)
        if not re.fullmatch('[a-f0-9]{64}', digest):
            raise web.HTTPBadRequest(text='Missing SHA256')
        if request.headers.get('Content-Encoding'):
            raise web.HTTPBadRequest(text='Encoded request bodies are not accepted')
        async with lock:
            final = settings.root/(digest+'.zip')
            receipt = {'ok': True, 'sha256': digest, 'bytes': size}
            if final.is_file():
                if final.stat().st_size != size:
                    raise web.HTTPConflict(text='Existing object size mismatch')
                # Drain retry bytes so a lost acknowledgement can be retried safely.
                total = 0
                check = hashlib.sha256()
                async for chunk in request.content.iter_chunked(1024*1024):
                    total += len(chunk)
                    if total > size:
                        raise web.HTTPBadRequest(text='Body length mismatch')
                    check.update(chunk)
                if total != size or check.hexdigest() != digest:
                    raise web.HTTPBadRequest(text='Body checksum mismatch')
                return web.json_response(receipt)
            used = sum(p.stat().st_size for p in settings.root.iterdir() if p.is_file())
            if used+size > settings.quota_bytes or shutil.disk_usage(settings.root).free-size < settings.free_floor:
                raise web.HTTPInsufficientStorage(text='Debug quota/free-space limit; keep local evidence')
            fd, temporary = tempfile.mkstemp(prefix='.upload-', suffix='.partial', dir=settings.root)
            path = Path(temporary)
            try:
                check = hashlib.sha256()
                total = 0
                with os.fdopen(fd, 'wb') as output:
                    async for chunk in request.content.iter_chunked(1024*1024):
                        total += len(chunk)
                        if total > size:
                            raise web.HTTPBadRequest(text='Body length mismatch')
                        check.update(chunk)
                        output.write(chunk)
                    if total != size or check.hexdigest() != digest:
                        raise web.HTTPBadRequest(text='Body checksum mismatch')
                    output.flush()
                    os.fsync(output.fileno())
                os.link(path, final)
                record = {**receipt, 'received_at': time.time()}
                record_path = settings.root/(digest+'.json')
                with record_path.open('x') as output:
                    json.dump(record, output)
                    output.flush()
                    os.fsync(output.fileno())
                print('Accepted debug archive', digest, size, flush=True)
                return web.json_response(receipt)
            finally:
                path.unlink(missing_ok=True)

    async def bounded(request):
        try:
            return await asyncio.wait_for(receive(request), timeout=180)
        except asyncio.TimeoutError:
            raise web.HTTPRequestTimeout(text='Upload timeout; retain local evidence')

    application = web.Application(client_max_size=settings.max_bytes)
    application.router.add_post('/px4xplane-debug/upload', bounded)
    application.router.add_head('/px4xplane-debug/upload', preflight)
    return application


if __name__ == '__main__':
    token = os.environ['PX4_DEBUG_UPLOAD_TOKEN']
    if len(token) < 32:
        raise SystemExit('Upload token is too short')
    settings = Settings(Path(os.environ['PX4_DEBUG_UPLOAD_ROOT']), token,
                        float(os.environ['PX4_DEBUG_UPLOAD_EXPIRES']))
    web.run_app(app(settings), host='127.0.0.1', port=18764, access_log=None)
