"""Private debug uploads. No credentials in bundles, command lines or evidence."""
import getpass
import hashlib
import http.client
import json
import os
from pathlib import Path
import ssl
import tempfile
import time
from urllib.parse import urlsplit

CONFIG = Path.home()/'.config/px4xplane-debug/upload.json'
DEFAULT_URL = 'https://de2.joomtalk.ir/px4xplane-debug/upload'
MAX_BYTES = 256*1024*1024


def save_json(path, value, private=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    if private:
        path.parent.chmod(0o700)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.upload-')
    try:
        with os.fdopen(fd, 'w') as output:
            json.dump(value, output, indent=2)
            output.write('\n')
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def endpoint(raw):
    url = urlsplit(raw)
    if url.scheme != 'https' or not url.hostname or url.username or url.password or url.query or url.fragment:
        raise ValueError('Upload URL must be HTTPS without credentials, query or fragment')
    return url


def configure(config=CONFIG, url=DEFAULT_URL):
    endpoint(url)
    print('DEBUG ONLY: uploads include flight position, configuration and truth data.')
    print('They go to your private analysis server, not a public flight-log site.')
    token = getpass.getpass('Paste the upload-only token from Telegram (hidden): ').strip()
    if len(token) < 24 or any(c.isspace() for c in token):
        raise ValueError('Invalid upload token; nothing saved')
    save_json(config, {'enabled': True, 'url': url, 'token': token}, private=True)
    print('Debug uploads enabled. Credentials are private and excluded from evidence.')


def settings(config=CONFIG):
    if not config.is_file():
        return None
    if config.is_symlink() or config.stat().st_mode & 0o077:
        raise ValueError('Upload config must be a regular private file (chmod 600)')
    data = json.loads(config.read_text())
    if not data.get('enabled'):
        return None
    endpoint(data['url'])
    if len(data.get('token', '')) < 24:
        raise ValueError('Invalid upload configuration')
    return data


def disable(config=CONFIG):
    if config.is_file():
        data = json.loads(config.read_text())
        data['enabled'] = False
        save_json(config, data, private=True)
    print('Debug uploads disabled. Local evidence remains enabled.')


def file_hash(archive):
    digest = hashlib.sha256()
    with archive.open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def send_once(archive, data, digest, size):
    url = endpoint(data['url'])
    connection = http.client.HTTPSConnection(url.hostname, url.port or 443,
                                             timeout=30, context=ssl.create_default_context())
    try:
        connection.putrequest('POST', url.path or '/')
        connection.putheader('Authorization', 'Bearer '+data['token'])
        connection.putheader('Content-Type', 'application/zip')
        connection.putheader('Content-Length', str(size))
        connection.putheader('X-Content-SHA256', digest)
        connection.endheaders()
        deadline = time.monotonic()+120
        with archive.open('rb') as source:
            for chunk in iter(lambda: source.read(1024*1024), b''):
                if time.monotonic() > deadline:
                    raise TimeoutError('Upload deadline exceeded')
                connection.send(chunk)
        response = connection.getresponse()
        body = response.read(8193)
        if response.status != 200:
            # Never echo server content or request headers containing credentials.
            if response.status in (500, 502, 503, 504):
                raise http.client.HTTPException(f'Temporary upload HTTP {response.status}')
            raise ValueError(f'Upload HTTP {response.status}; local archive retained')
        if len(body) > 8192:
            raise ValueError('Oversized upload receipt')
        receipt = json.loads(body)
        if receipt.get('ok') is not True or receipt.get('sha256') != digest or receipt.get('bytes') != size:
            raise ValueError('Upload receipt checksum/size mismatch; local archive retained')
        return receipt
    finally:
        connection.close()


def upload(archive, config=CONFIG, attempts=3):
    archive = Path(archive)
    if archive.is_symlink():
        raise ValueError('Symlink archives are not uploaded')
    archive = archive.resolve()
    data = settings(config)
    if data is None:
        print('Auto-upload OFF/unconfigured; local ZIP retained:', archive)
        return False
    if not archive.is_file() or archive.is_symlink():
        raise ValueError('Upload requires a closed local archive')
    before = archive.stat()
    if not 0 < before.st_size <= MAX_BYTES:
        raise ValueError('Archive exceeds the 256 MiB debug upload limit; local copy retained')
    digest = file_hash(archive)
    pending = archive.with_suffix(archive.suffix+'.pending.json')
    acknowledgement = archive.with_suffix(archive.suffix+'.upload.json')
    save_json(pending, {'archive': str(archive), 'sha256': digest, 'bytes': before.st_size,
                        'status': 'pending', 'created_at': time.time()})
    for attempt in range(attempts):
        try:
            receipt = send_once(archive, data, digest, before.st_size)
            after = archive.stat()
            if (after.st_size, after.st_mtime_ns) != (before.st_size, before.st_mtime_ns):
                raise ValueError('Archive changed during upload; receipt not accepted')
            save_json(acknowledgement, receipt)
            pending.unlink(missing_ok=True)
            print('UPLOAD VERIFIED:', receipt['sha256'], 'bytes:', receipt['bytes'])
            return True
        except (OSError, ValueError, http.client.HTTPException) as error:
            print('Upload attempt failed:', str(error))
            if isinstance(error, ValueError) or attempt+1 == attempts:
                break
            time.sleep(2**(attempt+1))
    print('PENDING upload retained. Run: bash run.sh retry-uploads')
    return False


def retry_pending(root, config=CONFIG):
    paths = sorted(root.glob('*.zip.pending.json'))
    if len(paths) > 100:
        raise ValueError('Too many pending archives; inspect before retrying')
    for path in paths:
        record = json.loads(path.read_text())
        archive = path.with_suffix('').with_suffix('')
        if not archive.is_file() or file_hash(archive) != record['sha256']:
            print('Pending archive missing/changed; retained for inspection:', archive)
            continue
        upload(archive, config)
