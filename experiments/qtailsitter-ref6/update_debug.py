#!/usr/bin/env python3
"""Update a Ref6 bundle in place. The caller MUST already own .ref6.lock.

HTTPS authenticates the manifest; its SHA256 authenticates the ZIP. Neither
checksum file is a signature. Only receipted aircraft bytes are installed;
flight runs and credentials are never updated.
"""
import argparse
import csv
import fcntl
import hashlib
import ipaddress
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import ssl
import stat
import subprocess
import tempfile
import urllib.parse
import urllib.request
import zipfile

MAX_ARCHIVE = 32 * 1024 * 1024
MAX_EXPANDED = 256 * 1024 * 1024
MAX_MEMBERS = 10000
ROOT_FILES = frozenset('''ref6.py flight.py collect.py source_state.py gates.py
locate_inputs.py attach_truth.py update_debug.py test_update_debug.py test_ref6.py
run.sh qgc-firewall.ps1 BENCH_REVIEW.md PROPULSION_REVIEW.py PROPULSION_REVIEW.md
diagnostics.bundle diagnostic-sha.txt QUICKSTART.md README.md COMMANDS.md
profile.sh parameters.json design.json VERSION.txt SOURCE.md REPORT.md'''.split())
ROOT_FILES = ROOT_FILES | frozenset(('upload_debug.py', 'test_upload_debug.py',
                                    'test_live_snapshot.py', 'test_allocator_contract.py',
                                    'test_shutdown_workflow.py', 'sizing.json'))
AIRCRAFT = 'aircraft/QuadTailsitterRef6/'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def safe_name(name):
    require(isinstance(name, str) and name and '\\' not in name and ':' not in name
            and not any(ord(c) < 32 for c in name), 'Unsafe path: ' + repr(name))
    parts = name.split('/')
    require(all(p not in ('', '.', '..') for p in parts), 'Unsafe path: ' + name)
    require(not PurePosixPath(name).is_absolute(), 'Absolute path refused')
    return name


def managed(name):
    safe_name(name)
    return (name in ROOT_FILES or name.startswith(AIRCRAFT)
            or (name.startswith('evidence/') and Path(name).suffix in
                ('.svg', '.json', '.py', '.cpp', '.hpp', '.patch'))
            or name in ('build_tools/build_qtailsitter_reference.py',
                        'build_tools/qtailsitter_sizing.py',
                        'build_tools/analyze_qtailsitter_design.py'))


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def checked_path(root, name):
    safe_name(name)
    require(not root.is_symlink(), 'Symlink refused: ' + str(root))
    path = root
    for part in name.split('/'):
        path = path / part
        require(not path.is_symlink(), 'Symlink refused: ' + str(path))
    return path


def checksums(path):
    result = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        match = re.fullmatch(r'([0-9a-fA-F]{64}) [ *](.+)', line)
        require(match is not None, 'Invalid SHA256SUMS line')
        name = safe_name(match[2])
        require(name != 'SHA256SUMS' and name not in result, 'Duplicate/self checksum: ' + name)
        result[name] = match[1].lower()
    require(bool(result), 'Empty SHA256SUMS')
    return result


def check_url(url, allow_loopback_http=False):
    parsed = urllib.parse.urlsplit(url)
    loopback = False
    try:
        loopback = ipaddress.ip_address(parsed.hostname or '').is_loopback
    except ValueError:
        pass
    require(parsed.hostname and not parsed.username and not parsed.password
            and (parsed.scheme == 'https' or
                 (allow_loopback_http and parsed.scheme == 'http' and loopback)),
            'HTTPS URL required (HTTP only for explicit numeric-loopback tests)')


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def __init__(self, allow_loopback_http):
        self.allow_loopback_http = allow_loopback_http

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        check_url(newurl, self.allow_loopback_http)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def download(url, target, limit, allow_loopback_http=False):
    check_url(url, allow_loopback_http)
    opener = urllib.request.build_opener(
        SafeRedirect(allow_loopback_http),
        urllib.request.HTTPSHandler(context=ssl.create_default_context()))
    with opener.open(url, timeout=30) as response, target.open('wb') as output:
        check_url(response.geturl(), allow_loopback_http)
        size = 0
        while True:
            block = response.read(min(1024 * 1024, limit - size + 1))
            if not block:
                break
            size += len(block)
            require(size <= limit, 'Download exceeds size limit')
            output.write(block)


def unpack(archive, target):
    require(archive.stat().st_size <= MAX_ARCHIVE, 'Archive exceeds 32MB')
    with zipfile.ZipFile(archive) as zipped:
        members = zipped.infolist()
        require(len(members) <= MAX_MEMBERS, 'Too many ZIP members')
        total = 0
        names = set()
        files = []
        for info in members:
            name = safe_name(info.filename[:-1] if info.is_dir() else info.filename)
            require(name not in names, 'Duplicate ZIP member: ' + name)
            names.add(name)
            mode = info.external_attr >> 16
            kind = stat.S_IFMT(mode)
            require(kind in (0, stat.S_IFREG, stat.S_IFDIR)
                    and (kind != stat.S_IFDIR or info.is_dir())
                    and not info.flag_bits & 1, 'ZIP symlink/special/encrypted member refused')
            total += info.file_size
            require(total <= MAX_EXPANDED, 'Expanded ZIP exceeds 256MB')
            if not info.is_dir():
                files.append((name, info))
        require(bool(files), 'Empty ZIP')
        # Accept either a flat payload or the packager's single wrapper directory.
        flat = any(name == 'SHA256SUMS' for name, _ in files)
        prefix = '' if flat else files[0][0].split('/')[0] + '/'
        require(all(name == prefix.rstrip('/') or name.startswith(prefix) for name in names),
                'Multiple ZIP roots')
        extracted = set()
        for name, info in files:
            relative = safe_name(name[len(prefix):])
            path = target / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            with zipped.open(info) as source, path.open('xb') as output:
                shutil.copyfileobj(source, output, 1024 * 1024)
            os.chmod(path, 0o755 if (info.external_attr >> 16) & 0o111 else 0o644)
            extracted.add(relative)
    sums = checksums(target / 'SHA256SUMS')
    require(extracted == set(sums) | {'SHA256SUMS'}, 'Incomplete internal SHA256SUMS')
    for name, expected in sums.items():
        require(digest(target / name) == expected, 'Internal hash mismatch: ' + name)
        require(managed(name), 'Unmanaged incoming file refused: ' + name)
    return sums


def stopped():
    result = subprocess.run(['pgrep', '-x', 'px4'], capture_output=True, text=True)
    require(result.returncode in (0, 1), 'Cannot inspect PX4 processes')
    require(result.returncode == 1, 'PX4 is running; stop it before updating')


def atomic_file(source, destination, stage):
    destination.parent.mkdir(parents=True, exist_ok=True)
    # The installation may be on Windows/another filesystem; replace locally.
    fd, name = tempfile.mkstemp(prefix='.ref6-write-', dir=destination.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'wb') as output, source.open('rb') as stream:
            shutil.copyfileobj(stream, output)
            output.flush()
            os.fsync(output.fileno())
        shutil.copymode(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def journal_write(backup, journal, stage):
    source = stage / 'journal.json'
    source.write_text(json.dumps(journal), encoding='utf-8')
    atomic_file(source, backup / 'journal.json', stage)


def assets_stopped():
    result = subprocess.run(['pgrep', '-x', r'X-Plane(-x86_64|\.exe)?'], capture_output=True, text=True)
    require(result.returncode == 1, 'Quit X-Plane before the aircraft update')
    if shutil.which('tasklist.exe'):
        result = subprocess.run(['tasklist.exe', '/FO', 'CSV', '/NH'], capture_output=True, text=True)
        require(result.returncode == 0, 'Cannot inspect Windows processes')
        names = {row[0].casefold() for row in csv.reader(io.StringIO(result.stdout)) if row}
        require('x-plane.exe' not in names, 'Quit Windows X-Plane before updating')
    else:
        require(input('Quit X-Plane, then type XPLANE CLOSED: ').strip() == 'XPLANE CLOSED',
                'Aircraft update cancelled')


def installed_assets(bundle, payload, old_acf, new_acf, version, prepared, stage):
    require(set(old_acf) == set(new_acf) and len(old_acf) == 1,
            'Aircraft additions/removals are not supported by an in-place update')
    receipt_path = checked_path(bundle, 'installation/receipt.json')
    # An uninstalled bundle owns no external aircraft.
    if not receipt_path.exists():
        return []
    receipt = json.loads(receipt_path.read_text())
    require(not receipt.get('restored'), 'Installation restored; reinstall before updating assets')
    name = next(iter(old_acf))
    acf = Path(receipt['aircraft']) / Path(name).name
    require(acf.is_absolute() and acf.resolve(strict=True) == acf and not acf.is_symlink(),
            'Installed ACF path must be absolute and not a symlink')
    require(digest(acf) == receipt['acf_sha256'] == old_acf[name],
            'Installed aircraft has local edits or differs from this bundle')
    assets_stopped()
    revised = {**receipt, 'version': version, 'acf_sha256': new_acf[name]}
    receipt_source = stage / 'receipt-new.json'
    receipt_source.write_text(json.dumps(revised, indent=2)+'\n')
    assets = []
    for index, (target, source) in enumerate(((acf, payload/name), (receipt_path, receipt_source))):
        saved = prepared / 'installed' / str(index)
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target, saved)
        assets.append({'target': str(target), 'backup': str(index),
                       'old': digest(saved), 'new': digest(source)})
    return assets


def recover(bundle, stage):
    backup = checked_path(bundle, '.previous-managed')
    if not backup.exists():
        return
    journal_path = checked_path(backup, 'journal.json')
    journal = json.loads(journal_path.read_text())
    require(journal['state'] in ('pending', 'complete', 'rolled-back'), 'Invalid update journal')
    if journal['state'] != 'pending':
        return
    assets = journal.get('installed', [])
    if assets:
        assets_stopped()
    for asset in assets:
        target = Path(asset['target'])
        require(target.is_absolute() and target.resolve(strict=True) == target,
                'Invalid installed rollback path')
        require(digest(target) in (asset['old'], asset['new']), 'Edited installed file blocks rollback')
        require(asset['backup'] in ('0', '1') and
                digest(backup/'installed'/asset['backup']) == asset['old'], 'Corrupt installed backup')
    old, new = journal['old'], journal['new']
    for name in old.keys() | new.keys():
        require(name == 'SHA256SUMS' or managed(name), 'Unmanaged rollback path')
        path = checked_path(bundle, name)
        require(not path.exists() or (path.is_file() and digest(path) in
                {old.get(name), new.get(name)}), 'Edited file blocks recovery: ' + name)
        if name in old:
            source = checked_path(backup / 'files', name)
            require(digest(source) == old[name], 'Corrupt rollback file: ' + name)
    # Validate the entire rollback before changing any destination.
    for asset in assets:
        atomic_file(backup/'installed'/asset['backup'], Path(asset['target']), stage)
    for name in sorted(old.keys() | new.keys(), key=lambda n: n == 'SHA256SUMS'):
        path = checked_path(bundle, name)
        if name in old:
            atomic_file(backup / 'files' / name, path, stage)
        else:
            path.unlink(missing_ok=True)
    journal['state'] = 'rolled-back'
    journal_write(backup, journal, stage)


def current_files(bundle):
    sums_path = checked_path(bundle, 'SHA256SUMS')
    old = {name: value for name, value in checksums(sums_path).items() if managed(name)}
    for name, expected in old.items():
        path = checked_path(bundle, name)
        require(path.is_file() and digest(path) == expected, 'Local edits/missing managed file: ' + name)
    old['SHA256SUMS'] = digest(sums_path)
    return old


def update(bundle, manifest_url, *, allow_loopback_http=False):
    """Caller holds bundle/.ref6.lock throughout; return manifest version on success."""
    bundle = Path(bundle).resolve(strict=True)
    stopped()
    with tempfile.TemporaryDirectory(prefix='.ref6-update-', dir=bundle.parent) as raw:
        stage = Path(raw)
        recover(bundle, stage)
        old = current_files(bundle)
        download(manifest_url, stage / 'manifest.json', 64 * 1024, allow_loopback_http)
        manifest = json.loads((stage / 'manifest.json').read_text())
        require(isinstance(manifest, dict) and set(manifest) == {'version', 'url', 'sha256'},
                'Manifest requires version, url, sha256')
        require(isinstance(manifest['version'], str) and manifest['version'].strip(), 'Invalid version')
        require(isinstance(manifest['sha256'], str)
                and re.fullmatch('[0-9a-fA-F]{64}', manifest['sha256']), 'Invalid archive SHA256')
        download(manifest['url'], stage / 'update.zip', MAX_ARCHIVE, allow_loopback_http)
        require(digest(stage / 'update.zip') == manifest['sha256'].lower(), 'Archive hash mismatch')
        payload = stage / 'payload'
        payload.mkdir()
        new = unpack(stage / 'update.zip', payload)
        require('VERSION.txt' in new and (payload / 'VERSION.txt').read_text().strip()
                == manifest['version'], 'Manifest/internal version mismatch')
        old_acf = {n: h for n, h in old.items() if n.lower().endswith('.acf')}
        new_acf = {n: h for n, h in new.items() if n.lower().endswith('.acf')}
        require(bool(old_acf) and set(new_acf) == set(old_acf),
                'ACF additions/removals need an asset install; update refused')
        new['SHA256SUMS'] = digest(payload / 'SHA256SUMS')
        for name in new.keys() - old.keys():
            require(not checked_path(bundle, name).exists(), 'Unowned file collision: ' + name)
        stopped()
        require(current_files(bundle) == old, 'Managed files changed during download')
        if old == new:
            return manifest['version']
        backup = checked_path(bundle, '.previous-managed')
        # Build the backup completely off to the side before publishing its journal.
        prepared = stage / 'backup'
        for name in old:
            destination = prepared / 'files' / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(bundle / name, destination)
            require(digest(destination) == old[name], 'Managed file changed during backup: ' + name)
        assets = installed_assets(bundle, payload, old_acf, new_acf, manifest['version'], prepared, stage) if old_acf != new_acf else []
        journal = {'state': 'pending', 'old': old, 'new': new, 'installed': assets}
        journal_write(prepared, journal, stage)
        if backup.exists():
            require(not any(p.is_symlink() for p in backup.rglob('*')), 'Symlink in rollback directory')
            shutil.rmtree(backup)
        os.replace(prepared, backup)
        try:
            if assets:
                name = next(iter(new_acf))
                for asset, source in zip(assets, (payload/name, stage/'receipt-new.json')):
                    require(digest(Path(asset['target'])) == asset['old'], 'Installed file changed during update')
                    atomic_file(source, Path(asset['target']), stage)
            for name in sorted(new, key=lambda n: n == 'SHA256SUMS'):
                if old.get(name) != new[name]:
                    atomic_file(payload / name, checked_path(bundle, name), stage)
            # Only checksummed, previously owned files absent from the verified payload.
            for name in old.keys() - new.keys():
                checked_path(bundle, name).unlink()
            journal['state'] = 'complete'
            journal_write(backup, journal, stage)
        except BaseException:
            recover(bundle, stage)
            raise
        return manifest['version']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest_url')
    parser.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    with (args.bundle/'.ref6.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print('Updated Ref6 to', update(args.bundle, args.manifest_url))


if __name__ == '__main__':
    main()
