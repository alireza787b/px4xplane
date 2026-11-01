#!/usr/bin/env python3
"""Capture stable, top-level TruthCapture session files without scanning the drive."""
import json
import csv
import hashlib
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

from locate_inputs import path


MAX_WINDOW = 256 * 1024 * 1024
TAIL_BYTES = 1024 * 1024
MAX_METADATA = 8 * 1024 * 1024
SNAPSHOT_FILES = ('manifest.json', 'datarefs.json', 'datarefs.csv', 'datarefs.txt',
                  'capture_config.ini', 'capture_config.json')


def _identity(stat):
    # Do not use ctime/mtime: both can change on a legitimate append (also on WSL).
    return [stat.st_dev, stat.st_ino]


def _regular(file):
    if file.is_symlink() or not file.is_file():
        raise ValueError(f'Not a regular capture file: {file}')


def _metadata(source):
    result = {}
    for name in SNAPSHOT_FILES:
        file = source/name
        if name != 'manifest.json' and not file.exists() and not file.is_symlink():
            continue
        _regular(file)
        with file.open('rb') as stream:
            data = stream.read(MAX_METADATA + 1)
        if len(data) > MAX_METADATA:
            raise ValueError(f'Capture metadata too large: {name}')
        result[name] = data
    json.loads(result['manifest.json'].decode('utf-8-sig'))
    return result


def _hashes(metadata):
    return {name: hashlib.sha256(data).hexdigest() for name, data in metadata.items()}


def _header(stream):
    stream.seek(0)
    header = stream.readline(TAIL_BYTES + 1)
    if not header.endswith(b'\n') or len(header) > TAIL_BYTES:
        raise ValueError('Missing or oversized complete CSV header.')
    fields = next(csv.reader([header.decode('utf-8-sig')], strict=True))
    if len(set(fields)) != len(fields) or 'frame_id' not in fields:
        raise ValueError('CSV header must contain unique columns and frame_id.')
    time_field = next((k for k in ('sim/time/total_flight_time_sec', 'sim_time_s')
                       if k in fields), None)
    if time_field is None:
        raise ValueError('CSV header is missing simulation time.')
    return header, fields, time_field


def _complete_end(stream, size):
    start = max(0, size - TAIL_BYTES)
    stream.seek(start)
    tail = stream.read(size - start)
    newline = tail.rfind(b'\n')
    if newline < 0:
        raise ValueError('No complete CSV row in bounded tail.')
    return start + newline + 1


def _point(row, fields, time_field):
    if len(row) != len(fields):
        raise ValueError('Incomplete or malformed CSV row.')
    try:
        frame = int(row[fields.index('frame_id')])
        sim_time = float(row[fields.index(time_field)])
    except (ValueError, OverflowError) as error:
        raise ValueError('Invalid frame/time in CSV row.') from error
    if frame < 0 or not math.isfinite(sim_time):
        raise ValueError('Invalid frame/time in CSV row.')
    return {'frame_id': frame, 'sim_time_s': sim_time}


def _range_hash(stream, start, end, output=None):
    stream.seek(start)
    digest = hashlib.sha256()
    remaining = end - start
    while remaining:
        chunk = stream.read(min(1024 * 1024, remaining))
        if not chunk:
            raise ValueError('Capture truncated during snapshot.')
        digest.update(chunk)
        if output is not None:
            output.write(chunk)
        remaining -= len(chunk)
    return digest.hexdigest()


def begin_snapshot(source):
    """Return JSON-serializable preflight state; exclude all preflight rows.

    Only the header and a bounded tail are read, even for multi-GB sessions.
    Persist this result in the run directory before starting the flight.
    """
    source = Path(source).resolve()
    file = source/'frames.csv'
    _regular(file)
    metadata = _metadata(source)
    with file.open('rb', buffering=0) as stream:
        stat = os.fstat(stream.fileno())
        header, fields, time_field = _header(stream)
        end = _complete_end(stream, stat.st_size)
        start = max(len(header), end - TAIL_BYTES)
        stream.seek(start)
        tail = stream.read(end - start)
        lines = tail.splitlines(keepends=True)
        if start > len(header):
            lines = lines[1:]  # The first bounded-tail line may begin mid-row.
        if not lines:
            raise ValueError('No complete data rows at preflight.')
        last = _point(next(csv.reader([lines[-1].decode('utf-8')], strict=True)),
                      fields, time_field)
        anchor_hash = hashlib.sha256(tail).hexdigest()
        if (_identity(file.stat()) != _identity(stat) or file.stat().st_size < stat.st_size
                or _range_hash(stream, start, end) != anchor_hash
                or _header(stream)[0] != header or _metadata(source) != metadata):
            raise ValueError('Capture changed during preflight.')
    return {'version': 1, 'source': str(source), 'header': header.decode('utf-8'),
            'start_byte': end, 'frames_bytes': stat.st_size, 'identity': _identity(stat),
            'anchor_start': start, 'anchor_sha256': anchor_hash,
            'manifest_sha256': _hashes(metadata)['manifest.json'],
            'metadata_sha256': _hashes(metadata), 'last_frame': last['frame_id'],
            'last_time_s': last['sim_time_s'], 'wall_time_ns': time.time_ns()}


def snapshot_live(source, run, initial):
    """Atomically publish a bounded current-run CSV without stopping its writer."""
    source, run = Path(source).resolve(), Path(run).resolve()
    if initial.get('version') != 1 or initial.get('source') != str(source):
        raise ValueError('Preflight belongs to a different capture source or version.')
    target = run/'truth'
    if target.exists() or target.is_symlink():
        raise ValueError('Truth evidence already exists; refusing to overwrite it.')
    metadata = _metadata(source)
    if _hashes(metadata) != initial['metadata_sha256']:
        raise ValueError('Capture manifest or session metadata changed since preflight.')
    file = source/'frames.csv'
    _regular(file)
    # Fresh reads are essential: buffered seeks could hide an in-place rewrite.
    with file.open('rb', buffering=0) as stream:
        stat = os.fstat(stream.fileno())
        header, fields, time_field = _header(stream)
        start = initial['start_byte']
        if not len(header) <= initial['anchor_start'] <= start <= initial['frames_bytes']:
            raise ValueError('Invalid preflight byte boundaries.')

        def verify_source():
            _regular(file)
            current = file.stat()
            if (_identity(current) != initial['identity']
                    or _identity(os.fstat(stream.fileno())) != initial['identity']):
                raise ValueError('Capture file identity changed (replaced session).')
            if current.st_size < max(stat.st_size, initial['frames_bytes']):
                raise ValueError('Capture truncated since preflight or during snapshot.')
            if (_header(stream)[0].decode('utf-8') != initial['header']
                    or _range_hash(stream, initial['anchor_start'], start)
                    != initial['anchor_sha256']):
                raise ValueError('Capture preflight prefix was rewritten.')
            if _metadata(source) != metadata:
                raise ValueError('Capture manifest or session metadata changed during snapshot.')

        verify_source()
        if stat.st_size - start > MAX_WINDOW:
            raise ValueError('Live run window exceeds 256 MiB limit.')
        end = _complete_end(stream, stat.st_size)
        if end <= start:
            raise ValueError('No new complete rows since preflight.')
        total = len(header) + end - start + sum(map(len, metadata.values()))
        if shutil.disk_usage(run).free < 2 * total + 32 * 1024 * 1024:
            raise ValueError('Insufficient space for truth snapshot and final ZIP.')
        temporary = Path(tempfile.mkdtemp(prefix='truth-live-', suffix='.partial', dir=run))
        try:
            before_hash = _range_hash(stream, start, end)
            with (temporary/'frames.csv').open('wb') as output:
                output.write(header)
                copied_hash = _range_hash(stream, start, end, output)
            with (temporary/'frames.csv').open(newline='', encoding='utf-8-sig') as copied:
                rows = csv.reader(copied, strict=True)
                next(rows)
                previous = {'frame_id': initial['last_frame'], 'sim_time_s': initial['last_time_s']}
                first, count = None, 0
                for row in rows:
                    point = _point(row, fields, time_field)
                    if (point['frame_id'] <= previous['frame_id']
                            or point['sim_time_s'] <= previous['sim_time_s']):
                        raise ValueError('Non-monotonic frame/time or capture reset in run window.')
                    first = first or point
                    previous = point
                    count += 1
            if not count:
                raise ValueError('No new complete rows since preflight.')
            if before_hash != copied_hash or copied_hash != _range_hash(stream, start, end):
                raise ValueError('Copied capture prefix was rewritten during snapshot.')
            for name, data in metadata.items():
                (temporary/name).write_bytes(data)
            summary = {'stop_reason': 'runner_snapshot', 'rows': count,
                       'first_frame': first['frame_id'], 'last_frame': previous['frame_id'],
                       'first_time_s': first['sim_time_s'], 'last_time_s': previous['sim_time_s']}
            snapshot = {'source': str(source), 'initial': initial, 'start_byte': start,
                        'end_byte': end, 'observed_bytes': stat.st_size,
                        'omitted_partial_bytes': stat.st_size - end,
                        'window_sha256': copied_hash, 'metadata_sha256': _hashes(metadata),
                        'wall_time_ns': time.time_ns(), **summary}
            for name, data in (('summary.json', summary), ('snapshot.json', snapshot)):
                (temporary/name).write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
            verify_source()
            if _range_hash(stream, start, end) != copied_hash:
                raise ValueError('Copied capture prefix was rewritten during snapshot.')
            if target.exists() or target.is_symlink():
                raise ValueError('Truth evidence already exists; refusing to overwrite it.')
            temporary.rename(target)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
    return snapshot


def active_session(source, interval=3):
    if not source.is_dir():
        raise ValueError(f'Capture output folder does not exist: {source}. Start TruthCapture first.')
    candidates = []
    if (source/'frames.csv').is_file():
        candidates.append(source)
    else:
        for count, child in enumerate(source.iterdir()):
            if count >= 2000:
                raise ValueError('Too many output entries. Supply the exact current session folder.')
            if not child.is_symlink() and child.is_dir() and (child/'frames.csv').is_file():
                candidates.append(child)
    candidates = [p for p in candidates if not (p/'frames.csv').is_symlink()]
    if not candidates:
        raise ValueError('No frames.csv in this folder or its immediate session folders. Start recording; do not select the plugin install folder.')
    before = {p: (p/'frames.csv').stat().st_size for p in candidates}
    print(f'Checking {len(candidates)} capture session(s) for live CSV growth ({interval}s)...',
          file=sys.stderr, flush=True)
    time.sleep(interval)
    growing = [p for p in candidates if (p/'frames.csv').stat().st_size > before[p]]
    if not growing:
        raise ValueError('No actively growing recording detected. Start TruthCapture, unpause X-Plane, wait a few seconds, then retry. Old sessions were not selected.')
    if len(growing) > 1:
        print('Several recordings are growing. Select this test recording:', file=sys.stderr)
        for i, candidate in enumerate(growing, 1):
            print(f'{i}. {candidate}', file=sys.stderr)
        print('Session number: ', end='', file=sys.stderr, flush=True)
        choice = input().strip()
        if not choice.isdigit() or not 1 <= int(choice) <= len(growing):
            raise ValueError('Invalid session selection.')
        selected = growing[int(choice)-1]
    else:
        selected = growing[0]
    with (selected/'frames.csv').open(newline='', encoding='utf-8-sig') as stream:
        rows = csv.reader(stream)
        header = next(rows, [])
        first = next(rows, [])
    if 'frame_id' not in header or not any(k in header for k in
            ('sim/time/total_flight_time_sec', 'sim_time_s')) or not first:
        raise ValueError('Growing CSV is missing frame/time fields or data rows; check TruthCapture configuration.')
    print('LIVE TruthCapture selected: '+str(selected.resolve()), file=sys.stderr, flush=True)
    return selected.resolve()


def files(source):
    if not source.is_dir() or not (source/'frames.csv').is_file() or (source/'frames.csv').is_symlink():
        raise ValueError('Select the current TruthCapture session folder containing frames.csv, not the plugin folder.')
    result = sorted(p for p in source.iterdir() if p.is_file() and not p.is_symlink()
                    and p.suffix.lower() in ('.csv', '.json', '.ini', '.txt'))
    if not result:
        raise ValueError('No regular capture files found.')
    return result


def stamps(paths):
    return {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in paths}


def attach(source, run, interval=2):
    selected = files(source)
    before = stamps(selected)
    time.sleep(interval)
    if stamps(files(source)) != before:
        raise ValueError('TruthCapture is still changing. Stop recording, then attach again; no capture copied.')
    total = sum(s[0] for s in before.values())
    preflight = run/'truth-preflight.json'
    if preflight.is_file():
        initial = json.loads(preflight.read_text())
        if before['frames.csv'][0] <= initial['frames_bytes']:
            raise ValueError('Capture has not grown since preflight; it did not record this flight. Originals retained.')
    if shutil.disk_usage(run).free < 2*total + 32*1024*1024:
        raise ValueError('Insufficient space for truth copy and final ZIP. Capture remains at the source path.')
    target = run/'truth'
    if target.exists():
        raise ValueError('Truth evidence already exists; refusing to overwrite it.')
    temporary = run/'truth-copy.partial'
    temporary.mkdir()
    try:
        for p in selected:
            shutil.copy2(p, temporary/p.name)
        if stamps(files(source)) != before or stamps(list(temporary.iterdir())) != before:
            raise ValueError('Capture changed during copy; temporary copy rejected.')
        temporary.rename(target)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    (run/'truth-attachment.json').write_text(json.dumps(
        {'source': str(source), 'files': before, 'copied_bytes': total,
         'note': 'Stable copy is not proof that capture covers this flight; verify timestamps on review.'}, indent=2)+'\n')
    print('Attached stable TruthCapture evidence:', target)


if __name__ == '__main__':
    try:
        mode, raw = sys.argv[1:3]
        source = path(raw).resolve()
        if mode == 'resolve':
            print(source)
        elif mode == 'live':
            print(active_session(source))
        elif mode == 'snapshot':
            files(source)
            Path(sys.argv[3]).write_text(json.dumps({'source': str(source),
                'frames_bytes': (source/'frames.csv').stat().st_size,
                'wall_time_ns': time.time_ns()}, indent=2)+'\n')
        elif mode == 'begin-snapshot':
            destination = Path(sys.argv[3])
            initial = begin_snapshot(source)
            with destination.open('x', encoding='utf-8') as output:
                output.write(json.dumps(initial, indent=2)+'\n')
        elif mode == 'snapshot-live':
            initial = json.loads(Path(sys.argv[4]).read_text(encoding='utf-8'))
            snapshot_live(source, Path(sys.argv[3]), initial)
        elif mode == 'check':
            print('Capture folder:', source, '| files:', len(files(source)))
        elif mode == 'attach':
            attach(source, Path(sys.argv[3]).resolve())
        else:
            raise ValueError('Usage: attach_truth.py resolve|live|check|snapshot|attach|begin-snapshot PATH [RUN_OR_JSON]; snapshot-live PATH RUN INITIAL_JSON')
    except (OSError, ValueError, csv.Error) as error:
        raise SystemExit(str(error))
