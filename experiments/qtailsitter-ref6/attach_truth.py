#!/usr/bin/env python3
"""Capture stable, top-level TruthCapture session files without scanning the drive."""
import json
import csv
import errno
import gzip
import hashlib
import io
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

from locate_inputs import path


MAX_WINDOW = 256 * 1024 * 1024
MAX_INPUT_WINDOW = 1024 * 1024 * 1024
MAX_COMPRESSED = 128 * 1024 * 1024
MAX_ROW_BYTES = 1024 * 1024
SPACE_HEADROOM = 32 * 1024 * 1024
TAIL_BYTES = 1024 * 1024
MAX_METADATA = 8 * 1024 * 1024
SNAPSHOT_FILES = ('manifest.json', 'datarefs.json', 'datarefs.csv', 'datarefs.txt',
                  'capture_config.ini', 'capture_config.json')

# Laminar's published catalog defines moments in Nm and inertia per kg in m^2.
# These are recorded only: neither this helper nor TruthCapture writes physics.
MOMENT_REFS = tuple('sim/flightmodel/forces/'+axis+'_'+kind
                    for kind in ('total', 'prop', 'aero') for axis in ('L', 'M', 'N'))
INERTIA_REFS = tuple('sim/aircraft/weight/acf_J'+axis+axis+'_unitmass' for axis in 'xyz')
ENERGY_REFS = ('sim/cockpit/electrical/battery_charge_watt_hr',
               'sim/aircraft/electrical/battery_watt_hr_max',
               'sim/cockpit2/electrical/battery_voltage_actual_volts',
               'sim/cockpit2/electrical/battery_amps')
FORCE_REFS = tuple('sim/flightmodel/forces/'+axis+'_'+kind
                   for kind in ('total', 'prop', 'aero') for axis in ('faxil', 'fside', 'fnrml'))
DIAGNOSTIC_REFS = MOMENT_REFS + INERTIA_REFS + ENERGY_REFS + FORCE_REFS
TRUTH_BLOCK = ('\n# BEGIN REF6 READ-ONLY MOMENT AND ENERGY CAPTURE\n' +
               ''.join(ref+'|ref6_dynamics|false\n' for ref in DIAGNOSTIC_REFS) +
               '# END REF6 READ-ONLY MOMENT AND ENERGY CAPTURE\n').encode()


def configure_diagnostics(state, restore=False):
    """Receipted, reversible append to the recorder's custom catalog only."""
    from ref6 import atomic, require
    journal_file = state/'truth-instrumentation.json'
    require(not journal_file.is_symlink(), 'Recorder instrumentation journal must not be a symlink')
    journal = json.loads(journal_file.read_text()) if journal_file.exists() else None
    if not restore:
        receipt = json.loads((state/'receipt.json').read_text())
        require(not receipt.get('restored'), 'Ref6 installation has been restored')
    if journal is None:
        require(not restore, 'No recorder instrumentation to restore')
        root = Path(receipt['aircraft']).parents[1]
        target = root/'Resources/plugins/XPlaneTruthCapture/config/datarefs.txt'
        require(target.parent.is_dir() and not target.is_symlink(),
                'Missing recorder config directory or symlink target: '+str(target))
        before = target.read_bytes() if target.exists() else b''
        require(len(before) < 256*1024, 'Recorder custom catalog exceeds 256 KiB')
        require(b'# BEGIN REF6 READ-ONLY' not in before,
                'Unreceipted Ref6 capture block found; refusing to overwrite')
        journal = dict(version=1, target=str(target), original=before.hex(),
                       originally_present=target.exists(), installed_sha256=hashlib.sha256(before+TRUTH_BLOCK).hexdigest())
        atomic(journal_file, (json.dumps(journal, indent=2)+'\n').encode())
    require(journal.get('version') == 1, 'Unsupported recorder instrumentation receipt')
    target = Path(journal['target'])
    require(target.is_absolute() and target.parent.resolve() == target.parent and not target.is_symlink(),
            'Recorder target must be an absolute regular path')
    before = bytes.fromhex(journal['original'])
    current = target.read_bytes() if target.exists() else b''
    expected = before+TRUTH_BLOCK
    require(hashlib.sha256(expected).hexdigest() == journal['installed_sha256'],
            'Recorder instrumentation receipt does not match this catalog')
    require(current in (before, expected), 'Recorder custom catalog was edited; nothing overwritten')
    if restore:
        if journal['originally_present']:
            atomic(target, before)
        else:
            target.unlink(missing_ok=True)
        journal_file.unlink()
        print('Restored recorder custom catalog:', target)
    else:
        if current != expected:
            atomic(target, expected)
        print('Read-only moment/inertia/battery capture configured:', target)
        print('Stop any current TruthCapture recording, then Start Capture again. No plugin/aircraft replacement.')
    return target


def diagnostic_availability(source):
    with (source/'datarefs.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = {row['path']: row for row in csv.DictReader(stream)}
    missing = [ref for ref in DIAGNOSTIC_REFS
               if ref not in rows or rows[ref].get('exists', '').lower() != 'true']
    return {'requested': list(DIAGNOSTIC_REFS), 'missing': missing,
            'note': 'Availability only; units/signs/physical validity require post-flight review.'}


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


def _zip_budget(run):
    # Allow for deflate expansion, ZIP headers, and SHA256SUMS entries.
    total = 0
    for file in run.rglob('*'):
        if file.is_file() and not file.is_symlink():
            size = file.stat().st_size
            total += size + size // 1000 + 1024 + 4 * len(str(file.relative_to(run)).encode())
    return total


def _require_space(run, needed, detail):
    free = shutil.disk_usage(run).free
    if free < needed:
        raise ValueError(f'Insufficient space for truth snapshot and final ZIP: need '
                         f'{needed} bytes ({needed / 1024**2:.1f} MiB), have '
                         f'{free} bytes ({free / 1024**2:.1f} MiB); {detail}.')


class _BoundedOutput:
    def __init__(self, output, run, limit, needed, detail):
        self.output, self.run, self.limit = output, run, limit
        self.needed, self.detail, self.size = needed, detail, 0

    def write(self, data):
        if self.size + len(data) > self.limit:
            raise ValueError(f'Compressed truth exceeds {self.limit} byte '
                             f'({self.limit / 1024**2:g} MiB) limit; '
                             'no evidence published, no rows truncated.')
        _require_space(self.run, self.needed, self.detail)
        written = self.output.write(data)
        self.size += written
        self.needed -= written
        return written


def _validate_copy(stream, header, fields, time_field, initial, window_bytes,
                   max_row_bytes=None):
    if stream.readline(TAIL_BYTES + 1) != header:
        raise ValueError('Copied CSV header was rewritten.')
    raw_digest, window_digest = hashlib.sha256(header), hashlib.sha256()
    remaining, row_bytes = window_bytes, 0
    row_limit = window_bytes if max_row_bytes is None else max_row_bytes

    def lines():
        nonlocal remaining, row_bytes
        # Preserve CSV newline handling while hashing the exact UTF-8 bytes.
        with io.TextIOWrapper(stream, encoding='utf-8', newline='') as text:
            while True:
                line = text.readline(min(row_limit + 1, remaining + 1))
                if not line:
                    break
                data = line.encode('utf-8')
                remaining -= len(data)
                row_bytes += len(data)
                if remaining < 0:
                    raise ValueError('Copied CSV has unexpected raw byte length.')
                if row_bytes > row_limit:
                    raise ValueError('CSV row exceeds bounded parsing limit; no rows truncated.')
                raw_digest.update(data)
                window_digest.update(data)
                yield line

    previous = {'frame_id': initial['last_frame'], 'sim_time_s': initial['last_time_s']}
    first, count = None, 0
    for row in csv.reader(lines(), strict=True):
        row_bytes = 0
        point = _point(row, fields, time_field)
        if (point['frame_id'] <= previous['frame_id']
                or point['sim_time_s'] <= previous['sim_time_s']):
            raise ValueError('Non-monotonic frame/time or capture reset in run window.')
        first = first or point
        previous = point
        count += 1
    if remaining:
        raise ValueError('Copied CSV truncated during snapshot.')
    if not count:
        raise ValueError('No new complete rows since preflight.')
    summary = {'stop_reason': 'runner_snapshot', 'rows': count,
               'first_frame': first['frame_id'], 'last_frame': previous['frame_id'],
               'first_time_s': first['sim_time_s'], 'last_time_s': previous['sim_time_s']}
    return summary, window_digest.hexdigest(), raw_digest.hexdigest()


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
        if stat.st_size - start > MAX_INPUT_WINDOW:
            raise ValueError(f'Live run window exceeds bounded input limit of '
                             f'{MAX_INPUT_WINDOW} bytes ({MAX_INPUT_WINDOW / 1024**2:g} MiB); '
                             'no evidence published, no rows truncated.')
        end = _complete_end(stream, stat.st_size)
        if end <= start:
            raise ValueError('No new complete rows since preflight.')
        compressed = end - start > MAX_WINDOW
        frames_name = 'frames.csv.gz' if compressed else 'frames.csv'
        raw_bytes = len(header) + end - start
        frames_budget = MAX_COMPRESSED if compressed else raw_bytes
        # snapshot.json embeds the preflight header; allow for JSON escaping.
        total = frames_budget + sum(map(len, metadata.values())) + 8 * len(header) + 64 * 1024
        zip_budget = _zip_budget(run) + total + total // 1000 + 64 * 1024
        needed = total + zip_budget + SPACE_HEADROOM
        detail = (f'{frames_budget / 1024**2:g} MiB '
                  f'{"compressed target cap" if compressed else "raw CSV"}, '
                  f'{zip_budget / 1024**2:.1f} MiB final ZIP budget including existing run '
                  f'evidence, {SPACE_HEADROOM / 1024**2:g} MiB free-space headroom')
        _require_space(run, needed, detail)
        temporary = Path(tempfile.mkdtemp(prefix='truth-live-', suffix='.partial', dir=run))
        try:
            before_hash = _range_hash(stream, start, end)
            artifact = temporary/frames_name
            with artifact.open('wb') as output:
                if compressed:
                    bounded = _BoundedOutput(output, run, MAX_COMPRESSED, needed, detail)
                    with gzip.GzipFile(filename='', mode='wb', fileobj=bounded,
                                       compresslevel=6, mtime=0) as packed:
                        packed.write(header)
                        copied_hash = _range_hash(stream, start, end, packed)
                else:
                    output.write(header)
                    copied_hash = _range_hash(stream, start, end, output)
            opener = gzip.open if compressed else open
            with opener(artifact, 'rb') as copied:
                summary, verified_hash, raw_hash = _validate_copy(
                    copied, header, fields, time_field, initial, end - start,
                    max_row_bytes=MAX_ROW_BYTES if compressed else None)
            if verified_hash != copied_hash:
                raise ValueError('Copied capture prefix was rewritten during snapshot.')
            stored_bytes = artifact.stat().st_size
            with artifact.open('rb') as stored:
                stored_hash = _range_hash(stored, 0, stored_bytes)
            if before_hash != copied_hash or copied_hash != _range_hash(stream, start, end):
                raise ValueError('Copied capture prefix was rewritten during snapshot.')
            for name, data in metadata.items():
                (temporary/name).write_bytes(data)
            snapshot = {'source': str(source), 'initial': initial, 'start_byte': start,
                        'end_byte': end, 'observed_bytes': stat.st_size,
                        'omitted_partial_bytes': stat.st_size - end,
                        'window_sha256': copied_hash, 'metadata_sha256': _hashes(metadata),
                        'frames_file': frames_name,
                        'frames_compression': 'gzip' if compressed else 'none',
                        'frames_bytes': stored_bytes, 'frames_sha256': stored_hash,
                        'frames_raw_bytes': raw_bytes, 'frames_raw_sha256': raw_hash,
                        'wall_time_ns': time.time_ns(), **summary}
            for name, data in (('summary.json', summary), ('snapshot.json', snapshot)):
                (temporary/name).write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
            verify_source()
            if _range_hash(stream, start, end) != copied_hash:
                raise ValueError('Copied capture prefix was rewritten during snapshot.')
            _require_space(run, zip_budget + SPACE_HEADROOM, detail)
            if target.exists() or target.is_symlink():
                raise ValueError('Truth evidence already exists; refusing to overwrite it.')
            temporary.rename(target)
        except OSError as error:
            if error.errno == errno.ENOSPC:
                raise ValueError(f'Insufficient space for truth snapshot and final ZIP; '
                                 f'{detail}. No evidence published.') from error
            raise
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
