#!/usr/bin/env python3
"""Capture stable, top-level TruthCapture session files without scanning the drive."""
import json
import csv
from pathlib import Path
import shutil
import sys
import time

from locate_inputs import path


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
        elif mode == 'check':
            print('Capture folder:', source, '| files:', len(files(source)))
        elif mode == 'attach':
            attach(source, Path(sys.argv[3]).resolve())
        else:
            raise ValueError('Usage: attach_truth.py resolve|live|check|snapshot|attach PATH [RUN_OR_JSON]')
    except (OSError, ValueError) as error:
        raise SystemExit(str(error))
