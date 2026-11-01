#!/usr/bin/env python3
"""Finalize a small manifest without duplicating ULogs."""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import zipfile


def collect(run):
    logs = list((run/'rootfs').rglob('*.ulg'))
    status = {'ulog_count': len(logs), 'ulog_bytes': sum(p.stat().st_size for p in logs),
              'truth_requested': (run/'truth-source.txt').exists(),
              'truth_attached': (run/'truth/frames.csv').is_file(),
              'note': 'Flight success and GCS reception require ULog review; process exit is not a pass.'}
    (run/'collection.json').write_text(json.dumps(status, indent=2)+'\n')
    paths = sorted(p for p in run.rglob('*') if p.is_file() and not p.is_symlink() and p.name != 'SHA256SUMS')
    before = {p: (p.stat().st_size, p.stat().st_mtime_ns) for p in paths}
    with (run/'SHA256SUMS').open('w') as output:
        for path in paths:
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            output.write(f'{digest.hexdigest()}  {path.relative_to(run)}\n')
    archive = run.parent / (run.name+'-'+str(time.time_ns())+'.zip')
    fd, temporary = tempfile.mkstemp(prefix='.collect-', suffix='.partial', dir=run.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=3) as z:
                for path in paths + [run/'SHA256SUMS']:
                    z.write(path, str(Path(run.name)/path.relative_to(run)))
        if any((p.stat().st_size, p.stat().st_mtime_ns) != stamp for p, stamp in before.items()):
            raise RuntimeError('Evidence changed during collection. Stop PX4 and retry; no ZIP published.')
        os.link(temporary, archive)  # Refuse to overwrite even on a name collision.
    finally:
        Path(temporary).unlink(missing_ok=True)
    print('Local archive (not uploaded):', archive)
    if not logs:
        print('WARNING: no ULog was recorded; this is not a usable flight result.')
    if status['truth_requested'] and not status['truth_attached']:
        print('WARNING: requested TruthCapture evidence is MISSING. ZIP is incomplete for actuator-response review.')


if __name__ == '__main__':
    collect(Path(sys.argv[1]).resolve())
