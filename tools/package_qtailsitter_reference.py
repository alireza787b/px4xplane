#!/usr/bin/env python3
"""Package the experimental Ref6 installer, model and optional offline review."""

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import zipfile

from build_qtailsitter_reference import build

ROOT = Path(__file__).resolve().parents[1]
BUNDLE_NAME = 'PX4_QTAILSITTER_REF6_2026-11-01'


def package(output_root, archive_dir, review_dir=None):
    output = output_root / BUNDLE_NAME
    archive = archive_dir / (BUNDLE_NAME + '.zip')
    if output.exists() or archive.exists():
        raise FileExistsError('Refusing existing bundle/archive; use fresh output locations')
    output.mkdir(parents=True)
    archive_dir.mkdir(parents=True, exist_ok=True)
    support = ROOT / 'experiments/qtailsitter-ref6'
    for path in sorted(support.iterdir()):
        if path.is_file() and not path.name.startswith('.'):
            shutil.copy2(path, output / path.name)
    document = build(ROOT / 'aircraft/QuadTailsitter/QuadTailsitter.acf', output)
    (output / 'VERSION.txt').write_text(document['revision'] + '\n')
    tools = output / 'build_tools'
    tools.mkdir()
    for name in ('build_qtailsitter_reference.py', 'analyze_qtailsitter_design.py'):
        shutil.copy2(ROOT / 'tools' / name, tools / name)
    commit = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    (output / 'SOURCE.md').write_text(
        '# Source Identity\n\n'
        f'px4xplane source checkout commit: `{commit}`.\n\n'
        'The per-file SHA256SUMS records the delivered scripts, including any local\n'
        'changes. Do not infer a clean working tree from the commit alone.\n\n'
        'PX4 baseline: `f90b40d61e56405513dfcd8e194287ac60d3a6ae`.\n'
        'Diagnostic-only build: `9cfe130a8d8e0c25f88e044141c523247ca53d48`.\n'
        'No private dive feature, optional control fixes or hardware firmware included.\n\n'
        'The generated model has NOT flown on this server. Geometry and BEM\n'
        'checks are not X-Plane calibration. Read QUICKSTART.md before testing.\n')
    if review_dir:
        shutil.copy2(review_dir / 'REPORT.md', output / 'REPORT.md')
        evidence = review_dir / 'evidence'
        if any(path.is_symlink() for path in evidence.rglob('*')):
            raise ValueError('Symlinks in review evidence refused')
        shutil.copytree(evidence, output / 'evidence', ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    files = sorted(path for path in output.rglob('*') if path.is_file())
    (output / 'SHA256SUMS').write_text(''.join(
        f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(output)}\n'
        for path in files))
    try:
        with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zipped:
            for path in sorted(output.rglob('*')):
                if path.is_file():
                    zipped.write(path, str(Path(BUNDLE_NAME) / path.relative_to(output)))
    except BaseException:
        archive.unlink(missing_ok=True)
        raise
    checksum = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix('.zip.sha256').write_text(f'{checksum}  {archive.name}\n')
    return output, archive, checksum


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--archive-dir', type=Path, required=True)
    parser.add_argument('--review-dir', type=Path)
    args = parser.parse_args()
    folder, archive, digest = package(args.output_root, args.archive_dir, args.review_dir)
    print(f'Bundle: {folder}\nArchive: {archive}\nSHA256: {digest}')
