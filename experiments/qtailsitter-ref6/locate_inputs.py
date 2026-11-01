#!/usr/bin/env python3
"""Resolve actual installed inputs; never silently choose ambiguous matches."""
import json
from pathlib import Path
import re
import subprocess
import sys


def path(value):
    value = value.strip().strip('"').strip("'")
    if re.match(r'^[A-Za-z]:[\\/]', value):
        value = subprocess.check_output(['wslpath', '-u', value], text=True).strip()
    return Path(value).expanduser()


def choose(label, candidates):
    candidates = sorted(set(p.resolve() for p in candidates if p.is_file()))
    if len(candidates) == 1:
        selected = candidates[0]
        print(f'{label}: {selected}', file=sys.stderr)
        return str(selected)
    if candidates:
        print(f'Multiple {label} matches:', file=sys.stderr)
        for i, p in enumerate(candidates, 1):
            print(f'  {i}. {p}', file=sys.stderr)
        answer = input('Select number, or enter full file path: ').strip()
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            return str(candidates[int(answer)-1])
    else:
        answer = input(f'{label} not found; enter its full file path: ')
    selected = path(answer)
    if not selected.is_file():
        raise ValueError(f'File does not exist: {selected}')
    return str(selected.resolve())


def locate(root):
    if not root.is_dir():
        raise ValueError(f'X-Plane folder is not accessible: {root}. Check the Windows drive is mounted in WSL.')
    print('Checking plugin locations (no recursive drive scan)...', file=sys.stderr, flush=True)
    plugins = root/'Resources/plugins'
    configs = []
    if plugins.is_dir():
        for folder in plugins.iterdir():
            if 'px4' in folder.name.lower() and folder.is_dir():
                configs += [folder/'config.ini', folder/'64/config.ini']
    config = choose('Active PX4 plugin config.ini', configs)
    print('Checking common aircraft locations (at most 200 containers)...', file=sys.stderr, flush=True)
    base = root/'Aircraft'
    aircraft = [base/'QuadTailsitter/QuadTailsitter.acf']
    if base.is_dir():
        for count, folder in enumerate(base.iterdir()):
            if count >= 200:
                print('Many Aircraft entries: stopping discovery. Enter the loaded path if not listed.',
                      file=sys.stderr, flush=True)
                break
            if folder.is_dir():
                aircraft += [folder/'QuadTailsitter.acf',
                             folder/'QuadTailsitter/QuadTailsitter.acf']
    return {'config': config,
            'acf': choose('QuadTailsitter aircraft', aircraft),
            'log': choose('X-Plane Log.txt', [root/'Log.txt'])}


if __name__ == '__main__':
    try:
        result = locate(path(sys.argv[1]))
        Path(sys.argv[2]).write_text(json.dumps(result))
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
