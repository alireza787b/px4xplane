#!/usr/bin/env python3
"""Save and restore Git identities without resetting source or moving branches."""
import json
from pathlib import Path
import subprocess
import sys


def git(repo, *args, check=True):
    return subprocess.run(['git', '-C', str(repo), *args], check=check,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def identity(repo):
    ref = git(repo, 'symbolic-ref', '--quiet', '--short', 'HEAD', check=False)
    return {'commit': git(repo, 'rev-parse', 'HEAD').stdout.strip(),
            'branch': ref.stdout.strip() if ref.returncode == 0 else ''}


def initialized(repo):
    result = git(repo, 'submodule', 'foreach', '--quiet', '--recursive',
                 'printf "%s\\0" "$displaypath"')
    return [path for path in result.stdout.split('\0') if path]


def save(repo, file):
    snapshot = {'main': identity(repo), 'submodules': {}}
    for relative in initialized(repo):
        snapshot['submodules'][relative] = identity(repo/relative)
    file.write_text(json.dumps(snapshot, indent=2)+'\n')


def restore_one(repo, original):
    if git(repo, 'status', '--porcelain', '--ignore-submodules').stdout:
        raise RuntimeError(f'{repo} changed during the test; restore stopped without discarding changes.')
    if original['branch']:
        head = git(repo, 'rev-parse', 'refs/heads/'+original['branch']).stdout.strip()
        if head != original['commit']:
            raise RuntimeError(f'Original branch moved: {repo}; refusing to rewind it.')
        git(repo, 'switch', original['branch'])
    else:
        git(repo, 'switch', '--detach', original['commit'])


def restore(repo, file):
    snapshot = json.loads(file.read_text())
    modules = initialized(repo)
    added = [relative for relative in modules if relative not in snapshot['submodules']]
    # Remove only clean worktrees initialized by this test, retaining Git caches.
    # A module added by newer PX4 would otherwise leave the old checkout untracked.
    for relative in sorted(added, key=lambda p: p.count('/')):
        if any(relative.startswith(parent+'/') for parent in added if parent != relative):
            continue
        parents = [p for p in modules if relative.startswith(p+'/')]
        parent = max(parents, key=len) if parents else ''
        owner = repo/parent if parent else repo
        path = relative[len(parent)+1:] if parent else relative
        git(owner, 'submodule', 'deinit', '--', path)
    restore_one(repo, snapshot['main'])
    for relative, original in snapshot['submodules'].items():
        restore_one(repo/relative, original)
    print('Restored source and previously initialized submodule identities.')


if __name__ == '__main__':
    mode, directory, snapshot = sys.argv[1:]
    try:
        {'save': save, 'restore': restore}[mode](Path(directory).resolve(), Path(snapshot))
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
