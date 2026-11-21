"""Exercise checkout preservation against tiny local Git fixtures, never PX4/network."""
import os
import pty
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE=(Path(__file__).resolve().parents[1]/'setup/setup_px4_sitl.sh').read_text()


def function(name):
    return name+'() {'+SOURCE.split(name+'() {',1)[1].split('\n}\n',1)[0]+'\n}\n'


class LauncherCheckoutSafety(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve()
        self.repo=self.root/'checkout'
        self.repo.mkdir()
        self.remote=self.root/'remote.git'
        self.env={**os.environ,'GIT_CONFIG_NOSYSTEM':'1','GIT_CONFIG_GLOBAL':os.devnull,
                  'GIT_AUTHOR_NAME':'Offline test','GIT_AUTHOR_EMAIL':'test@example.invalid',
                  'GIT_COMMITTER_NAME':'Offline test','GIT_COMMITTER_EMAIL':'test@example.invalid',
                  'GIT_ALLOW_PROTOCOL':'file'}
        self.git('init','-b','main')
        (self.repo/'tracked').write_text('initial\n')
        self.git('add','tracked')
        self.git('commit','-m','initial')
        subprocess.run(['git','init','--bare',str(self.remote)],env=self.env,check=True,capture_output=True)
        self.git('remote','add','origin',str(self.remote))
        self.git('push','origin','main')
        self.original=self.git('rev-parse','HEAD')

    def git(self,*args):
        return subprocess.check_output(['git','-C',str(self.repo),'-c','commit.gpgsign=false',*args],
                                       env=self.env,stderr=subprocess.DEVNULL,text=True).strip()

    def invoke(self,remote=None,git_wrapper='',answer=None,input_text=''):
        script='''
warning() { printf '%s\\n' "$*"; }
info() { printf '%s\\n' "$*"; }
success() { printf '%s\\n' "$*"; }
progress() { :; }
fetch_guard_ref() { return 0; }
apply_guard_commits() { return 0; }
require_no_running_px4() { return 0; }
CLONE_PATH="$1"
REPO_URL="$2"
PX4_REMOTE_NAME=px4xplane
BRANCH_NAME=main
APPLY_TECS_ALT_FRAME_GUARD=false
'''+function('confirm')+function('require_clean_px4_checkout')+function('update_submodules_safely')+function('sync_px4_repo_to_remote')+git_wrapper+'\nsync_px4_repo_to_remote\n'
        command = ['/bin/bash','-c',script,'launcher-test',str(self.repo),str(remote or self.remote)]
        if answer is None:
            return subprocess.run(command, env=self.env,input=input_text,capture_output=True,text=True,timeout=30)
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen(command, env=self.env, stdin=slave, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            os.close(slave)
            slave = None
            os.write(master, (answer+'\n').encode())
            stdout, stderr = process.communicate(timeout=30)
            return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
        finally:
            os.close(master)
            if slave is not None:
                os.close(slave)

    def test_sync_detaches_without_moving_local_branches_or_origin(self):
        (self.repo/'tracked').write_text('my local commit\n')
        self.git('commit','-am','local work')
        local_commit=self.git('rev-parse','HEAD')
        result=self.invoke()
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.git('rev-parse','HEAD'),self.original)
        self.assertEqual(self.git('rev-parse','main'),local_commit)
        self.assertEqual(self.git('remote','get-url','origin'),str(self.remote))
        branch=subprocess.run(['git','-C',str(self.repo),'symbolic-ref','-q','HEAD'],capture_output=True)
        self.assertEqual(branch.returncode,1)

    def test_dirty_tracked_file_is_preserved_and_sync_stops(self):
        (self.repo/'tracked').write_text('unsaved edit\n')
        result=self.invoke()
        self.assertNotEqual(result.returncode,0)
        self.assertEqual((self.repo/'tracked').read_text(),'unsaved edit\n')
        self.assertEqual(self.git('rev-parse','HEAD'),self.original)
        self.assertNotIn('px4xplane',self.git('remote').splitlines())

    def test_untracked_file_is_not_cleaned_or_stashed(self):
        (self.repo/'my-work').write_text('keep')
        result=self.invoke()
        self.assertNotEqual(result.returncode,0)
        self.assertEqual((self.repo/'my-work').read_text(),'keep')
        self.assertEqual(self.git('stash','list'),'')

    def test_hidden_nested_submodule_changes_stop_sync(self):
        module=self.root/'module-source'
        module.mkdir()
        subprocess.run(['git','init','-b','main',str(module)],env=self.env,check=True,capture_output=True)
        (module/'file').write_text('module')
        subprocess.run(['git','-C',str(module),'add','file'],env=self.env,check=True,capture_output=True)
        subprocess.run(['git','-C',str(module),'-c','commit.gpgsign=false','commit','-m','module'],
                       env=self.env,check=True,capture_output=True)
        self.git('submodule','add',str(module),'module')
        self.git('commit','-am','module')
        self.git('config','submodule.module.ignore','all')
        hidden=self.repo/'module/untracked-work'
        hidden.write_text('preserve nested work')
        self.assertEqual(self.git('status','--porcelain'),'')
        result=self.invoke()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('submodule has local changes',result.stdout)
        self.assertEqual(hidden.read_text(),'preserve nested work')

    def test_failed_fetch_does_not_move_checkout(self):
        result=self.invoke(remote=self.root/'absent.git')
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.git('rev-parse','HEAD'),self.original)
        self.assertEqual(self.git('remote','get-url','origin'),str(self.remote))

    def test_detached_local_commit_gets_a_durable_recovery_ref(self):
        self.git('checkout', '--detach')
        (self.repo/'tracked').write_text('detached work\n')
        self.git('commit', '-am', 'detached local work')
        previous = self.git('rev-parse', 'HEAD')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(self.git('rev-parse', 'HEAD'), self.original)
        self.assertEqual(self.git('rev-parse', f'refs/px4xplane/previous/{previous}'), previous)

    def test_linked_worktree_is_reused_without_moving_its_branch(self):
        worktree = self.root/'linked checkout'
        self.git('worktree', 'add', '-b', 'local-worktree', str(worktree))
        self.repo = worktree
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertTrue((worktree/'.git').is_file())
        self.assertEqual(self.git('rev-parse', 'local-worktree'), self.original)
        self.assertEqual(self.git('remote', 'get-url', 'origin'), str(self.remote))

    def test_ignored_file_collision_does_not_overwrite_local_work(self):
        (self.repo/'.gitignore').write_text('ignored-work\n')
        self.git('add', '.gitignore')
        self.git('commit', '-m', 'ignore local work')
        previous = self.git('rev-parse', 'HEAD')
        (self.repo/'ignored-work').write_text('upstream content')
        self.git('add', '-f', 'ignored-work')
        self.git('commit', '-m', 'track previously ignored path')
        self.git('push', 'origin', 'main')
        self.git('checkout', '--detach', previous)
        (self.repo/'ignored-work').write_text('my ignored work')
        self.assertEqual(self.git('status', '--porcelain'), '')
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.repo/'ignored-work').read_text(), 'my ignored work')
        self.assertEqual(self.git('rev-parse', 'HEAD'), previous)

    def test_failed_submodule_sync_or_update_is_not_reported_as_success(self):
        for stage in ('sync', 'update'):
            wrapper = '''
git() {
    if [[ " $* " == *" submodule %s "* ]]; then return 23; fi
    command git "$@"
}
''' % stage
            result = self.invoke(git_wrapper=wrapper)
            self.assertNotEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertNotIn('PX4 checkout synced to', result.stdout)

    def test_clean_submodule_head_mismatch_is_preserved_before_alignment(self):
        module = self.root/'module-source'
        subprocess.run(['git', 'init', '-b', 'main', str(module)], env=self.env, check=True, capture_output=True)
        (module/'file').write_text('module')
        for args in (['add', 'file'], ['-c', 'commit.gpgsign=false', 'commit', '-m', 'module']):
            subprocess.run(['git', '-C', str(module), *args], env=self.env, check=True, capture_output=True)
        self.git('submodule', 'add', str(module), 'module')
        self.git('commit', '-am', 'module')
        self.git('push', 'origin', 'main')
        self.git('config', 'submodule.module.ignore', 'all')
        (self.repo/'module/file').write_text('local submodule commit')
        for args in (['add', 'file'], ['-c', 'commit.gpgsign=false', 'commit', '-m', 'local work']):
            subprocess.run(['git', '-C', str(self.repo/'module'), *args],
                           env=self.env, check=True, capture_output=True)
        previous = subprocess.check_output(['git', '-C', str(self.repo/'module'), 'rev-parse', 'HEAD'], text=True)
        self.assertEqual(self.git('status', '--porcelain'), '')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(subprocess.check_output(['git', '-C', str(self.repo/'module'), 'rev-parse',
                                                  'refs/px4xplane/previous/'+previous.strip()], text=True), previous)
        self.assertNotEqual(subprocess.check_output(['git', '-C', str(self.repo/'module'), 'rev-parse', 'HEAD'],
                                                   text=True), previous)

    def test_force_clean_stash_and_parameter_deletion_are_absent(self):
        self.assertNotIn('git reset --hard',SOURCE)
        self.assertNotIn('git clean -',SOURCE)
        self.assertNotIn('git stash pop',SOURCE)
        self.assertNotIn('git checkout -B',SOURCE)
        self.assertNotIn('reset_saved_sitl_parameters()',SOURCE)
        uninstall=SOURCE.split('if [ "$UNINSTALL_MODE" = true ]; then',1)[1].split('\nfi\n',1)[0]
        self.assertNotIn('rm -rf',uninstall)

    def test_decline_and_eof_leave_index_worktree_and_untracked_files_unchanged(self):
        (self.repo/'tracked').write_text('staged\n')
        self.git('add', 'tracked')
        (self.repo/'tracked').write_text('unstaged\n')
        (self.repo/'untracked').write_text('keep\n')
        before = self.git('status', '--porcelain')
        for answer in (None, 'n', ''):
            result = self.invoke(answer=answer)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(self.git('status', '--porcelain'), before)
            self.assertEqual(self.git('stash', 'list'), '')
            self.assertEqual(self.git('rev-parse', 'HEAD'), self.original)

    def test_explicit_yes_stashes_index_and_untracked_and_preserves_recovery(self):
        (self.repo/'tracked').write_text('staged\n')
        self.git('add', 'tracked')
        (self.repo/'tracked').write_text('unstaged\n')
        (self.repo/'untracked').write_text('keep\n')
        result = self.invoke(answer='yes')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        oid = self.git('rev-parse', 'refs/stash')
        self.assertEqual(self.git('rev-parse', 'refs/px4xplane/stashes/'+oid), oid)
        self.assertIn('stash apply --index '+oid, result.stdout)
        self.assertIn('NOT be automatically applied', result.stdout)
        self.assertEqual(self.git('status', '--porcelain'), '')
        self.git('stash', 'drop')
        self.git('stash', 'apply', '--index', 'refs/px4xplane/stashes/'+oid)
        self.assertEqual(self.git('show', ':tracked'), 'staged')
        self.assertEqual((self.repo/'tracked').read_text(), 'unstaged\n')
        self.assertEqual((self.repo/'untracked').read_text(), 'keep\n')

    def test_piped_yes_is_not_interactive_consent(self):
        (self.repo/'tracked').write_text('keep my edits')
        result = self.invoke(input_text='yes\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.git('stash', 'list'), '')
        self.assertEqual((self.repo/'tracked').read_text(), 'keep my edits')

    def test_interrupted_operations_are_not_stashed_or_switched(self):
        for name in ('MERGE_HEAD', 'CHERRY_PICK_HEAD', 'REVERT_HEAD', 'BISECT_START', 'rebase-merge', 'rebase-apply', 'sequencer'):
            marker = Path(self.git('rev-parse', '--git-path', name))
            if not marker.is_absolute(): marker = self.repo/marker
            marker.write_text(self.original+'\n')
            result = self.invoke(answer='y')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('interrupted Git operation', result.stdout)
            self.assertEqual(self.git('stash', 'list'), '')
            self.assertEqual(self.git('rev-parse', 'HEAD'), self.original)
            marker.unlink()

    def git_in(self, repo, *args):
        return subprocess.check_output(['git', '-C', str(repo), '-c', 'commit.gpgsign=false', *args],
                                       env=self.env, stderr=subprocess.DEVNULL, text=True).strip()

    def module_source(self, name):
        repo = self.root/name
        subprocess.run(['git', 'init', '-b', 'main', str(repo)],
                       env=self.env, check=True, capture_output=True)
        (repo/'file').write_text('initial module\n')
        (repo/'.gitignore').write_text('ignored-work\n')
        self.git_in(repo, 'add', '.')
        self.git_in(repo, 'commit', '-m', 'initial module')
        return repo

    def ignored_module_collision(self, nested=False, rename=False):
        leaf = self.module_source('leaf-source')
        parent = self.module_source('parent-source') if nested else leaf
        if nested:
            self.git_in(parent, 'submodule', 'add', '--name', 'old child', str(leaf), 'child dir')
            self.git_in(parent, 'commit', '-am', 'add child')
        self.git('submodule', 'add', '--name', 'parent module', str(parent), 'module')
        self.git('commit', '-am', 'add module')
        previous = self.git('rev-parse', 'HEAD')
        self.git('submodule', 'update', '--init', '--recursive')
        (leaf/'ignored-work').write_text('upstream tracked content\n')
        self.git_in(leaf, 'add', '-f', 'ignored-work')
        self.git_in(leaf, 'commit', '-m', 'track ignored path')
        if nested:
            child = parent/'child dir'
            self.git_in(child, 'fetch', 'origin')
            self.git_in(child, 'checkout', '--detach', self.git_in(leaf, 'rev-parse', 'HEAD'))
            if rename:
                self.git_in(parent, 'config', '--file', '.gitmodules', '--rename-section',
                            'submodule.old child', 'submodule.new child')
            self.git_in(parent, 'add', '.gitmodules', 'child dir')
            self.git_in(parent, 'commit', '-m', 'update child registration')
        module = self.repo/'module'
        self.git_in(module, 'fetch', 'origin')
        self.git_in(module, 'checkout', '--detach', self.git_in(parent, 'rev-parse', 'HEAD'))
        self.git('add', 'module')
        self.git('commit', '-m', 'update module')
        self.git('push', 'origin', 'main')
        self.git('checkout', '--detach', previous)
        self.git('submodule', 'update', '--init', '--recursive')
        local_leaf = module/'child dir' if nested else module
        previous_leaf = self.git_in(local_leaf, 'rev-parse', 'HEAD')
        protected = local_leaf/'ignored-work'
        protected.write_text('preserve ignored local content\n')
        self.git('config', 'submodule.recurse', 'true')
        self.git('config', 'submodule.parent module.update', 'checkout')
        self.assertEqual(self.git('status', '--porcelain'), '')
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('would be overwritten by checkout', result.stderr)
        self.assertNotIn('PX4 checkout synced to', result.stdout)
        self.assertEqual(protected.read_text(), 'preserve ignored local content\n')
        self.assertEqual(self.git_in(local_leaf, 'rev-parse', 'HEAD'), previous_leaf)
        self.assertEqual(self.git('config', 'submodule.parent module.update'), 'checkout')
        for repo in (self.repo, module, local_leaf):
            self.assertNotIn('!git', self.git_in(repo, 'config', '--local', '--list'))

    def test_ignored_submodule_collision_preserves_local_content(self):
        self.ignored_module_collision()

    def test_ignored_nested_submodule_collision_preserves_local_content(self):
        self.ignored_module_collision(nested=True)

    def test_renamed_nested_registration_retains_ignored_collision_protection(self):
        self.ignored_module_collision(nested=True, rename=True)

    def test_active_root_bisect_is_not_switched_or_stashed(self):
        for text in ('middle', 'latest'):
            (self.repo/'tracked').write_text(text+'\n')
            self.git('commit', '-am', text)
        latest = self.git('rev-parse', 'HEAD')
        self.git('push', 'origin', 'main')
        self.git('bisect', 'start', latest, self.original)
        candidate = self.git('rev-parse', 'HEAD')
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('BISECT_START', result.stdout)
        self.assertEqual(self.git('rev-parse', 'HEAD'), candidate)
        self.assertTrue((self.repo/'.git/BISECT_START').exists())
        self.assertEqual(self.git('stash', 'list'), '')
        self.assertNotIn('px4xplane', self.git('remote').splitlines())

    def test_active_submodule_bisect_is_not_switched_or_stashed(self):
        source = self.module_source('module-source')
        first = self.git_in(source, 'rev-parse', 'HEAD')
        for text in ('middle', 'latest'):
            (source/'file').write_text(text+'\n')
            self.git_in(source, 'commit', '-am', text)
        self.git('submodule', 'add', str(source), 'module')
        self.git('commit', '-am', 'add module')
        root_head = self.git('rev-parse', 'HEAD')
        module = self.repo/'module'
        self.git_in(module, 'bisect', 'start', 'HEAD', first)
        candidate = self.git_in(module, 'rev-parse', 'HEAD')
        (self.repo/'tracked').write_text('root edit must not be stashed\n')
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('interrupted operation', result.stdout)
        self.assertEqual(self.git_in(module, 'rev-parse', 'HEAD'), candidate)
        self.assertEqual(self.git('rev-parse', 'HEAD'), root_head)
        self.assertEqual((self.repo/'tracked').read_text(), 'root edit must not be stashed\n')
        self.assertEqual(self.git('stash', 'list'), '')
        self.assertNotIn('px4xplane', self.git('remote').splitlines())

    def test_nested_untracked_repository_is_not_claimed_by_root_stash(self):
        nested = self.repo/'new-directory/nested'
        subprocess.run(['git', 'init', str(nested)], env=self.env, check=True, capture_output=True)
        (nested/'work').write_text('nested edits')
        (self.repo/'tracked').write_text('root edits')
        result = self.invoke(answer='y')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Nested untracked repository preserved', result.stdout)
        self.assertEqual((nested/'work').read_text(), 'nested edits')
        self.assertEqual((self.repo/'tracked').read_text(), 'root edits')
        self.assertEqual(self.git('stash', 'list'), '')

    def ignored_generated_repositories(self, mixed=False, directory='build'):
        rules = f'{directory}/*\n'
        if mixed:
            rules += f'!{directory}/notes.txt\n'
        (self.repo/'.gitignore').write_text(rules)
        self.git('add', '.gitignore')
        self.git('commit', '-m', 'ignore generated contents')
        self.git('push', 'origin', 'main')
        generated = self.repo/directory/'px4_sitl_default/external/project with spaces'
        subprocess.run(['git', 'init', str(generated)], env=self.env, check=True, capture_output=True)
        # Also cover a generated Git-file marker, not only a .git directory.
        other = generated.parent/'other generated project'
        other.mkdir()
        (other/'.git').write_text('generated gitdir metadata\n')
        stores = self.repo/directory/'px4_sitl_default/rootfs'
        (stores/'fs').mkdir(parents=True)
        (stores/'log').mkdir()
        sentinels = {stores/'fs/parameters.bson': b'preserve parameters',
                     stores/'log/flight.ulg': b'preserve ULog',
                     generated/'.git/HEAD': (generated/'.git/HEAD').read_bytes(),
                     other/'.git': (other/'.git').read_bytes()}
        for path, data in sentinels.items():
            path.write_bytes(data)
        return sentinels

    def test_ignored_build_contents_with_external_git_metadata_do_not_block_sync(self):
        sentinels = self.ignored_generated_repositories()
        self.assertEqual(self.git('status', '--porcelain'), '')
        self.assertEqual(self.git('ls-files', '--others', '--exclude-standard', '--directory'), 'build/')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(self.git('stash', 'list'), '')
        for path, data in sentinels.items():
            self.assertEqual(path.read_bytes(), data)

    def test_ignore_handling_is_not_a_hardcoded_build_path_exemption(self):
        sentinels = self.ignored_generated_repositories(directory='generated-cache')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        for path, data in sentinels.items():
            self.assertEqual(path.read_bytes(), data)

    def test_mixed_directory_decline_preserves_user_work_and_ignored_metadata(self):
        sentinels = self.ignored_generated_repositories(mixed=True)
        notes = self.repo/'build/notes.txt'
        notes.write_text('genuine user work\n')
        self.assertEqual(self.git('ls-files', '--others', '--exclude-standard', '--directory',
                                  '--no-empty-directory'), 'build/')
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('PX4 has local file edits', result.stdout)
        self.assertNotIn('Nested untracked repository', result.stdout)
        self.assertEqual(notes.read_text(), 'genuine user work\n')
        self.assertEqual(self.git('stash', 'list'), '')
        for path, data in sentinels.items():
            self.assertEqual(path.read_bytes(), data)

    def test_mixed_directory_consent_stashes_only_unignored_user_work(self):
        sentinels = self.ignored_generated_repositories(mixed=True)
        notes = self.repo/'build/notes.txt'
        notes.write_text('genuine user work\n')
        result = self.invoke(answer='yes')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        oid = self.git('rev-parse', 'refs/stash')
        self.assertEqual(self.git('ls-tree', '-r', '--name-only', oid+'^3'), 'build/notes.txt')
        self.assertEqual(self.git('show', oid+'^3:build/notes.txt'), 'genuine user work')
        self.assertEqual(self.git('rev-parse', 'refs/px4xplane/stashes/'+oid), oid)
        for path, data in sentinels.items():
            self.assertEqual(path.read_bytes(), data)

    def test_ignored_marker_does_not_hide_a_genuine_repository_in_same_directory(self):
        (self.repo/'.gitignore').write_text('mixed/generated/*\n')
        self.git('add', '.gitignore')
        self.git('commit', '-m', 'ignore generated contents')
        ignored = self.repo/'mixed/generated/output'
        ignored.mkdir(parents=True)
        (ignored/'.git').write_text('generated metadata\n')
        nested = self.repo/'mixed/genuine'
        subprocess.run(['git', 'init', str(nested)], env=self.env, check=True, capture_output=True)
        (nested/'work').write_text('keep nested work\n')
        (self.repo/'tracked').write_text('root edit\n')
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Nested untracked repository preserved', result.stdout)
        self.assertEqual((nested/'work').read_text(), 'keep nested work\n')
        self.assertEqual((self.repo/'tracked').read_text(), 'root edit\n')
        self.assertEqual(self.git('stash', 'list'), '')

    def test_unignored_embedded_git_file_is_preserved(self):
        source = self.module_source('linked-source')
        nested = self.repo/'nested/linked worktree'
        self.git_in(source, 'worktree', 'add', '--detach', str(nested))
        self.assertTrue((nested/'.git').is_file())
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Nested untracked repository preserved', result.stdout)
        self.assertTrue((nested/'.git').is_file())
        self.assertEqual(self.git('stash', 'list'), '')

    def test_untracked_bare_repository_is_preserved(self):
        bare = self.repo/'bare.git'
        subprocess.run(['git', 'init', '--bare', str(bare)], env=self.env, check=True, capture_output=True)
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Nested untracked repository preserved', result.stdout)
        self.assertTrue((bare/'HEAD').is_file())
        self.assertEqual(self.git('stash', 'list'), '')

    def test_nested_untracked_bare_repository_is_preserved(self):
        bare = self.repo/'outer/nested/bare.git'
        subprocess.run(['git', 'init', '--bare', str(bare)], env=self.env, check=True, capture_output=True)
        result = self.invoke(answer='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Nested untracked repository preserved', result.stdout)
        self.assertTrue((bare/'HEAD').is_file())
        self.assertEqual(self.git('stash', 'list'), '')

    def test_ignore_check_error_stops_without_stashing(self):
        nested = self.repo/'nested'
        subprocess.run(['git', 'init', str(nested)], env=self.env, check=True, capture_output=True)
        wrapper = '''
git() {
    if [ "$1" = check-ignore ]; then return 23; fi
    command git "$@"
}
'''
        result = self.invoke(answer='yes', git_wrapper=wrapper)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.git('stash', 'list'), '')
        self.assertTrue((nested/'.git').is_dir())

    def test_stash_postcondition_failure_stops_before_fetch(self):
        (self.repo/'tracked').write_text('work')
        wrapper = '''
git() {
    if [ "$1" = stash ] && [ "$2" = push ]; then
        command git "$@" || return $?
        printf 'remaining work' > left-behind
    elif [ "$1" = fetch ]; then
        echo UNEXPECTED_FETCH >&2
        return 97
    else command git "$@"; fi
}
'''
        result = self.invoke(answer='y', git_wrapper=wrapper)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('UNEXPECTED_FETCH', result.stderr)
        self.assertIn('expected clean state', result.stdout)
        oid = self.git('rev-parse', 'refs/stash')
        self.assertEqual(self.git('rev-parse', 'refs/px4xplane/stashes/'+oid), oid)


if __name__=='__main__':
    unittest.main()
