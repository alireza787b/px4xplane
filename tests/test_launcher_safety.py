"""Exercise checkout preservation against tiny local Git fixtures, never PX4/network."""
import os
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

    def invoke(self,remote=None):
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
'''+function('require_clean_px4_checkout')+function('sync_px4_repo_to_remote')+'\nsync_px4_repo_to_remote\n'
        return subprocess.run(['/bin/bash','-c',script,'launcher-test',str(self.repo),str(remote or self.remote)],
                              env=self.env,capture_output=True,text=True,timeout=30)

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

    def test_force_clean_stash_and_parameter_deletion_are_absent(self):
        self.assertNotIn('git reset --hard',SOURCE)
        self.assertNotIn('git clean -',SOURCE)
        self.assertNotIn('git stash push',SOURCE)
        self.assertNotIn('git checkout -B',SOURCE)
        self.assertNotIn('reset_saved_sitl_parameters()',SOURCE)
        uninstall=SOURCE.split('if [ "$UNINSTALL_MODE" = true ]; then',1)[1].split('\nfi\n',1)[0]
        self.assertNotIn('rm -rf',uninstall)


if __name__=='__main__':
    unittest.main()
