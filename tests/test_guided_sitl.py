"""Generic public startup tests; no simulator, private aircraft or network needed."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('guided_sitl',ROOT/'setup/launch_sitl.py')
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
RCS='#!/bin/sh\n# stock parameters/controllers\nparam import\n. px4-rc.mavlink\n'
MAVLINK=runner.GCS_START+'# stock offboard links\n'


def write_startup(px4):
    source = px4/'ROMFS/px4fmu_common'
    (source/'init.d-posix/airframes').mkdir(parents=True)
    (source/'init.d').mkdir()
    (source/'init.d-posix/rcS').write_text(RCS)
    (source/'init.d-posix/px4-rc.mavlink').write_text(MAVLINK)
    (source/'init.d/rc.vtol_defaults').write_text('. ${R}etc/init.d/rc.mc_defaults\n')
    (source/'init.d/rc.mc_defaults').write_text('param set-default MC_ROLL_P 1\n')
    for target, number in runner.AIRFRAMES.items():
        (source/f'init.d-posix/airframes/{number}_{target}').write_text(
            '. ${R}etc/init.d/rc.vtol_defaults\nparam set-default MAV_TYPE 20\n')
    return source


class GuidedStartupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home=Path(self.tmp.name).resolve()
        self.px4=self.home/'PX4 with spaces'
        self.source = write_startup(self.px4)
        build=self.px4/'build/px4_sitl_default'
        shutil.copytree(self.source, build/'etc')
        self.build = build
        (build/'bin').mkdir()
        (build/'bin/px4').write_bytes(b'mock binary')
        self.state=self.home/'runtime'
        self.argv=['--checkout',str(self.px4),'--target','xplane_qtailsitter',
                   '--host','172.23.128.1','--qgc-host','172.23.128.2',
                   '--state-root',str(self.state),'--source-label','official']
        self.target=runner.state_path(self.px4,'xplane_qtailsitter','official',self.state)

    def invoke(self,*extra,run=None):
        output=io.StringIO()
        default=mock.Mock(returncode=1)
        def commands(args,**kwargs):
            return default if args[0]=='pgrep' else mock.Mock(returncode=0)
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output), \
             mock.patch.object(runner.subprocess,'run',side_effect=run or commands) as calls, \
             mock.patch.object(runner.subprocess,'check_output',return_value='source-commit\n'):
            code=runner.main(self.argv+list(extra))
        return code,output.getvalue(),calls

    def test_api_version_is_read_only(self):
        with mock.patch.object(runner,'run') as launch, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(['--api-version']),0)
        launch.assert_not_called()

    def test_console_interrupt_has_no_python_traceback(self):
        output = io.StringIO()
        with mock.patch.object(runner, 'run', side_effect=KeyboardInterrupt), \
             contextlib.redirect_stderr(output):
            self.assertEqual(runner.main(self.argv), 130)
        self.assertIn('Launch interrupted.', output.getvalue())
        self.assertNotIn('Traceback', output.getvalue())

    def test_only_gcs_endpoint_changes(self):
        startup,mavlink=runner.prepare_startup(RCS,MAVLINK,'172.23.128.2')
        self.assertEqual(startup,RCS.replace('. px4-rc.mavlink\n','. ../mavlink.sh\n'))
        self.assertIn('-t 172.23.128.2 -o 14550',mavlink)
        self.assertTrue(mavlink.endswith('# stock offboard links\n'))

    def test_changed_startup_layout_fails_before_creating_state(self):
        (self.px4/'ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink').write_text('new unsupported layout')
        code,out,_=self.invoke()
        self.assertEqual(code,1)
        self.assertIn('startup layout changed',out)
        self.assertFalse(self.state.exists())

    def test_stale_build_fails_before_launch(self):
        (self.px4/'build/px4_sitl_default/etc/init.d-posix/rcS').write_text('stale')
        code,out,calls=self.invoke()
        self.assertEqual(code,1)
        self.assertIn('rebuild',out)
        calls.assert_not_called()

    def test_active_px4_refuses_state_mutation(self):
        code,out,_=self.invoke(run=lambda *a,**k:mock.Mock(returncode=0))
        self.assertEqual(code,1)
        self.assertIn('already running',out)
        self.assertFalse(self.state.exists())

    def test_stock_startup_inherits_console_and_has_separate_runtime(self):
        code,out,calls=self.invoke()
        self.assertEqual(code,0,out)
        command=calls.call_args_list[-1]
        self.assertEqual(command.args[0][1:5],['-w',str(self.target/'rootfs'),'-s','../startup.sh'])
        self.assertNotIn('stdin',command.kwargs)
        self.assertNotIn('stdout',command.kwargs)
        self.assertNotIn('capture_output',command.kwargs)
        self.assertEqual(command.kwargs['env']['PX4_SYS_AUTOSTART'],'5021')
        self.assertIn('NOT verified',out)
        meta=json.loads((self.target/'last-launch.json').read_text())
        self.assertEqual(meta['source_commit'],'source-commit')
        self.assertEqual(meta['qgc_host'],'172.23.128.2')

    def test_parameters_survive_relaunch_and_defaults_change(self):
        code,out,_=self.invoke()
        self.assertEqual(code,0,out)
        params=self.target/'rootfs/fs/parameters.bson'
        params.parent.mkdir()
        params.write_bytes(b'personal tuning')
        (self.px4/'ROMFS/px4fmu_common/init.d-posix/airframes/5021_xplane_qtailsitter').write_text('changed')
        shutil.copytree(self.source, self.build/'etc', dirs_exist_ok=True)
        code,out,_=self.invoke()
        self.assertEqual(code,0,out)
        self.assertIn('PRESERVED',out)
        self.assertEqual(params.read_bytes(),b'personal tuning')
        code,out,_=self.invoke()
        self.assertEqual(code,0,out)
        self.assertIn('PRESERVED',out)

    def test_explicit_reset_backs_up_only_selected_airframe(self):
        self.invoke()
        params=self.target/'rootfs/fs/parameters.bson'
        params.parent.mkdir()
        params.write_bytes(b'my tuning')
        other=self.state/'another-airframe'
        other.mkdir()
        (other/'parameters.bson').write_bytes(b'other tuning')
        code,out,_=self.invoke('--reset-params')
        self.assertEqual(code,0,out)
        self.assertFalse(params.exists())
        copies=list((self.target/'backups').glob('*/fs/parameters.bson'))
        self.assertEqual(len(copies),1)
        self.assertEqual(copies[0].read_bytes(),b'my tuning')
        self.assertEqual((other/'parameters.bson').read_bytes(),b'other tuning')

    def test_failed_backup_preserves_all_original_parameters(self):
        root=self.home/'stores'
        root.mkdir()
        (root/'parameters.bson').write_bytes(b'primary')
        (root/'parameters_backup.bson').write_bytes(b'backup')
        with mock.patch.object(runner.shutil,'copy2',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                runner.reset_parameters(root)
        self.assertEqual((root/'parameters.bson').read_bytes(),b'primary')
        self.assertEqual((root/'parameters_backup.bson').read_bytes(),b'backup')

    def test_reset_refuses_symlinked_parameter_directory(self):
        root=self.home/'runtime-root'
        root.mkdir()
        other=self.home/'other-airframe'
        other.mkdir()
        (other/'parameters.bson').write_bytes(b'preserve')
        (root/'fs').symlink_to(other,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'symlinked'):
            runner.reset_parameters(root)
        self.assertEqual((other/'parameters.bson').read_bytes(),b'preserve')

    def test_invalid_destination_rejected_without_state(self):
        for address in ('0.0.0.0','224.0.0.1','255.255.255.255','bad;command'):
            with self.subTest(address=address):
                code,_,_=self.invoke('--qgc-host',address)
                self.assertEqual(code,1)
                self.assertFalse(self.state.exists())

    def test_target_and_source_namespaces_do_not_mix(self):
        a=runner.state_path(self.px4,'xplane_qtailsitter','official',self.state)
        b=runner.state_path(self.px4,'xplane_qtailsitter','maintained',self.state)
        c=runner.state_path(self.px4,'xplane_alia250','official',self.state)
        self.assertEqual(len({a,b,c}),3)

    def test_every_public_airframe_uses_its_established_identity(self):
        for target, number in runner.AIRFRAMES.items():
            with self.subTest(target=target):
                code, out, calls = self.invoke('--target', target)
                self.assertEqual(code, 0, out)
                self.assertEqual(calls.call_args_list[-1].kwargs['env']['PX4_SYS_AUTOSTART'], str(number))
                self.assertIn(f'Airframe: {target} (ID {number})', out)
                self.assertIn('Binary SHA256:', out)

    def test_profile_post_inherited_defaults_and_common_startup_must_match_build(self):
        files = ('init.d-posix/airframes/5021_xplane_qtailsitter',
                 'init.d-posix/airframes/5021_xplane_qtailsitter.post',
                 'init.d/rc.vtol_defaults', 'init.d/rc.mc_defaults',
                 'init.d-posix/px4-rc.mavlink', 'init.d/rc.vehicle_setup')
        for name in files:
            with self.subTest(name=name):
                original = self.source/name
                before = original.read_bytes() if original.exists() else None
                if before is None:
                    original.write_text('# new source startup script\n')
                    (self.build/'etc'/name).write_text('# stale built startup script\n')
                else:
                    original.write_bytes(before+b'# source changed\n')
                code, out, calls = self.invoke('--reset-params')
                self.assertEqual(code, 1, out)
                self.assertIn('rebuild', out)
                calls.assert_not_called()
                self.assertFalse(self.state.exists())
                if before is None:
                    original.unlink()
                    (self.build/'etc'/name).unlink()
                else:
                    original.write_bytes(before)

    def test_stale_built_only_post_script_is_rejected(self):
        (self.build/'etc/init.d-posix/airframes/5021_xplane_qtailsitter.post').write_text('unexpected')
        code, _, calls = self.invoke()
        self.assertEqual(code, 1)
        calls.assert_not_called()
        self.assertFalse(self.state.exists())

    def test_px4_generated_startup_scripts_need_no_romfs_source_counterpart(self):
        for name in ('rc.autostart', 'rc.autostart.post', 'rc.serial', 'rc.filepaths'):
            self.assertFalse((self.source/'init.d'/name).exists())
            (self.build/'etc/init.d'/name).write_text('# Generated by the PX4 board build\n')
        for target in runner.AIRFRAMES:
            with self.subTest(target=target):
                code, out, _ = self.invoke('--target', target)
                self.assertEqual(code, 0, out)

    def test_generated_name_does_not_exempt_an_existing_source_counterpart(self):
        for name in ('rc.autostart', 'rc.autostart.post', 'rc.serial', 'rc.filepaths'):
            with self.subTest(name=name):
                source = self.source/'init.d'/name
                installed = self.build/'etc/init.d'/name
                source.write_text('# Source-owned startup\n')
                installed.write_text('# Stale built startup\n')
                code, out, calls = self.invoke('--reset-params')
                self.assertEqual(code, 1, out)
                self.assertIn(f'init.d/{name}', out)
                self.assertIn('rebuild', out)
                calls.assert_not_called()
                self.assertFalse(self.state.exists())
                source.unlink()
                installed.unlink()

    def test_unknown_built_only_startup_script_is_still_rejected(self):
        (self.build/'etc/init.d/rc.unexpected').write_text('# Not a known PX4 build output\n')
        code, out, calls = self.invoke()
        self.assertEqual(code, 1, out)
        self.assertIn('rc.unexpected', out)
        calls.assert_not_called()
        self.assertFalse(self.state.exists())

    def test_generated_outputs_do_not_exempt_missing_required_dependencies(self):
        (self.build/'etc/init.d/rc.autostart').write_text('# Generated by PX4\n')
        (self.source/'init.d/rc.mc_defaults').unlink()
        code, out, calls = self.invoke('--reset-params')
        self.assertEqual(code, 1, out)
        self.assertIn('rc.mc_defaults', out)
        calls.assert_not_called()
        self.assertFalse(self.state.exists())

    def test_inherited_defaults_change_preserves_tuning_and_warns_after_rebuild(self):
        self.invoke()
        params = self.target/'rootfs/parameters.bson'
        params.write_bytes(b'keep tuning')
        (self.source/'init.d/rc.mc_defaults').write_text('param set-default MC_ROLL_P 2\n')
        shutil.copytree(self.source, self.build/'etc', dirs_exist_ok=True)
        code, out, _ = self.invoke()
        self.assertEqual(code, 0, out)
        self.assertIn('PRESERVED', out)
        self.assertEqual(params.read_bytes(), b'keep tuning')

    def test_cross_airframe_environment_does_not_reach_px4(self):
        inherited = {'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_x500', 'PX4_SIMULATOR': 'gz',
                     'PX4_PARAM_SYS_AUTOCONFIG': '1', 'PX4_GZ_WORLD': 'other', 'PX4_ESTIMATOR': 'lpe',
                     'PX4_SIM_HOST_ADDR': '1.2.3.4', 'replay_mode': 'ekf2', 'R': '/other/',
                     'px4_instance': '3', 'SYS_AUTOSTART': '4001', 'VEHICLE_TYPE': 'other',
                     'autostart_file': '/other/airframe', 'PX4_NET_INTERFACE': 'eth0',
                     'PX4_SIM_SPEED_FACTOR': '2'}
        with mock.patch.dict(os.environ, inherited):
            code, out, calls = self.invoke()
        self.assertEqual(code, 0, out)
        env = calls.call_args_list[-1].kwargs['env']
        for name in inherited:
            if name not in {'PX4_SYS_AUTOSTART', 'PX4_SIM_MODEL', 'PX4_SIMULATOR',
                            'PX4_NET_INTERFACE', 'PX4_SIM_SPEED_FACTOR'}:
                self.assertNotIn(name, env)
        self.assertEqual(env['PX4_SIM_MODEL'], 'xplane_qtailsitter')
        self.assertEqual(env['PX4_SIMULATOR'], 'xplane')
        self.assertEqual(env['PX4_NET_INTERFACE'], 'eth0')


@unittest.skipUnless(sys.platform.startswith('linux'),'Guided shell setup targets Ubuntu/Debian and WSL')
class GuidedShellWorkflow(unittest.TestCase):
    def workflow(self, target=None, reverse=False, remember=False, bootstrap=False, worktree=False,
                 sync=False, sync_fail=False, build_fail=False):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp).resolve()
            home=base/'home'
            px4=base/'PX4'
            fake=base/'commands'
            home.mkdir()
            px4.mkdir()
            fake.mkdir()
            write_startup(px4)
            (px4/'.gitignore').write_text('build/\n')
            env={**os.environ,'HOME':str(home),'PATH':str(fake)+':/usr/bin:/bin',
                 'TERM':'dumb','XDG_STATE_HOME':str(home/'state'),
                 'GIT_CONFIG_NOSYSTEM':'1', 'GIT_ALLOW_PROTOCOL':'file',
                 'GIT_CONFIG_GLOBAL':os.devnull,
                 'PX4XPLANE_LAUNCHER_BASE_URL':'https://example.invalid/feature/setup'}
            for name in list(env):
                if name.startswith('PX4XPLANE_') and name != 'PX4XPLANE_LAUNCHER_BASE_URL':
                    env.pop(name)
            for args in (['init','-b','main'],['add','.'],
                         ['-c','user.name=Offline test','-c','user.email=test@example.invalid',
                          '-c','commit.gpgsign=false','commit','-m','fixture']):
                subprocess.run(['git','-C',str(px4),*args],env=env,check=True,capture_output=True)
            if worktree:
                linked = base/'linked PX4'
                subprocess.run(['git', '-C', str(px4), 'worktree', 'add', '-b', 'linked', str(linked)],
                               env=env, check=True, capture_output=True)
                px4 = linked
            make='''#!/usr/bin/python3
from pathlib import Path
import shutil
p=Path.cwd()
b=p/'build/px4_sitl_default'
shutil.copytree(p/'ROMFS/px4fmu_common',b/'etc',dirs_exist_ok=True)
(b/'bin').mkdir(parents=True,exist_ok=True)
binary=b/'bin/px4'
binary.write_text('#!/bin/sh\\nprintf "OFFLINE PX4 CONSOLE STUB\\\\n"\\n')
binary.chmod(0o755)
'''
            for name,text in [('make',make),('pgrep','#!/bin/sh\nexit 1\n'),
                              ('curl','#!/bin/sh\necho UNEXPECTED_NETWORK >&2\nexit 97\n')]:
                file=fake/name
                file.write_text(text)
                file.chmod(0o755)
            launcher = ROOT/'setup/setup_px4_sitl.sh'
            source_before = launcher.read_bytes()
            flags = ['--no-sync', '--px4-path', os.path.relpath(px4, base)] if reverse else [
                '--px4-path', str(px4), '--no-sync']
            if target:
                flags += ['--airframe', target]
            if remember:
                flags += ['--qgc-host', '192.168.1.42']
            if sync:
                remote = base/'remote.git'
                if not sync_fail:
                    subprocess.run(['git', 'init', '--bare', str(remote)], env=env, check=True, capture_output=True)
                    subprocess.run(['git', '-C', str(px4), 'push', str(remote), 'HEAD:main'],
                                   env=env, check=True, capture_output=True)
                flags.remove('--no-sync')
                flags += ['--px4-repo', str(remote), '--px4-branch', 'main']
            if build_fail:
                (fake/'make').write_text('#!/bin/sh\nexit 23\n')
            if bootstrap:
                launcher = base/'bootstrap.sh'
                shutil.copyfile(ROOT/'setup/setup_px4_sitl.sh', launcher)
                # Re-bootstrap must refresh the companion, even with a cached API-compatible copy.
                old_pair = home/'.local/share/px4xplane/older-pair'
                old_pair.mkdir(parents=True)
                (old_pair/'launch_sitl.py').write_text('print(2)\n')
                (old_pair.parent/'launcher').symlink_to(old_pair, target_is_directory=True)
                (home/'.px4sitl_config').write_text('LAUNCHER_BASE_URL=https://example.invalid/old/setup\n')
                # A local HTTP stand-in serves only the pair, without sockets or real network.
                (fake/'curl').write_text('''#!/usr/bin/python3
from pathlib import Path
import shutil, sys
args=sys.argv[1:]
index=args.index('-o')
url=args[index-1]
if url != 'https://example.invalid/feature/setup/launch_sitl.py':
    raise SystemExit('Unexpected URL: '+url)
shutil.copyfile(%r,args[index+1])
''' % str(ROOT/'setup/launch_sitl.py'))
                flags.remove('--no-sync')
                # Preserve real source selection/safety, but prohibit checkout/network operations.
                (fake/'git').write_text('''#!/bin/sh
case "$*" in
  *fetch*|*clone*|*submodule*) echo UNEXPECTED_GIT_NETWORK >&2; exit 97 ;;
esac
exec /usr/bin/git "$@"
''')
                (fake/'git').chmod(0o755)
                # Source-only bootstrap installation is tested independently of PX4 synchronization.
                bootstrap_source = launcher.read_text()
                setup_prefix = bootstrap_source.split('# === Introductory Information ===', 1)[0]
                pair_install = setup_prefix + '\nensure_sitl_runner\n'
                result = subprocess.run(['/bin/bash', '-c', pair_install, str(launcher)], env=env,
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                launcher = home/'.local/share/px4xplane/launcher/setup_px4_sitl.sh'
                flags.insert(0, '--no-sync')
                (fake/'curl').write_text('#!/bin/sh\necho UNEXPECTED_NETWORK >&2\nexit 97\n')
            result=subprocess.run(['/bin/bash',str(launcher), *flags],env=env,cwd=base,
                                  input='\n\nn\n1\n',capture_output=True,text=True,timeout=30)
            if sync_fail or build_fail:
                self.assertNotEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertNotIn('OFFLINE PX4 CONSOLE STUB', result.stdout)
                self.assertNotIn('UNEXPECTED_NETWORK', result.stderr)
                self.assertFalse((home/'state').exists())
                self.assertEqual((ROOT/'setup/setup_px4_sitl.sh').read_bytes(), source_before)
                return
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('OFFLINE PX4 CONSOLE STUB',result.stdout)
            expected_qgc = '192.168.1.42' if remember else '127.0.0.1'
            self.assertIn(f'QGC target: UDP {expected_qgc}:14550',result.stdout)
            self.assertNotIn('UNEXPECTED_NETWORK',result.stderr)
            self.assertNotIn('Setup Complete!',result.stdout)
            self.assertIn('PX4-XPlane launcher ', result.stdout)
            self.assertIn(f'Checkout: {px4}', result.stdout)
            for obsolete in ('Author:', 'Current Status:', 'Supported Aircraft:',
                             'Retired guards:', 'TECS', '#22493', '#27533', '#27601', '#27793'):
                self.assertNotIn(obsolete, result.stdout)
            self.assertTrue((home/'.local/share/px4xplane/launcher/launch_sitl.py').is_file())
            selected = target or 'xplane_ehang184'
            self.assertEqual(len(list((home/'state/px4xplane/sitl').glob(f'*/{selected}/last-launch.json'))),1)
            self.assertFalse((px4/'build/px4_sitl_default/rootfs/parameters.bson').exists())
            self.assertEqual((ROOT/'setup/setup_px4_sitl.sh').read_bytes(), source_before)
            if target:
                self.assertNotIn('Select the X-Plane airframe to build and launch:', result.stdout)
            if remember:
                env.pop('PX4XPLANE_LAUNCHER_BASE_URL')
                result = subprocess.run(['/bin/bash', str(home/'.local/share/px4xplane/launcher/setup_px4_sitl.sh'),
                                         '--no-sync', '--airframe', selected], env=env, cwd=base,
                                        input='\n\nn\n', capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertIn(f'Checkout: {px4}', result.stdout)
                self.assertIn('QGC target: UDP 192.168.1.42:14550', result.stdout)
                self.assertIn('LAUNCHER_BASE_URL=https://example.invalid/feature/setup',
                              (home/'.px4sitl_config').read_text())

    def test_existing_checkout_offline_setup_through_interactive_console(self):
        self.workflow()

    def test_all_five_direct_targets_and_both_no_sync_orders(self):
        for target in runner.AIRFRAMES:
            for reverse in (False, True):
                with self.subTest(target=target, reverse=reverse):
                    self.workflow(target=target, reverse=reverse)

    def test_remembered_checkout_qgc_and_branch_bootstrap_url(self):
        self.workflow(target='xplane_tb2', remember=True)

    def test_single_script_bootstrap_installs_reusable_pair(self):
        self.workflow(target='xplane_alia250', bootstrap=True)

    def test_linked_worktree_and_relative_path(self):
        self.workflow(target='xplane_qtailsitter', reverse=True, worktree=True)

    def test_local_remote_sync_does_not_download_or_rewrite_launcher(self):
        self.workflow(target='xplane_cessna172', sync=True)

    def test_sync_and_build_failures_do_not_launch(self):
        self.workflow(target='xplane_tb2', sync=True, sync_fail=True)
        self.workflow(target='xplane_tb2', build_fail=True)


class LauncherPairInstallation(unittest.TestCase):
    def test_explicit_updates_are_atomic_and_never_rewrite_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            commands = home/'commands'
            commands.mkdir()
            curl = commands/'curl'
            curl.write_text('''#!/usr/bin/python3
import os, shutil, sys
from pathlib import Path
args=sys.argv[1:]
index=args.index('-o')
url=args[index-1]
name=url.rsplit('/',1)[-1]
assert url == 'https://example.invalid/branch/setup/'+name
assert name in ('setup_px4_sitl.sh','launch_sitl.py')
assert '--connect-timeout' in args and '--max-time' in args
mode=os.environ.get('UPDATE_FAILURE','')
if name == 'launch_sitl.py' and mode == 'download':
    raise SystemExit(22)
destination=Path(args[index+1])
if name == 'launch_sitl.py' and mode == 'api':
    destination.write_text('print(1)\\n')
elif name == 'setup_px4_sitl.sh' and mode == 'shell-api':
    destination.write_text('#!/bin/bash\\nLAUNCHER_API_VERSION=1\\n')
else:
    shutil.copyfile(Path(os.environ['SETUP_SOURCE'])/name,destination)
''')
            curl.chmod(0o755)
            env = {'HOME': str(home), 'PATH': str(commands)+':/usr/bin:/bin',
                   'GIT_ALLOW_PROTOCOL': 'file', 'GIT_CONFIG_NOSYSTEM': '1', 'GIT_CONFIG_GLOBAL': os.devnull,
                   'PX4XPLANE_LAUNCHER_BASE_URL': 'https://example.invalid/branch/setup',
                   'SETUP_SOURCE': str(ROOT/'setup')}
            script = ROOT/'setup/setup_px4_sitl.sh'
            before = script.read_bytes()
            command = ['/bin/bash', str(script), '--update-launcher']
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            installed = home/'.local/share/px4xplane/launcher'
            initial = installed.resolve()
            self.assertEqual((installed/'setup_px4_sitl.sh').read_bytes(), before)
            for failure in ('download', 'api', 'shell-api'):
                with self.subTest(failure=failure):
                    result = subprocess.run(command, env={**env, 'UPDATE_FAILURE': failure},
                                            capture_output=True, text=True, timeout=30)
                    self.assertNotEqual(result.returncode, 0, result.stdout+result.stderr)
                    self.assertEqual(installed.resolve(), initial)
                    self.assertEqual((installed/'setup_px4_sitl.sh').read_bytes(), before)
                    self.assertEqual(script.read_bytes(), before)
                    self.assertEqual(list(installed.parent.glob('.launcher.*')), [])
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertEqual(installed.resolve(), initial)


if __name__=='__main__':
    unittest.main()
