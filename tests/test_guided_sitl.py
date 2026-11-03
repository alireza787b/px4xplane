"""Generic public startup tests; no simulator, private aircraft or network needed."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
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


class GuidedStartupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home=Path(self.tmp.name).resolve()
        self.px4=self.home/'PX4 with spaces'
        source=self.px4/'ROMFS/px4fmu_common/init.d-posix'
        (source/'airframes').mkdir(parents=True)
        (source/'rcS').write_text(RCS)
        (source/'px4-rc.mavlink').write_text(MAVLINK)
        (source/'airframes/5021_xplane_qtailsitter').write_text('param set-default MAV_TYPE 20\n')
        build=self.px4/'build/px4_sitl_default'
        (build/'etc/init.d-posix').mkdir(parents=True)
        (build/'etc/init.d-posix/rcS').write_text(RCS)
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
        b=runner.state_path(self.px4,'xplane_qtailsitter','validation',self.state)
        c=runner.state_path(self.px4,'xplane_alia250','official',self.state)
        self.assertEqual(len({a,b,c}),3)


@unittest.skipUnless(sys.platform.startswith('linux'),'Guided shell setup targets Ubuntu/Debian and WSL')
class GuidedShellWorkflow(unittest.TestCase):
    def test_existing_checkout_offline_setup_through_interactive_console(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp).resolve()
            home=base/'home'
            px4=base/'PX4'
            fake=base/'commands'
            home.mkdir()
            px4.mkdir()
            fake.mkdir()
            source=px4/'ROMFS/px4fmu_common/init.d-posix'
            (source/'airframes').mkdir(parents=True)
            (source/'rcS').write_text(RCS)
            (source/'px4-rc.mavlink').write_text(MAVLINK)
            (source/'airframes/5010_xplane_ehang184').write_text('param set-default MAV_TYPE 2\n')
            (px4/'.gitignore').write_text('build/\n')
            env={**os.environ,'HOME':str(home),'PATH':str(fake)+':/usr/bin:/bin',
                 'TERM':'dumb','XDG_STATE_HOME':str(home/'state'),
                 'PX4XPLANE_SKIP_LAUNCHER_UPDATE':'1','GIT_CONFIG_NOSYSTEM':'1',
                 'GIT_CONFIG_GLOBAL':os.devnull}
            for args in (['init','-b','main'],['add','.'],
                         ['-c','user.name=Offline test','-c','user.email=test@example.invalid',
                          '-c','commit.gpgsign=false','commit','-m','fixture']):
                subprocess.run(['git','-C',str(px4),*args],env=env,check=True,capture_output=True)
            make='''#!/usr/bin/python3
from pathlib import Path
import shutil
p=Path.cwd()
b=p/'build/px4_sitl_default'
(b/'etc/init.d-posix').mkdir(parents=True)
shutil.copyfile(p/'ROMFS/px4fmu_common/init.d-posix/rcS',b/'etc/init.d-posix/rcS')
(b/'bin').mkdir()
binary=b/'bin/px4'
binary.write_text('#!/bin/sh\\nprintf "OFFLINE PX4 CONSOLE STUB\\\\n"\\n')
binary.chmod(0o755)
'''
            for name,text in [('make',make),('pgrep','#!/bin/sh\nexit 1\n'),
                              ('curl','#!/bin/sh\necho UNEXPECTED_NETWORK >&2\nexit 97\n')]:
                file=fake/name
                file.write_text(text)
                file.chmod(0o755)
            result=subprocess.run(['/bin/bash',str(ROOT/'setup/setup_px4_sitl.sh'),
                                   '--px4-path',str(px4),'--no-sync'],env=env,
                                  input='\n\nn\n1\n',capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('OFFLINE PX4 CONSOLE STUB',result.stdout)
            self.assertIn('QGC target: UDP 127.0.0.1:14550',result.stdout)
            self.assertNotIn('UNEXPECTED_NETWORK',result.stderr)
            self.assertNotIn('Setup Complete!',result.stdout)
            self.assertTrue((home/'.local/share/px4xplane/launch_sitl.py').is_file())
            self.assertEqual(len(list((home/'state/px4xplane/sitl').glob('*/xplane_ehang184/last-launch.json'))),1)
            self.assertFalse((px4/'build/px4_sitl_default/rootfs/parameters.bson').exists())


if __name__=='__main__':
    unittest.main()
