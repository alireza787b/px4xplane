#!/usr/bin/env python3
"""Host-only regression tests. Never touches a real X-Plane install or PX4 checkout."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import attach_truth
import flight
import argparse
import io
from contextlib import redirect_stdout
import gates
import ref6
import source_state


class Ref6Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='.test-', dir=ref6.HERE)
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.bundle = self.base/'bundle'
        self.source = self.bundle/'aircraft'/ref6.NAME
        self.source.mkdir(parents=True)
        (self.source/(ref6.NAME+'.acf')).write_text('I\n1200 Version\nP acf/_num_engn 4\n')
        (self.bundle/'design.json').write_text(json.dumps({'name': ref6.NAME,
            'acf_sha256': ref6.digest(self.source/(ref6.NAME+'.acf'))}))
        params = {'MC_ROLLRATE_D': .001, 'MPC_JERK_AUTO': 4,
                  'MPC_ACC_HOR': 2, 'MPC_ACC_HOR_MAX': 2}
        for i in range(4):
            params[f'CA_ROTOR{i}_CT'] = 6.5
            params[f'CA_ROTOR{i}_KM'] = .01905 if i < 2 else -.01905
        (self.bundle/'parameters.json').write_text(json.dumps({'parameters': params}))
        (self.bundle/'profile.sh').write_text('#!/bin/sh\n'+''.join(f'param set {k} {v}\n' for k,v in params.items()))
        (self.bundle/'diagnostics.bundle').write_bytes(b'fixture')
        (self.bundle/'diagnostic-sha.txt').write_text(ref6.TARGET+'\n')
        self.root = self.base/'X-Plane 12'
        (self.root/'Aircraft/QuadTailsitter').mkdir(parents=True)
        self.original_acf = self.root/'Aircraft/QuadTailsitter/QuadTailsitter.acf'
        self.original_acf.write_bytes(b'original aircraft')
        self.config = self.root/'config.ini'
        self.original = b'; original comments\r\nconfig_name = QuadTailsitter\r\n[QuadTailsitter]\r\nchannel0 = motor\r\n'
        self.config.write_bytes(self.original)
        self.state = self.base/'state'

    def install(self):
        ref6.install(self.root, self.config, self.state, self.bundle)
        return json.loads((self.state/'receipt.json').read_text())

    def test_install_separate_and_idempotent(self):
        receipt = self.install()
        first = self.config.read_bytes()
        self.install()
        self.assertEqual(self.config.read_bytes(), first)
        self.assertEqual(self.original_acf.read_bytes(), b'original aircraft')
        self.assertEqual(ref6.read_config(first.decode())['global']['config_name'], ref6.NAME)
        self.assertEqual(ref6.verify_install(receipt, self.bundle).name, ref6.NAME+'.acf')
        self.assertIn(b'; original comments\r\n', first)
        self.assertIn(b'[QuadTailsitter]\r\nchannel0 = motor\r\n', first)

    def test_process_arguments_are_not_simulator_executables(self):
        process = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)',
                                    '/mnt/h/X-Plane 12'])
        try:
            with patch('ref6.shutil.which', return_value=None), patch('builtins.input', return_value='XPLANE CLOSED'):
                ref6.stopped()
        finally:
            process.terminate()
            process.wait(timeout=5)

    def test_install_preserves_bom_and_comments(self):
        self.config.write_bytes(b'\xef\xbb\xbf'+self.original)
        self.install()
        self.assertTrue(self.config.read_bytes().startswith(b'\xef\xbb\xbf; original comments\r\n'))

    def test_plugin_cosmetic_resave_allowed_without_rollback_data_loss(self):
        receipt = self.install()
        self.config.write_text(self.config.read_text()+'\n; plugin cosmetic resave\n')
        ref6.verify_install(receipt, self.bundle)
        with self.assertRaisesRegex(ValueError, 'edited'):
            ref6.rollback(self.state)
        self.config.write_text(self.config.read_text().replace('channel0 = motor', 'channel0 = WRONG'))
        with self.assertRaisesRegex(ValueError, 'Config values changed'):
            ref6.verify_install(receipt, self.bundle)

    def test_plugin_whitespace_resave_can_restore(self):
        self.install()
        self.config.write_text(self.config.read_text().replace('channel0 =', 'channel0='))
        ref6.rollback(self.state)
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_rollback_exact_bytes_and_idempotent(self):
        receipt = self.install()
        ref6.rollback(self.state)
        ref6.rollback(self.state)
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertTrue(Path(receipt['aircraft']).is_dir())
        self.assertEqual(self.original_acf.read_bytes(), b'original aircraft')

    def test_rollback_refuses_user_edits(self):
        self.install()
        self.config.write_text('user edit')
        with self.assertRaisesRegex(ValueError, 'edited'):
            ref6.rollback(self.state)
        self.assertEqual(self.config.read_text(), 'user edit')

    def test_rollback_corrupt_backup(self):
        self.install()
        (self.state/'config.original').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, 'Backup hash'):
            ref6.rollback(self.state)

    def test_refuse_existing_aircraft(self):
        (self.root/'Aircraft'/ref6.NAME).mkdir()
        with self.assertRaisesRegex(ValueError, 'existing Ref6'):
            self.install()
        self.assertFalse(self.state.exists())
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_missing_bundle_files(self):
        for relative in ['profile.sh', 'parameters.json', 'design.json', 'diagnostics.bundle', 'diagnostic-sha.txt',
                         f'aircraft/{ref6.NAME}/{ref6.NAME}.acf']:
            file = self.bundle/relative
            content = file.read_bytes()
            file.unlink()
            with self.assertRaisesRegex(ValueError, 'Missing bundle'):
                self.install()
            file.write_bytes(content)
        self.assertFalse(self.state.exists())

    def test_missing_config(self):
        self.config.unlink()
        with self.assertRaisesRegex(ValueError, 'Missing'):
            self.install()
        self.assertFalse(self.state.exists())

    def test_transaction_copy_failure(self):
        with patch('ref6.shutil.copytree', side_effect=OSError('copy failed')):
            with self.assertRaises(OSError):
                self.install()
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse((self.root/'Aircraft'/ref6.NAME).exists())
        ref6.rollback(self.state)

    def test_transaction_config_failure(self):
        with patch('ref6.atomic', side_effect=OSError('write failed')):
            with self.assertRaises(OSError):
                self.install()
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse((self.root/'Aircraft'/ref6.NAME).exists())

    def test_truth_identity_gates(self):
        receipt = self.install()
        session = self.base/'truth'
        session.mkdir()
        manifest = {'plugin': 'XPlaneTruthCapture', 'version': '0.1.9',
                    'aircraft_name': ref6.NAME+'.acf',
                    'aircraft_path': str(Path(receipt['aircraft'])/(ref6.NAME+'.acf'))}
        file = session/'manifest.json'
        file.write_text(json.dumps(manifest))
        self.assertEqual(ref6.loaded_truth(session, receipt, self.bundle)['acf_sha256'], receipt['acf_sha256'])
        for key, wrong in [('version', '0.1.8'), ('version', 'broken'),
                           ('aircraft_name', 'QuadTailsitter.acf'),
                           ('aircraft_path', str(self.original_acf)), ('aircraft_sha256', 'bad'),
                           ('plugin', 'other')]:
            file.write_text(json.dumps({**manifest, key: wrong}))
            with self.assertRaises(ValueError):
                ref6.loaded_truth(session, receipt, self.bundle)

    def test_installed_hash_gate(self):
        receipt = self.install()
        (Path(receipt['aircraft'])/(ref6.NAME+'.acf')).write_text('changed')
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            ref6.verify_install(receipt, self.bundle)

    def test_profile_verification_startup(self):
        startup = flight.startup(self.bundle/'profile.sh')
        self.assertIn('. "$REF6_PROFILE"', startup)
        self.assertIn('param compare MC_ROLLRATE_D 0.001', startup)
        self.assertIn('param compare SYS_AUTOSTART 5021', startup)
        self.assertIn('shutdown', startup)
        self.assertIn('-u 18571 -o 14550 -t "$REF6_QGC"', startup)

    def test_r13_baseline_retains_and_checks_reusable_profile(self):
        profile = self.bundle/'profile.sh'
        original = profile.read_bytes()
        text = flight.startup(profile, 180, overrides=ref6.scenario_overrides('flight', 'baseline'))
        for name, value in (('MPC_JERK_AUTO', 4), ('MPC_ACC_HOR', 2), ('MPC_ACC_HOR_MAX', 2)):
            self.assertIn(f'param compare {name} {value};', text)
            self.assertNotIn(f'param set {name} ', text)
        self.assertIn('param compare MC_ROLLRATE_D 0.001;', text)
        self.assertEqual(profile.read_bytes(), original)

    def test_lateral_hover_scenario_has_checked_fixed_yaw(self):
        text = flight.startup(self.bundle/'profile.sh', 50, overrides=ref6.scenario_overrides('hover', 'baseline'))
        self.assertIn('param set MPC_YAW_MODE 5\n', text)
        self.assertIn('param compare MPC_YAW_MODE 5', text)
        self.assertIn('param compare MIS_TAKEOFF_ALT 50', text)
        self.assertNotIn('param set MPC_YAW_MODE', flight.startup(self.bundle/'profile.sh', 180))

    def test_scenario_overrides_are_narrow_and_mode_specific(self):
        for mode, scenario, expected in (
                ('flight', 'baseline', {}),
                ('hover', 'baseline', {'MPC_YAW_MODE': 5}),
                ('A', 'baseline', {}),
                ('flight', 'recovery-jerk', {'MPC_JERK_AUTO': 4.0}),
                ('flight', 'braking-isolation', {'MPC_XY_VEL_MAX': 12.0})):
            with self.subTest(mode=mode, scenario=scenario):
                self.assertEqual(ref6.scenario_overrides(mode, scenario), expected)
        with self.assertRaisesRegex(ValueError, 'Unknown scenario'):
            ref6.scenario_overrides('flight', 'unknown')

    def test_scenario_startup_applies_after_profile_and_altitude_and_gates_effective_values(self):
        profile = self.bundle/'profile.sh'
        original = profile.read_bytes()
        for mode, scenario, name, old in (
                ('hover', 'baseline', 'MPC_YAW_MODE', '4'),
                ('flight', 'recovery-jerk', 'MPC_JERK_AUTO', '8'),
                ('flight', 'braking-isolation', 'MPC_XY_VEL_MAX', '5')):
            with self.subTest(scenario=scenario):
                overrides = ref6.scenario_overrides(mode, scenario)
                value = overrides[name]
                with patch('flight.profile_params', return_value={name: old, 'MIS_TAKEOFF_ALT': '10'}):
                    text = flight.startup(profile, 180, overrides=overrides)
                applied = text.index(f'param set {name} {value}\n')
                self.assertLess(text.index('. "$REF6_PROFILE"\n'), text.index('param set MIS_TAKEOFF_ALT 180\n'))
                self.assertLess(text.index('param set MIS_TAKEOFF_ALT 180\n'), applied)
                self.assertLess(applied, text.index('param show '))
                self.assertIn(f'param show {name}\nif ! param compare {name} {value}; then\n', text)
                self.assertNotIn(f'param compare {name} {old};', text)
                self.assertEqual(text.count(f'param compare {name} '), 1)
                self.assertIn('param compare MIS_TAKEOFF_ALT 180;', text)
                self.assertNotIn('param compare MIS_TAKEOFF_ALT 10;', text)
        self.assertEqual(profile.read_bytes(), original)

    def test_nonbaseline_wrong_mode_rejected_before_lock_or_dispatch(self):
        for mode in ('hover', 'A', 'check', 'install', 'bench', 'collect', 'restore', 'update',
                     'configure-upload', 'disable-upload', 'retry-uploads'):
            for scenario in ('recovery-jerk', 'braking-isolation'):
                with self.subTest(mode=mode, scenario=scenario), \
                     patch('sys.argv', ['ref6.py', mode, '--scenario', scenario]), \
                     patch('pathlib.Path.open') as opened, patch('ref6.dispatch') as dispatch:
                    with self.assertRaisesRegex(ValueError, 'only valid for mode flight'):
                        ref6.main()
                    opened.assert_not_called()
                    dispatch.assert_not_called()

    def test_direct_entry_points_reject_wrong_mode_before_side_effects(self):
        for entry in (ref6.dispatch, flight.flight):
            for mode in ('hover', 'A'):
                for scenario in ('recovery-jerk', 'braking-isolation'):
                    with self.subTest(entry=entry.__name__, mode=mode, scenario=scenario), \
                         patch('flight.bundle_check') as checked, patch('flight.git') as git, \
                         patch('pathlib.Path.open') as opened:
                        with self.assertRaisesRegex(ValueError, 'only valid for mode flight'):
                            entry(argparse.Namespace(mode=mode, scenario=scenario))
                        checked.assert_not_called()
                        git.assert_not_called()
                        opened.assert_not_called()

    def test_flight_scenario_cli_and_diagnostic_warnings(self):
        for scenario in ref6.SCENARIOS:
            with self.subTest(scenario=scenario):
                output = io.StringIO()
                with patch('sys.argv', ['ref6.py', 'flight', '--scenario', scenario, '--yes-debug',
                                        '--host', '127.0.0.1', '--truth', str(self.base/'truth')]), \
                     patch('ref6.bundle_check'), patch('gates.report', return_value={}), \
                     patch('flight.flight') as launch, redirect_stdout(output):
                    ref6.main()
                self.assertEqual(launch.call_args.args[0].scenario, scenario)
                text = output.getvalue()
                self.assertIn('Scenario: '+scenario, text)
                self.assertIn('R14: P-only FW damping test, FW_RR_P=0.18 (was 0.14)', text)
                self.assertIn(ref6.VERSION, text)
                self.assertIn('official PX4 BASE: '+ref6.BASE, text)
                self.assertIn('log-only TARGET: '+ref6.TARGET, text)
                self.assertIn('#28350', text)
                self.assertIn('Same R13 firmware, MC, TECS, trajectory and ACF', text)
                self.assertNotIn('PID/ACF unchanged', text)
                if scenario == 'braking-isolation':
                    self.assertIn('increases the 1.3*bound threshold to 15.6 m/s', text)
                    self.assertIn('suppresses initial 9.3 m/s braking too', text)
                    self.assertIn('NOT a production fix', text)
                    self.assertIn('Explicit diagnostic only; NOT recommended', text)
                elif scenario == 'recovery-jerk':
                    self.assertIn('diagnostics only; no full-flight promise', text)
                    self.assertIn('now the same as baseline', text)
                else:
                    self.assertNotIn('WARNING:', text)

    def test_update_points_to_current_fw_scenario(self):
        args = argparse.Namespace(state=self.state, mode='update', manifest_url='https://example.invalid/current')
        output = io.StringIO()
        with patch('update_debug.update', return_value=ref6.VERSION), redirect_stdout(output):
            ref6.dispatch(args)
        self.assertIn('Next: bash run.sh prepare-truth', output.getvalue())
        self.assertIn('QUICKSTART.md for the latest-upstream baseline flight', output.getvalue())
        self.assertNotIn('Next: bash run.sh flight --yes-debug', output.getvalue())
        self.assertNotIn('Next: bash run.sh hover', output.getvalue())

    def test_check_verifies_bundle_prerequisites_without_requiring_new_base_or_fetch(self):
        args = argparse.Namespace(state=self.state, mode='check', repo=self.base/'old-repo')
        output = io.StringIO()
        with patch('ref6.HERE', self.bundle), patch('ref6.bundle_check'), \
             patch('ref6.subprocess.run') as execute, redirect_stdout(output):
            ref6.dispatch(args)
        execute.assert_called_once_with(['git', '-C', str(args.repo), 'bundle', 'verify',
                                         str(self.bundle/'diagnostics.bundle')], check=True)
        self.assertIn(ref6.BASE, output.getvalue())
        self.assertIn(ref6.TARGET, output.getvalue())
        self.assertIn('#28350', output.getvalue())

    def test_profile_dynamic_values_refused(self):
        (self.bundle/'profile.sh').write_text('param set MC_ROLLRATE_D $VALUE\n')
        with self.assertRaisesRegex(ValueError, 'literal'):
            ref6.bundle_check(self.bundle)

    def test_unicast(self):
        self.assertEqual(ref6.ipv4('192.168.1.2'), '192.168.1.2')
        for address in ['0.0.0.0', '255.255.255.255', '239.1.2.3', 'hostname', '::1']:
            with self.assertRaises(ValueError):
                ref6.ipv4(address)

    def test_stopped_requires_confirmation_when_windows_unknown(self):
        with patch('ref6.subprocess.run', return_value=subprocess.CompletedProcess([], 1)), \
             patch('ref6.shutil.which', return_value=None), patch('builtins.input', return_value='no'):
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                ref6.stopped()

    def test_running_windows_refused(self):
        outputs = [subprocess.CompletedProcess([], 1),
                   subprocess.CompletedProcess([], 0, stdout='"X-Plane.exe","123"')]
        with patch('ref6.subprocess.run', side_effect=outputs), patch('ref6.shutil.which', return_value='tasklist.exe'):
            with self.assertRaisesRegex(ValueError, 'running'):
                ref6.stopped()

    def test_truth_growth_and_stability(self):
        session = self.base/'truth'
        session.mkdir()
        csv = session/'frames.csv'
        csv.write_text('frame_id,sim_time_s\n1,1\n')
        with self.assertRaisesRegex(ValueError, 'No actively growing'):
            attach_truth.active_session(session, interval=0)
        def grow(_):
            with csv.open('a') as stream:
                stream.write('2,2\n')
        with patch('attach_truth.time.sleep', side_effect=grow):
            self.assertEqual(attach_truth.active_session(session, interval=0), session)
        run = self.base/'run'
        run.mkdir()
        (run/'truth-preflight.json').write_text(json.dumps({'frames_bytes': csv.stat().st_size}))
        with self.assertRaisesRegex(ValueError, 'not grown'):
            attach_truth.attach(session, run, interval=0)
        grow(0)
        with patch('attach_truth.time.sleep', side_effect=grow):
            with self.assertRaisesRegex(ValueError, 'still changing'):
                attach_truth.attach(session, run, interval=0)
        attach_truth.attach(session, run, interval=0)
        self.assertTrue((run/'truth/frames.csv').exists())

    def test_source_restore_preserves_branch_and_refuses_dirty(self):
        repo = self.base/'repo'
        repo.mkdir()
        source_state.git(repo, 'init', '-b', 'original')
        source_state.git(repo, '-c', 'user.name=Test', '-c', 'user.email=test@example.test',
                         'commit', '--allow-empty', '-m', 'fixture')
        snapshot = self.base/'source.json'
        source_state.save(repo, snapshot)
        source_state.git(repo, 'switch', '--detach', 'HEAD')
        source_state.restore(repo, snapshot)
        self.assertEqual(source_state.identity(repo)['branch'], 'original')
        (repo/'user-file').write_text('retain')
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            source_state.restore(repo, snapshot)
        self.assertEqual((repo/'user-file').read_text(), 'retain')

    def test_design_hash_mismatch(self):
        design = self.bundle/'design.json'
        design.write_text(json.dumps({'name': ref6.NAME, 'acf_sha256': 'bad'}))
        with self.assertRaisesRegex(ValueError, 'Design name/ACF hash'):
            ref6.bundle_check(self.bundle)

    def test_parameters_mismatch(self):
        (self.bundle/'parameters.json').write_text(json.dumps({'parameters': {'MC_ROLLRATE_D': .002}}))
        with self.assertRaisesRegex(ValueError, 'Profile/parameters'):
            ref6.bundle_check(self.bundle)

    def test_bench_pending_and_measured_gates(self):
        design = {'acf_sha256': 'fixture', 'rpm_limit': 13000, 'shaft_power_budget_w': 3000, 'mass_kg': 6}
        self.assertFalse(gates.evaluate(design, None)['bench_review_complete'])
        self.assertNotIn('eligible_for_hover', gates.evaluate(design, None))
        measurements = {key: True for key in ('prop_geometry_expert_reviewed', 'mass_cg_measured',
            'force_moment_signs_confirmed', 'static_thrust_q_over_t_reviewed', 'loaded_accel_decel_reviewed')}
        measurements.update(acf_sha256='fixture', operator='Test', evidence_reference='fixture',
                            measured_max_rpm=[12000]*4, measured_max_shaft_power_w=[700]*4,
                            measured_hover_thrust_n=[15]*4)
        result = gates.evaluate(design, measurements)
        self.assertTrue(result['bench_review_complete'])
        self.assertFalse(result['fw_release'])
        self.assertFalse(result['flight_validated'])
        for key, value in [('measured_max_rpm', [13001]*4), ('measured_max_shaft_power_w', [751]*4),
                           ('measured_hover_thrust_n', [1]*4), ('measured_max_rpm', [float('nan')]*4),
                           ('force_moment_signs_confirmed', False), ('acf_sha256', 'other')]:
            self.assertFalse(gates.evaluate(design, {**measurements, key: value})['bench_review_complete'])

    def test_recovery_rejects_different_session(self):
        run = self.base/'run'
        run.mkdir()
        (run/'identity.json').write_text('{}')
        (run/'truth-source.txt').write_text(str(self.base/'original'))
        with self.assertRaisesRegex(ValueError, 'original flight capture'):
            ref6.recovery_truth(run, self.base/'other')

    def test_build_failure_restores_source_and_collects(self):
        receipt = self.install()
        truth = self.base/'truth'
        truth.mkdir()
        measurements = self.base/'measurements.json'
        measurements.write_text('{}')
        (self.bundle/'bench-report.json').write_text('{}')
        (self.root/'Log.txt').write_text('fixture')
        args = __import__('argparse').Namespace(host='127.0.0.1', qgc='127.0.0.1', truth=truth,
            repo=self.base/'repo', state=self.state, mode='hover', bench_measurements=None, no_upload=True)
        identity = {'manifest': {'system_path': str(self.root)},
                    'acf_path': str(Path(receipt['aircraft'])/(ref6.NAME+'.acf'))}
        def fake_git(repo, *parts):
            return subprocess.CompletedProcess([], 0, stdout=ref6.TARGET if parts == ('rev-parse', 'FETCH_HEAD') else '')
        with patch('flight.HERE', self.bundle), patch('flight.bundle_check'), patch('flight.no_px4'), \
             patch('flight.active_session', return_value=truth), patch('flight.loaded_truth', return_value=identity), \
             patch('flight.git', side_effect=fake_git), patch('flight.save'), \
             patch('flight.restore') as restored, patch('flight.collect') as collected, \
             patch('flight.subprocess.run', side_effect=subprocess.CalledProcessError(2, 'make')), \
             patch('flight.tempfile.gettempdir', return_value=str(self.base)):
            with self.assertRaises(subprocess.CalledProcessError):
                flight.flight(args)
            restored.assert_called_once()
            collected.assert_called_once()

    def test_bundle_import_into_old_checkout_restores_main_and_submodule_identities(self):
        receipt = self.install()
        truth = self.base/'truth'
        truth.mkdir()
        (truth/'datarefs.csv').write_text('path,exists\n'+''.join(
            f'{ref},true\n' for ref in attach_truth.DIAGNOSTIC_REFS))
        (self.bundle/'bench-report.json').write_text('{}')
        (self.root/'Log.txt').write_text('fixture')
        identity = {'manifest': {'system_path': str(self.root)},
                    'acf_path': str(Path(receipt['aircraft'])/(ref6.NAME+'.acf'))}
        git = source_state.git

        def init(repo):
            repo.mkdir()
            git(repo, 'init', '-b', 'original')
            git(repo, 'config', 'user.name', 'Test')
            git(repo, 'config', 'user.email', 'test@example.test')

        def commit(repo, message):
            git(repo, 'commit', '--allow-empty', '-m', message)
            return git(repo, 'rev-parse', 'HEAD').stdout.strip()

        for detached in (False, True):
            with self.subTest(detached=detached), tempfile.TemporaryDirectory(dir=self.base) as directory:
                root = Path(directory)
                module, repo, producer = (root/name for name in ('module', 'repo', 'producer'))
                init(module)
                module_old = commit(module, 'original submodule')
                module_new = commit(module, 'build submodule')
                init(repo)
                git(repo, '-c', 'protocol.file.allow=always', 'submodule', 'add', str(module), 'module')
                git(repo/'module', 'switch', '-c', 'saved-module', module_old)
                git(repo, 'add', '.')
                old = commit(repo, 'old bundle prerequisite')
                if detached:
                    git(repo, 'switch', '--detach', old)
                    git(repo/'module', 'switch', '--detach', module_old)
                original = source_state.identity(repo)
                original_module = source_state.identity(repo/'module')
                git(root, 'clone', '--no-hardlinks', str(repo), str(producer))
                git(producer, 'config', 'user.name', 'Test')
                git(producer, 'config', 'user.email', 'test@example.test')
                base = commit(producer, 'new official upstream base')
                target = commit(producer, 'log-only diagnostic target')
                git(producer, 'update-ref', ref6.BUNDLE_REF, target)
                git(producer, 'bundle', 'create', str(self.bundle/'diagnostics.bundle'), ref6.BUNDLE_REF, '^'+old)
                self.assertNotEqual(git(repo, 'cat-file', '-e', base+'^{commit}', check=False).returncode, 0)
                args = argparse.Namespace(host='127.0.0.1', qgc='127.0.0.1', truth=truth,
                    repo=repo, state=self.state, mode='flight', scenario='baseline',
                    bench_measurements=None, no_upload=True)

                real_run = subprocess.run

                def build(command, **kwargs):
                    if command[0] != 'make':
                        return real_run(command, **kwargs)
                    self.assertEqual(command[0], 'make')
                    self.assertEqual(source_state.identity(repo), {'branch': '', 'commit': target})
                    git(repo, 'cat-file', '-e', base+'^{commit}')
                    git(repo/'module', 'switch', '--detach', module_new)
                    raise subprocess.CalledProcessError(2, command)

                with patch('flight.HERE', self.bundle), patch('flight.bundle_check'), patch('flight.no_px4'), \
                     patch('flight.BASE', base), patch('flight.TARGET', target), \
                     patch('flight.active_session', return_value=truth), \
                     patch('flight.loaded_truth', side_effect=lambda *args: dict(identity)), \
                     patch('flight.collect') as collected, \
                     patch('flight.subprocess.run', side_effect=build):
                    with self.assertRaises(subprocess.CalledProcessError):
                        flight.flight(args)
                    collected.assert_called_once()
                self.assertEqual(source_state.identity(repo), original)
                self.assertEqual(source_state.identity(repo/'module'), original_module)
                self.assertFalse(git(repo, 'status', '--porcelain').stdout)

    def test_missing_diagnostics_rejected_before_checkout_or_run_creation(self):
        self.install()
        truth = self.base/'truth'
        truth.mkdir()
        catalog = truth/'datarefs.csv'
        for scenario in ref6.SCENARIOS:
            for missing in ('absent', 'unavailable', 'catalog'):
                with self.subTest(scenario=scenario, missing=missing):
                    catalog.write_text('path,exists\n'+''.join(
                        f'{ref},{"false" if missing == "unavailable" and index == 0 else "true"}\n'
                        for index, ref in enumerate(attach_truth.DIAGNOSTIC_REFS)
                        if not (missing == 'absent' and index == 0)))
                    if missing == 'catalog':
                        catalog.unlink()
                    args = argparse.Namespace(host='127.0.0.1', qgc='127.0.0.1', truth=truth,
                        repo=self.base/'repo', state=self.state, mode='flight',
                        scenario=scenario, no_upload=True)
                    with patch('flight.HERE', self.bundle), patch('flight.bundle_check'), \
                         patch('flight.no_px4'), patch('flight.active_session', return_value=truth), \
                         patch('flight.loaded_truth', return_value={}), \
                         patch('flight.git', return_value=subprocess.CompletedProcess([], 0, stdout='')) as git, \
                         patch('flight.save') as saved, patch('flight.subprocess.run') as execute, \
                         patch('flight.tempfile.gettempdir', return_value=str(self.base)):
                        error = FileNotFoundError if missing == 'catalog' else ValueError
                        with self.assertRaises(error):
                            flight.flight(args)
                        self.assertEqual([call.args[1:] for call in git.call_args_list],
                            [('status', '--porcelain')])
                        saved.assert_not_called()
                        execute.assert_not_called()
                    self.assertFalse((self.bundle/'runs').exists())

    def test_restore_removes_only_new_clean_module_worktrees(self):
        git = source_state.git
        module, repo = self.base/'module-source', self.base/'restore-repo'
        for directory in (module, repo):
            directory.mkdir()
            git(directory, 'init', '-b', 'original')
            git(directory, 'config', 'user.name', 'Test')
            git(directory, 'config', 'user.email', 'test@example.test')
            (directory/'tracked').write_text('preserved')
            git(directory, 'add', '.')
            git(directory, 'commit', '-m', 'original')
        snapshot = self.base/'before.json'
        source_state.save(repo, snapshot)
        original = source_state.identity(repo)
        git(repo, 'switch', '-c', 'diagnostic')
        git(repo, '-c', 'protocol.file.allow=always', 'submodule', 'add', str(module), 'new-module')
        git(repo, 'commit', '-am', 'new upstream module')
        (repo/'new-module/tracked').write_text('user edit during run')
        with self.assertRaises(subprocess.CalledProcessError):
            source_state.restore(repo, snapshot)
        self.assertEqual((repo/'new-module/tracked').read_text(), 'user edit during run')
        self.assertEqual(source_state.identity(repo)['branch'], 'diagnostic')
        # Returning the fixture to its committed bytes permits clean deinitialization.
        (repo/'new-module/tracked').write_text('preserved')
        source_state.restore(repo, snapshot)
        self.assertEqual(source_state.identity(repo), original)
        self.assertFalse(git(repo, 'status', '--porcelain').stdout)
        self.assertFalse((repo/'new-module/tracked').exists())
        self.assertTrue((repo/'.git/modules/new-module/objects').is_dir())

    def test_hover_aliases_accept_pending_report_without_measurements(self):
        for mode in ('hover', 'A', 'flight'):
            with patch('sys.argv', ['ref6.py', mode, '--host', '127.0.0.1', '--truth', str(self.base/'truth')]), patch('ref6.bundle_check'), \
                 patch('gates.report', return_value={'bench_review_complete': False}) as report, \
                 patch('builtins.input', return_value='SITL EXPERIMENT'), patch('flight.flight') as launch:
                ref6.main()
                report.assert_called_once_with(None)
                launch.assert_called_once()
                self.assertIsNone(launch.call_args.args[0].bench_measurements)
                self.assertEqual(launch.call_args.args[0].mode, mode)
                self.assertEqual(launch.call_args.args[0].scenario, 'baseline')

    def test_hover_requires_explicit_experiment_consent(self):
        with patch('sys.argv', ['ref6.py', 'hover']), patch('ref6.bundle_check'), \
             patch('gates.report', return_value={}), patch('builtins.input', return_value=''), \
             patch('flight.flight') as launch:
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                ref6.main()
            launch.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
