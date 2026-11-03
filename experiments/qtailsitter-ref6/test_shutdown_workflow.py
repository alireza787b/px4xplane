"""Exercise shutdown -> live snapshot -> archive -> upload without an input prompt."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import flight
from attach_truth import DIAGNOSTIC_REFS


class ShutdownWorkflow(unittest.TestCase):
    def scenario(self, exit_code, mode='flight', scenario='baseline'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, repo, truth, state = (root/n for n in ('bundle', 'repo', 'truth', 'state'))
            for folder in (bundle, repo, truth, state):
                folder.mkdir()
            (bundle/'profile.sh').write_text('param set MPC_THR_HOVER 0.71\n')
            for name in ('design.json', 'parameters.json', 'bench-report.json'):
                (bundle/name).write_text('{}')
            (bundle/'diagnostics.bundle').write_text('fixture')
            config, acf = root/'config.ini', root/'Ref6.acf'
            config.write_text('fixture config')
            acf.write_text('fixture aircraft')
            (root/'Log.txt').write_text('fixture log')
            (truth/'manifest.json').write_text('{"session":"unchanged"}')
            (truth/'frames.csv').write_text('frame_id,sim_time_s\n1,1\n')
            (truth/'datarefs.csv').write_text('path,exists\n'+''.join(
                f'{ref},true\n' for ref in DIAGNOSTIC_REFS))
            binary = repo/'build/px4_sitl_default/bin/px4'
            binary.parent.mkdir(parents=True)
            binary.write_text('fixture binary')
            (state/'receipt.json').write_text(json.dumps({'restored':False, 'config':str(config)}))
            identity = {'manifest': {'system_path':str(root)}, 'acf_path':str(acf)}
            args = argparse.Namespace(host='127.0.0.1', qgc='127.0.0.1', truth=truth,
                repo=repo, state=state, mode=mode, scenario=scenario, bench_measurements=None, no_upload=False)
            def git(_repo, *parts):
                return subprocess.CompletedProcess([],0,stdout=flight.TARGET if parts==('rev-parse','FETCH_HEAD') else '')
            def execute(command, **kwargs):
                if command[0]=='script':
                    Path(command[-1]).write_text('REF6 PARAMETER GATE PASSED\n')
                    with (truth/'frames.csv').open('a') as output:
                        output.write('2,2\n3,3\n')
                    return subprocess.CompletedProcess(command,exit_code)
                return subprocess.CompletedProcess(command,0)
            with patch('flight.HERE',bundle), patch('flight.bundle_check'), patch('flight.no_px4'), \
                    patch('flight.settings',return_value={'enabled':True}), \
                    patch('flight.active_session',return_value=truth), \
                    patch('flight.loaded_truth',return_value=identity), patch('flight.git',side_effect=git), \
                    patch('flight.save'), patch('flight.restore') as restore, \
                    patch('flight.subprocess.run',side_effect=execute), \
                    patch('flight.upload') as upload, patch('builtins.input',side_effect=AssertionError('Unexpected prompt')):
                if exit_code:
                    with self.assertRaisesRegex(ValueError,'unsuccessfully'):
                        flight.flight(args)
                else:
                    flight.flight(args)
                upload.assert_called_once()
                restore.assert_called_once()
                archive = upload.call_args.args[0]
                with zipfile.ZipFile(archive) as zipped:
                    summary=json.loads(zipped.read(next(n for n in zipped.namelist() if n.endswith('/truth/summary.json'))))
                    session=json.loads(zipped.read(next(n for n in zipped.namelist() if n.endswith('/session.json'))))
                    self.assertEqual(summary['stop_reason'],'runner_snapshot')
                    self.assertEqual(summary['rows'],2)
                    self.assertEqual(session['px4_exit_code'],exit_code)
                    recorded=json.loads(zipped.read(next(n for n in zipped.namelist() if n.endswith('/identity.json'))))
                    self.assertEqual(recorded['runner'], '2026-11-03.ref6.12')
                    self.assertEqual(recorded['mode'], mode)
                    self.assertEqual(recorded['scenario'], scenario)
                    overrides = flight.scenario_overrides(mode, scenario)
                    self.assertEqual(recorded['scenario_overrides'], overrides)
                    startup=zipped.read(next(n for n in zipped.namelist() if n.endswith('/startup.sh'))).decode()
                    for name, value in overrides.items():
                        self.assertIn(f'param set {name} {value}\n', startup)
                        self.assertIn(f'param compare {name} {value};', startup)

    def test_successful_shutdown_needs_no_recorder_stop(self):
        self.scenario(0)

    def test_failed_session_is_also_collected_and_uploaded(self):
        self.scenario(1)

    def test_diagnostic_scenario_metadata_survives_collection(self):
        for scenario in ('recovery-jerk', 'braking-isolation'):
            for exit_code in (0, 1):
                with self.subTest(scenario=scenario, exit_code=exit_code):
                    self.scenario(exit_code, scenario=scenario)

    def test_hover_metadata_retains_fixed_yaw_override(self):
        for mode in ('hover', 'A'):
            with self.subTest(mode=mode):
                self.scenario(0, mode=mode)


if __name__=='__main__':
    unittest.main()
