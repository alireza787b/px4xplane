#!/usr/bin/env python3
"""Live snapshot tests using only small, local TruthCapture fixtures."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import attach_truth as truth


class LiveSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root/'session'
        self.run = self.root/'run'
        self.source.mkdir()
        self.run.mkdir()
        self.frames = self.source/'frames.csv'
        self.header = b'frame_id,sim_time_s,value\r\n'
        self.frames.write_bytes(self.header + b'10,1.0,old\r\n')
        (self.source/'manifest.json').write_text('{"session": "test"}\n')
        (self.source/'datarefs.json').write_text('{"fields": []}\n')
        (self.source/'capture_config.ini').write_text('enabled=true\n')
        (self.source/'summary.json').write_text('{"stop_reason": "menu_stop"}\n')
        self.initial = json.loads(json.dumps(truth.begin_snapshot(self.source)))

    def append(self, data=b'11,1.1,new\r\n12,1.2,new\r\n'):
        with self.frames.open('ab') as output:
            output.write(data)

    def snapshot(self):
        return truth.snapshot_live(self.source, self.run, self.initial)

    def rejected(self, pattern):
        with self.assertRaisesRegex((ValueError, csv.Error), pattern):
            self.snapshot()
        self.assertFalse((self.run/'truth').exists())
        self.assertEqual(list(self.run.iterdir()), [])

    def test_boundaries_and_own_summary_preserve_source(self):
        self.assertEqual(self.initial['start_byte'], self.frames.stat().st_size)
        self.assertEqual(self.initial['last_frame'], 10)
        self.assertEqual(self.initial['last_time_s'], 1.0)
        self.assertEqual(self.initial['manifest_sha256'], hashlib.sha256(
            (self.source/'manifest.json').read_bytes()).hexdigest())
        self.append()
        originals = {p.name: p.read_bytes() for p in self.source.iterdir()}
        snapshot = self.snapshot()
        target = self.run/'truth'
        self.assertEqual((target/'frames.csv').read_bytes(),
                         self.header + b'11,1.1,new\r\n12,1.2,new\r\n')
        summary = json.loads((target/'summary.json').read_text())
        self.assertEqual(summary['stop_reason'], 'runner_snapshot')
        self.assertEqual((summary['first_frame'], summary['last_frame'], summary['rows']), (11, 12, 2))
        self.assertEqual(json.loads((target/'snapshot.json').read_text()), snapshot)
        self.assertEqual(originals, {p.name: p.read_bytes() for p in self.source.iterdir()})
        for name in ('manifest.json', 'datarefs.json', 'capture_config.ini'):
            self.assertEqual((target/name).read_bytes(), originals[name])
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.snapshot()

    def test_appending_during_copy_is_allowed_and_cutoff_is_fixed(self):
        self.append()
        original_hash = truth._range_hash

        def append_when_copying(stream, start, end, output=None):
            if output is not None:
                self.append(b'13,1.3,later\r\n14,1.')
            return original_hash(stream, start, end, output)

        with patch.object(truth, '_range_hash', side_effect=append_when_copying):
            result = self.snapshot()
        self.assertEqual(result['last_frame'], 12)
        self.assertTrue(self.frames.read_bytes().endswith(b'14,1.'))
        self.assertNotIn(b'later', (self.run/'truth/frames.csv').read_bytes())

    def test_partial_tail_is_omitted_and_can_complete_after_preflight(self):
        self.append(b'11,1.')
        self.initial = truth.begin_snapshot(self.source)
        self.assertEqual(self.initial['last_frame'], 10)
        self.append(b'1,new\r\n12,1.2,incomplete')
        result = self.snapshot()
        self.assertEqual(result['last_frame'], 11)
        self.assertEqual(result['omitted_partial_bytes'], len(b'12,1.2,incomplete'))
        self.assertTrue((self.run/'truth/frames.csv').read_bytes().endswith(b'11,1.1,new\r\n'))

    def test_no_new_complete_rows(self):
        self.rejected('No new complete rows')
        self.append(b'11,1.1,partial')
        self.rejected('No new complete rows')

    def test_no_preflight_data_rows(self):
        self.frames.write_bytes(self.header)
        with self.assertRaisesRegex(ValueError, 'No complete data rows'):
            truth.begin_snapshot(self.source)

    def test_truncation(self):
        self.frames.write_bytes(self.header)
        self.rejected('truncated')

    def test_same_size_preflight_rewrite(self):
        self.frames.write_bytes(self.frames.read_bytes().replace(b'old', b'BAD'))
        self.append()
        self.rejected('rewritten')

    def test_file_replacement(self):
        replacement = self.source/'replacement.csv'
        replacement.write_bytes(self.frames.read_bytes())
        replacement.replace(self.frames)
        self.append()
        self.rejected('identity')

    def test_identity_device_and_inode_checked(self):
        self.append()
        for index in (0, 1):
            with self.subTest(index=index):
                self.initial['identity'][index] += 1
                self.rejected('identity')
                self.initial['identity'][index] -= 1

    def test_manifest_or_configuration_changed(self):
        self.append()
        for name in ('manifest.json', 'datarefs.json', 'capture_config.ini'):
            file = self.source/name
            original = file.read_bytes()
            file.write_bytes(original + b' ')
            self.rejected('manifest or session metadata changed')
            file.write_bytes(original)

    def test_metadata_without_optional_files(self):
        (self.source/'datarefs.json').unlink()
        (self.source/'capture_config.ini').unlink()
        self.initial = truth.begin_snapshot(self.source)
        self.append()
        self.snapshot()
        self.assertFalse((self.run/'truth/capture_config.ini').exists())

    def test_strict_frame_and_time_and_row_validation(self):
        for row in (b'10,1.1,x\n', b'9,1.1,x\n', b'11,1.0,x\n',
                    b'11,0.9,x\n', b'11,nan,x\n', b'11,inf,x\n',
                    b'11,1.1\n', b'11.5,1.1,x\n', b'\n',
                    b'11,1.1,x\n11,1.2,x\n', b'11,1.1,x\n12,1.1,x\n',
                    b'11,1.1,"unterminated\n'):
            with self.subTest(row=row):
                self.frames.write_bytes(self.header + b'10,1.0,old\r\n' + row)
                self.rejected('monotonic|Invalid|malformed|unexpected end')

    def test_alternate_time_column_and_bom(self):
        self.frames.write_bytes(b'\xef\xbb\xbfframe_id,sim/time/total_flight_time_sec\n1,1\n')
        self.initial = truth.begin_snapshot(self.source)
        self.append(b'2,2\n')
        self.assertEqual(self.snapshot()['last_frame'], 2)
        self.assertTrue((self.run/'truth/frames.csv').read_bytes().startswith(b'\xef\xbb\xbf'))

    def test_low_disk_space(self):
        self.append()
        usage = shutil.disk_usage(self.run)
        with patch.object(truth.shutil, 'disk_usage', return_value=usage._replace(free=1)):
            self.rejected('Insufficient space')

    def test_window_limit_before_copy(self):
        self.append()
        with patch.object(truth, 'MAX_WINDOW', 1):
            self.rejected('256 MiB')

    def test_mutation_during_copy(self):
        self.append()
        original_hash = truth._range_hash

        def mutate(stream, start, end, output=None):
            result = original_hash(stream, start, end, output)
            if output is not None:
                self.frames.write_bytes(self.frames.read_bytes().replace(b'new', b'BAD'))
            return result

        with patch.object(truth, '_range_hash', side_effect=mutate):
            self.rejected('rewritten')

    def test_changes_during_copy_rejected_and_temporary_cleaned(self):
        for change in ('truncate', 'manifest', 'replace'):
            with self.subTest(change=change):
                self.frames.write_bytes(self.header + b'10,1.0,old\r\n')
                (self.source/'manifest.json').write_text('{"session":"test"}')
                self.initial = truth.begin_snapshot(self.source)
                self.append()
                original_hash = truth._range_hash

                def mutate(stream, start, end, output=None):
                    result = original_hash(stream, start, end, output)
                    if output is not None:
                        if change == 'truncate':
                            self.frames.write_bytes(self.header)
                        elif change == 'manifest':
                            (self.source/'manifest.json').write_text('{"session":"other"}')
                        else:
                            replacement = self.source/'new.csv'
                            replacement.write_bytes(self.frames.read_bytes())
                            replacement.replace(self.frames)
                    return result

                with patch.object(truth, '_range_hash', side_effect=mutate):
                    self.rejected('truncated|metadata changed|identity')

    def test_symlinks_rejected(self):
        original = self.frames.read_bytes()
        self.frames.unlink()
        other = self.root/'other.csv'
        other.write_bytes(original)
        self.frames.symlink_to(other)
        with self.assertRaisesRegex(ValueError, 'regular'):
            truth.begin_snapshot(self.source)

    def test_source_binding(self):
        self.append()
        self.initial['source'] = str(self.root/'different')
        self.rejected('different capture source')

    def test_bounded_tail_and_window_exclude_old_history(self):
        self.frames.write_bytes(self.header + b'1,0,history\n' * 1000 + b'10,1.0,old\n')
        with patch.object(truth, 'TAIL_BYTES', 64), patch.object(truth, 'MAX_WINDOW', 64):
            self.initial = truth.begin_snapshot(self.source)
            self.assertGreater(self.initial['anchor_start'], len(self.header))
            self.assertLessEqual(self.initial['start_byte'] - self.initial['anchor_start'], 64)
            self.append()
            result = self.snapshot()
        self.assertEqual(result['rows'], 2)
        self.assertNotIn(b'history', (self.run/'truth/frames.csv').read_bytes())

    def test_header_rewrite(self):
        self.frames.write_bytes(self.frames.read_bytes().replace(b'value', b'other'))
        self.append()
        self.rejected('rewritten')

    def test_invalid_headers(self):
        for header in (b'frame_id,value\n', b'frame_id,sim_time_s,frame_id\n',
                       b'value,sim_time_s\n', b'frame_id,sim_time_s'):
            with self.subTest(header=header):
                self.frames.write_bytes(header)
                with self.assertRaises(ValueError):
                    truth.begin_snapshot(self.source)

    def test_cli_persists_preflight_and_publishes_snapshot(self):
        script = Path(truth.__file__).resolve()
        initial_file = self.run/'truth-live-preflight.json'
        begin = [sys.executable, str(script), 'begin-snapshot', str(self.source), str(initial_file)]
        subprocess.run(begin, check=True, capture_output=True, text=True)
        initial_bytes = initial_file.read_bytes()
        self.assertEqual(json.loads(initial_bytes)['last_frame'], 10)
        retry = subprocess.run(begin, capture_output=True, text=True)
        self.assertNotEqual(retry.returncode, 0)
        self.assertEqual(initial_file.read_bytes(), initial_bytes)
        self.append()
        subprocess.run([sys.executable, str(script), 'snapshot-live', str(self.source),
                        str(self.run), str(initial_file)], check=True, capture_output=True, text=True)
        self.assertTrue((self.run/'truth/frames.csv').is_file())


if __name__ == '__main__':
    unittest.main()
