#!/usr/bin/env python3
"""Host-only live snapshot tests; large source windows are generated in RAM."""
import csv
import errno
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import attach_truth as truth
from collect import collect


class DiagnosticCatalogTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.state = root/'state'
        self.state.mkdir()
        self.target = root/'X-Plane 12/Resources/plugins/XPlaneTruthCapture/config/datarefs.txt'
        self.target.parent.mkdir(parents=True)
        (self.state/'receipt.json').write_text(json.dumps({
            'aircraft': str(root/'X-Plane 12/Aircraft/QuadTailsitterRef6')}))

    def test_configure_preserves_bytes_idempotent_and_restore(self):
        original = b'\xef\xbb\xbf# user refs\r\nsim/test/user|custom|false\r\n'
        self.target.write_bytes(original)
        truth.configure_diagnostics(self.state)
        installed = self.target.read_bytes()
        self.assertEqual(installed, original+truth.TRUTH_BLOCK)
        truth.configure_diagnostics(self.state)
        self.assertEqual(self.target.read_bytes(), installed)
        truth.configure_diagnostics(self.state, restore=True)
        self.assertEqual(self.target.read_bytes(), original)

    def test_new_custom_file_is_removed_on_restore(self):
        truth.configure_diagnostics(self.state)
        truth.configure_diagnostics(self.state, restore=True)
        self.assertFalse(self.target.exists())

    def test_user_edits_and_symlinks_are_not_overwritten(self):
        truth.configure_diagnostics(self.state)
        self.target.write_bytes(self.target.read_bytes()+b'# later user edit\n')
        for restore in (False, True):
            with self.assertRaisesRegex(ValueError, 'edited'):
                truth.configure_diagnostics(self.state, restore=restore)
        self.target.unlink()
        other = self.target.parent/'other'
        other.write_text('unrelated')
        self.target.symlink_to(other)
        with self.assertRaisesRegex(ValueError, 'regular path'):
            truth.configure_diagnostics(self.state, restore=True)
        self.assertEqual(other.read_text(), 'unrelated')

    def test_crash_after_receipt_can_resume(self):
        from ref6 import atomic
        def fail_target(file, data):
            if file == self.target:
                raise OSError('simulated write failure')
            atomic(file, data)
        with patch('ref6.atomic', side_effect=fail_target):
            with self.assertRaises(OSError):
                truth.configure_diagnostics(self.state)
        truth.configure_diagnostics(self.state)
        self.assertEqual(self.target.read_bytes(), truth.TRUTH_BLOCK)

    def test_symlink_journal_cannot_restore_another_catalog(self):
        truth.configure_diagnostics(self.state)
        journal = self.state/'truth-instrumentation.json'
        other_state = self.state.parent/'other-state'
        other_state.mkdir()
        link = other_state/journal.name
        link.symlink_to(journal)
        original_journal = journal.read_bytes()
        with self.assertRaisesRegex(ValueError, 'symlink'):
            truth.configure_diagnostics(other_state, restore=True)
        self.assertEqual(self.target.read_bytes(), truth.TRUTH_BLOCK)
        self.assertEqual(journal.read_bytes(), original_journal)
        self.assertTrue(link.is_symlink())

    def test_resumed_prepare_rejects_restored_install_but_allows_truth_restore(self):
        original = b'# user catalog\n'
        self.target.write_bytes(original)
        truth.configure_diagnostics(self.state)
        # A truth restore can finish its write but leave the journal after a crash.
        self.target.write_bytes(original)
        receipt_file = self.state/'receipt.json'
        receipt = json.loads(receipt_file.read_text())
        receipt['restored'] = True
        receipt_file.write_text(json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, 'installation has been restored'):
            truth.configure_diagnostics(self.state)
        self.assertEqual(self.target.read_bytes(), original)
        truth.configure_diagnostics(self.state, restore=True)
        self.assertEqual(self.target.read_bytes(), original)

    def test_runtime_availability_does_not_synthesize_missing_values(self):
        session = Path(self.temp.name)/'session'
        session.mkdir()
        (session/'datarefs.csv').write_text('path,exists\n'+''.join(
            f'{ref},{"false" if ref.endswith("L_prop") else "true"}\n' for ref in truth.DIAGNOSTIC_REFS))
        status = truth.diagnostic_availability(session)
        self.assertEqual(status['missing'], ['sim/flightmodel/forces/L_prop'])


class GeneratedWindow(io.RawIOBase):
    """Seekable, fixed-width CSV source without storing hundreds of MiB."""

    def __init__(self, prefix, row_size, count, descriptor):
        self.prefix, self.row_size, self.count = prefix, row_size, count
        self.descriptor, self.position = descriptor, 0
        self.size = len(prefix) + row_size * count

    def fileno(self):
        return self.descriptor

    def row(self, index):
        frame = index + 11
        point = f'{frame:08d},{frame:08d},'.encode()
        return point + b'x' * (self.row_size - len(point) - 1) + b'\n'

    def seek(self, offset, whence=os.SEEK_SET):
        base = (0, self.position, self.size)[whence]
        self.position = base + offset
        return self.position

    def read(self, size=-1):
        end = self.size if size < 0 else min(self.size, self.position + size)
        chunks = []
        if self.position < len(self.prefix):
            stop = min(end, len(self.prefix))
            chunks.append(self.prefix[self.position:stop])
            self.position = stop
        while self.position < end:
            index, offset = divmod(self.position - len(self.prefix), self.row_size)
            stop = min(self.row_size, offset + end - self.position)
            chunks.append(self.row(index)[offset:stop])
            self.position += stop - offset
        return b''.join(chunks)

    def readline(self, size=-1):
        start = self.position
        data = self.read(size)
        newline = data.find(b'\n')
        if newline >= 0:
            data = data[:newline + 1]
            self.seek(start + len(data))
        return data


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

    def assert_artifact(self, result, expected, compressed):
        target = self.run/'truth'
        name = 'frames.csv.gz' if compressed else 'frames.csv'
        artifact = (target/name).read_bytes()
        self.assertEqual(gzip.decompress(artifact) if compressed else artifact, expected)
        self.assertFalse((target/('frames.csv' if compressed else 'frames.csv.gz')).exists())
        self.assertEqual(result['frames_file'], name)
        self.assertEqual(result['frames_compression'], 'gzip' if compressed else 'none')
        self.assertEqual(result['frames_bytes'], len(artifact))
        self.assertEqual(result['frames_raw_bytes'], len(expected))
        self.assertEqual(result['frames_sha256'], hashlib.sha256(artifact).hexdigest())
        self.assertEqual(result['frames_raw_sha256'], hashlib.sha256(expected).hexdigest())
        self.assertEqual(result['window_sha256'], hashlib.sha256(expected[len(self.header):]).hexdigest())
        self.assertEqual(json.loads((target/'snapshot.json').read_text()), result)

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
        self.assert_artifact(snapshot, self.header + b'11,1.1,new\r\n12,1.2,new\r\n', False)
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

    def test_input_limit_before_copy(self):
        self.append()
        window = self.frames.stat().st_size - self.initial['start_byte']
        with patch.object(truth, 'MAX_INPUT_WINDOW', window - 1), \
             patch.object(truth.tempfile, 'mkdtemp') as temporary:
            self.rejected('bounded input limit.*no rows truncated')
        temporary.assert_not_called()

    def test_complete_window_threshold_and_exact_input_limit(self):
        data = b'11,1.1,new\r\n12,1.2,new\r\n'
        self.append(data + b'13,1.')
        with patch.object(truth, 'MAX_WINDOW', len(data)), \
             patch.object(truth, 'MAX_INPUT_WINDOW', len(data) + 5):
            result = self.snapshot()
        self.assert_artifact(result, self.header + data, False)
        self.assertEqual(result['omitted_partial_bytes'], 5)

    def test_gzip_complete_csv_metadata_and_no_raw_duplicate(self):
        data = b'11,1.1,"quoted,\r\nvalue"\r\n12,1.2,\xc3\xa9\r\n'
        self.append(data + b'13,1.3,partial')
        originals = {p.name: p.read_bytes() for p in self.source.iterdir()}
        with patch.object(truth, 'MAX_WINDOW', len(data) - 1):
            result = self.snapshot()
        self.assert_artifact(result, self.header + data, True)
        self.assertEqual((result['rows'], result['first_frame'], result['last_frame']), (2, 11, 12))
        self.assertEqual(result['omitted_partial_bytes'], len(b'13,1.3,partial'))
        self.assertEqual(originals, {p.name: p.read_bytes() for p in self.source.iterdir()})
        self.assertEqual({p.name for p in (self.run/'truth').iterdir()},
                         {'frames.csv.gz', 'manifest.json', 'datarefs.json',
                          'capture_config.ini', 'summary.json', 'snapshot.json'})
        # Gzip stores no temporary basename or timestamp.
        packed = (self.run/'truth/frames.csv.gz').read_bytes()
        self.assertEqual(packed[3] & 8, 0)
        self.assertEqual(packed[4:8], b'\0' * 4)

    def test_default_threshold_over_256_mib_without_large_fixture(self):
        self.assertEqual(truth.MAX_WINDOW, 256 * 1024 * 1024)
        self.assertEqual(truth.MAX_INPUT_WINDOW, 1024 * 1024 * 1024)
        self.assertEqual(truth.MAX_COMPRESSED, 128 * 1024 * 1024)
        row_size = 64 * 1024
        count = truth.MAX_WINDOW // row_size + 1
        prefix = self.frames.read_bytes()
        real_open, real_stat, real_fstat = Path.open, Path.stat, os.fstat
        with real_open(self.frames, 'rb') as descriptor:
            generated = GeneratedWindow(prefix, row_size, count, descriptor.fileno())
            stat_fields = list(real_stat(self.frames))
            stat_fields[6] = generated.size
            virtual_stat = os.stat_result(stat_fields)

            def open_source(file, *args, **kwargs):
                if file == self.frames and args and args[0] == 'rb':
                    return generated
                return real_open(file, *args, **kwargs)

            def stat_source(file, *args, **kwargs):
                return virtual_stat if file == self.frames else real_stat(file, *args, **kwargs)

            def fstat_source(fd):
                return virtual_stat if fd == descriptor.fileno() else real_fstat(fd)

            with patch.object(Path, 'open', open_source), \
                 patch.object(Path, 'stat', stat_source), \
                 patch.object(truth.os, 'fstat', fstat_source):
                result = self.snapshot()
        self.assertGreater(result['end_byte'] - result['start_byte'], truth.MAX_WINDOW)
        self.assertEqual(result['frames_file'], 'frames.csv.gz')
        self.assertEqual((result['rows'], result['first_frame'], result['last_frame']),
                         (count, 11, count + 10))
        raw_hash, window_hash = hashlib.sha256(self.header), hashlib.sha256()
        with gzip.open(self.run/'truth/frames.csv.gz', 'rb') as output:
            self.assertEqual(output.readline(), self.header)
            for index in range(count):
                expected = generated.row(index)
                self.assertEqual(output.readline(), expected)
                raw_hash.update(expected)
                window_hash.update(expected)
            self.assertEqual(output.read(1), b'')
        self.assertEqual(result['window_sha256'], window_hash.hexdigest())
        self.assertEqual(result['frames_raw_sha256'], raw_hash.hexdigest())
        self.assertEqual(result['frames_raw_bytes'], len(self.header) + count * row_size)
        self.assertLess(result['frames_bytes'], 2 * 1024 * 1024)
        self.assertFalse((self.run/'truth/frames.csv').exists())
        self.assertFalse(list(self.run.glob('*.partial')))

    def test_gzip_compressed_cap_includes_header_and_trailer(self):
        self.append()
        expected = self.header + b'11,1.1,new\r\n12,1.2,new\r\n'
        packed = io.BytesIO()
        with gzip.GzipFile(filename='', mode='wb', fileobj=packed, mtime=0) as output:
            output.write(expected)
        size = len(packed.getvalue())
        with patch.object(truth, 'MAX_WINDOW', 1):
            for limit in (0, 1, 9, 10, size - 8, size - 1):
                with self.subTest(limit=limit), patch.object(truth, 'MAX_COMPRESSED', limit):
                    self.rejected('Compressed truth exceeds.*no rows truncated')
            with patch.object(truth, 'MAX_COMPRESSED', size):
                result = self.snapshot()
        self.assertEqual(result['frames_bytes'], size)
        self.assert_artifact(result, expected, True)

    def test_raw_valid_wide_row_keeps_existing_behavior(self):
        self.header = b'frame_id,sim_time_s,' + b','.join(
            f'value{i}'.encode() for i in range(12)) + b'\n'
        self.frames.write_bytes(self.header + b'10,1.0,' + b','.join([b'old'] * 12) + b'\n')
        self.initial = truth.begin_snapshot(self.source)
        data = b'11,1.1,' + b','.join([b'x' * 100000] * 12) + b'\n'
        self.assertGreater(len(data), truth.MAX_ROW_BYTES)
        self.append(data)
        result = self.snapshot()
        self.assert_artifact(result, self.header + data, False)
        self.assertEqual(result['rows'], 1)

    def test_gzip_logical_row_limit_counts_quoted_multiline_bytes(self):
        self.append(b'11,1.1,"' + b'x' * 30 + b'\n' + b'x' * 30 + b'"\n')
        with patch.object(truth, 'MAX_WINDOW', 1), patch.object(truth, 'MAX_ROW_BYTES', 64):
            self.rejected('row exceeds bounded parsing limit.*no rows truncated')

    def test_gzip_preserves_bom_and_alternate_time_column(self):
        self.header = b'\xef\xbb\xbfframe_id,sim/time/total_flight_time_sec\r\n'
        self.frames.write_bytes(self.header + b'10,1.0\r\n')
        self.initial = truth.begin_snapshot(self.source)
        data = b'11,1.1\r\n12,1.2\r\n'
        self.append(data)
        with patch.object(truth, 'MAX_WINDOW', 1):
            result = self.snapshot()
        self.assert_artifact(result, self.header + data, True)

    def test_mixed_csv_newlines_preserve_raw_bytes_in_both_modes(self):
        data = b'11,1.1,"quoted\rvalue"\r12,1.2,new\n'
        self.append(data)
        for compressed in (False, True):
            with self.subTest(compressed=compressed), \
                 patch.object(truth, 'MAX_WINDOW', 1 if compressed else truth.MAX_WINDOW):
                result = self.snapshot()
                self.assert_artifact(result, self.header + data, compressed)
                self.assertEqual(result['rows'], 2)
                shutil.rmtree(self.run/'truth')

    def test_gzip_append_during_copy_preserves_fixed_cutoff(self):
        self.append()
        original_hash = truth._range_hash

        def grow(stream, start, end, output=None):
            if output is not None:
                self.append(b'13,1.3,later\r\n14,1.')
            return original_hash(stream, start, end, output)

        with patch.object(truth, 'MAX_WINDOW', 1), \
             patch.object(truth, '_range_hash', side_effect=grow):
            result = self.snapshot()
        self.assert_artifact(result, self.header + b'11,1.1,new\r\n12,1.2,new\r\n', True)
        self.assertEqual(result['rows'], 2)

    def test_gzip_resets_and_malformed_rows_fail_without_decimation(self):
        for row in (b'10,1.3,reset\n', b'13,1.2,reset\n', b'13,nan,x\n',
                    b'13,1.3\n', b'13,1.3,"unterminated\n'):
            with self.subTest(row=row):
                self.frames.write_bytes(self.header + b'10,1.0,old\r\n')
                self.append(b'11,1.1,new\n12,1.2,new\n' + row)
                with patch.object(truth, 'MAX_WINDOW', 1):
                    self.rejected('monotonic|Invalid|malformed|unexpected end')

    def test_gzip_mutations_fail_and_remove_partial_artifacts(self):
        for change in ('prefix', 'window', 'truncate', 'manifest', 'replace', 'header'):
            with self.subTest(change=change):
                self.frames.write_bytes(self.header + b'10,1.0,old\r\n')
                (self.source/'manifest.json').write_text('{"session":"test"}')
                self.initial = truth.begin_snapshot(self.source)
                self.append()
                original_hash = truth._range_hash

                def mutate(stream, start, end, output=None):
                    result = original_hash(stream, start, end, output)
                    if output is not None:
                        if change in ('prefix', 'window', 'header'):
                            old, new = {'prefix': (b'old', b'BAD'), 'window': (b'new', b'BAD'),
                                        'header': (b'value', b'other')}[change]
                            self.frames.write_bytes(self.frames.read_bytes().replace(old, new))
                        elif change == 'truncate':
                            self.frames.write_bytes(self.header)
                        elif change == 'manifest':
                            (self.source/'manifest.json').write_text('{"session":"other"}')
                        else:
                            replacement = self.source/'replacement.csv'
                            replacement.write_bytes(self.frames.read_bytes())
                            replacement.replace(self.frames)
                    return result

                with patch.object(truth, 'MAX_WINDOW', 1), \
                     patch.object(truth, '_range_hash', side_effect=mutate):
                    self.rejected('rewritten|truncated|metadata changed|identity')

    def test_copied_bytes_are_verified_independently_of_source_hash(self):
        self.append()
        original_hash = truth._range_hash

        class CorruptOutput:
            def __init__(self, output):
                self.output = output

            def write(self, data):
                return self.output.write(data.replace(b'new', b'BAD'))

        def corrupt(stream, start, end, output=None):
            return original_hash(stream, start, end, CorruptOutput(output) if output is not None else None)

        for compressed in (False, True):
            with self.subTest(compressed=compressed), \
                 patch.object(truth, 'MAX_WINDOW', 1 if compressed else truth.MAX_WINDOW), \
                 patch.object(truth, '_range_hash', side_effect=corrupt):
                self.rejected('Copied capture prefix was rewritten')
        self.assertNotIn(b'BAD', self.frames.read_bytes())

    def test_late_prefix_or_window_rewrite_prevents_publication(self):
        original_write = Path.write_text
        for change in ('prefix', 'window'):
            with self.subTest(change=change):
                self.frames.write_bytes(self.header + b'10,1.0,old\r\n')
                self.initial = truth.begin_snapshot(self.source)
                self.append()

                def rewrite(file, *args, **kwargs):
                    result = original_write(file, *args, **kwargs)
                    if file.name == 'snapshot.json':
                        old = b'old' if change == 'prefix' else b'new'
                        self.frames.write_bytes(self.frames.read_bytes().replace(old, b'BAD'))
                    return result

                with patch.object(truth, 'MAX_WINDOW', 1), patch.object(Path, 'write_text', rewrite):
                    self.rejected('rewritten')

    def test_gzip_corrupt_trailer_never_publishes(self):
        self.append()
        original_validate = truth._validate_copy

        def corrupt(stream, *args, **kwargs):
            artifact = next(self.run.glob('*/frames.csv.gz'))
            packed = bytearray(artifact.read_bytes())
            packed[-8] ^= 1
            artifact.write_bytes(packed)
            return original_validate(stream, *args, **kwargs)

        with patch.object(truth, 'MAX_WINDOW', 1), \
             patch.object(truth, '_validate_copy', side_effect=corrupt):
            with self.assertRaisesRegex(gzip.BadGzipFile, 'CRC check failed'):
                self.snapshot()
        self.assertEqual(list(self.run.iterdir()), [])

    def test_late_existing_target_is_preserved_and_partial_cleaned(self):
        self.append()
        original_space = truth._require_space
        target = self.run/'truth'

        def create_target(run, needed, detail):
            original_space(run, needed, detail)
            if list(self.run.glob('*/snapshot.json')):
                target.mkdir()
                (target/'keep.txt').write_text('existing evidence')

        with patch.object(truth, 'MAX_WINDOW', 1), \
             patch.object(truth, '_require_space', side_effect=create_target):
            with self.assertRaisesRegex(ValueError, 'refusing to overwrite'):
                self.snapshot()
        self.assertEqual(list(self.run.iterdir()), [target])
        self.assertEqual((target/'keep.txt').read_text(), 'existing evidence')

    def test_gzip_space_preflight_reserves_existing_run_and_zip(self):
        self.append()
        usage = shutil.disk_usage(self.run)
        seen = []

        def record(run, needed, detail):
            seen.append(needed)

        with patch.object(truth, 'MAX_WINDOW', 1), \
             patch.object(truth, '_require_space', side_effect=record):
            self.snapshot()
        self.assertGreater(seen[0], 2 * truth.MAX_COMPRESSED + truth.SPACE_HEADROOM)
        shutil.rmtree(self.run/'truth')
        existing = self.run/'existing.ulg'
        existing.write_bytes(b'x' * 1024)
        budget = truth._zip_budget(self.run)
        with patch.object(truth, 'MAX_WINDOW', 1), \
             patch.object(truth.shutil, 'disk_usage', return_value=usage._replace(free=seen[0] + budget - 1)), \
             patch.object(truth.tempfile, 'mkdtemp') as temporary:
            with self.assertRaisesRegex(ValueError, 'Insufficient space.*existing run evidence'):
                self.snapshot()
        temporary.assert_not_called()
        self.assertEqual(list(self.run.iterdir()), [existing])

    def test_gzip_disk_exhaustion_during_write_or_before_publish(self):
        self.append()
        usage = shutil.disk_usage(self.run)
        for phase in ('write', 'publish'):
            with self.subTest(phase=phase):
                calls = 0

                def disk_usage(run):
                    nonlocal calls
                    calls += 1
                    low = calls > 1 if phase == 'write' else bool(list(self.run.glob('*/snapshot.json')))
                    return usage._replace(free=1 if low else 10**12)

                with patch.object(truth, 'MAX_WINDOW', 1), \
                     patch.object(truth.shutil, 'disk_usage', side_effect=disk_usage):
                    self.rejected('Insufficient space')
                self.assertGreater(calls, 1)

    def test_enospc_during_copy_and_metadata_cleans_atomic_stage(self):
        self.append()
        for compressed in (False, True):
            for phase in ('copy', 'metadata'):
                with self.subTest(compressed=compressed, phase=phase):
                    original_hash, original_write = truth._range_hash, Path.write_text

                    def fail_copy(stream, start, end, output=None):
                        if output is not None:
                            raise OSError(errno.ENOSPC, 'fixture full')
                        return original_hash(stream, start, end, output)

                    def fail_metadata(file, *args, **kwargs):
                        if file.name == 'snapshot.json':
                            raise OSError(errno.ENOSPC, 'fixture full')
                        return original_write(file, *args, **kwargs)

                    with patch.object(truth, 'MAX_WINDOW', 1 if compressed else truth.MAX_WINDOW), \
                         patch.object(truth, '_range_hash', side_effect=fail_copy if phase == 'copy' else original_hash), \
                         patch.object(Path, 'write_text', fail_metadata if phase == 'metadata' else original_write):
                        self.rejected('Insufficient space.*No evidence published')

    def test_gzip_collection_keeps_single_stored_attachment_and_hash(self):
        self.append()
        with patch.object(truth, 'MAX_WINDOW', 1):
            result = self.snapshot()
        (self.run/'truth-source.txt').write_text(str(self.source))
        with patch('sys.stdout', new_callable=io.StringIO) as output:
            archive = collect(self.run)
        self.assertNotIn('MISSING', output.getvalue())
        self.assertTrue(json.loads((self.run/'collection.json').read_text())['truth_attached'])
        with zipfile.ZipFile(archive) as bundle:
            name = f'{self.run.name}/truth/{result["frames_file"]}'
            self.assertEqual(bundle.getinfo(name).compress_type, zipfile.ZIP_STORED)
            self.assertNotIn(f'{self.run.name}/truth/frames.csv', bundle.namelist())
            packed = bundle.read(name)
            self.assertEqual(hashlib.sha256(packed).hexdigest(), result['frames_sha256'])
            sums = bundle.read(f'{self.run.name}/SHA256SUMS').decode()
            self.assertIn(f'{result["frames_sha256"]}  truth/frames.csv.gz\n', sums)
            self.assertEqual(gzip.decompress(packed), self.header + b'11,1.1,new\r\n12,1.2,new\r\n')
        self.assertFalse(list(self.root.glob('.collect-*.partial')))

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
