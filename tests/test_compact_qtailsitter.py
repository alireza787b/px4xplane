"""Offline launcher tests: no aircraft distribution, PX4 build, flight, or network."""

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import run_compact_qtailsitter as runner


REPO = Path(__file__).resolve().parents[1]
PROFILE = REPO / "config/px4_params" / runner.PROFILE_NAME
GCS_LINE = "mavlink start -x -u $udp_gcs_port_local -r 4000000 -f $mavlink_network_interface_arg\n"
# Keep the real rcS anchors and reset branch in the small standalone fixture.
RCS = """#!/bin/sh
. px4-alias.sh
# Load parameters
set PARAM_FILE fs/parameters.bson
set PARAM_BACKUP_FILE fs/parameters_backup.bson
param select $PARAM_FILE
param import
# exit early when the minimal shell is requested
if param compare SYS_AUTOSTART 5021; then
  AUTOCNF=no
else
  AUTOCNF=yes
  param set SYS_AUTOCONFIG 1
fi
if param greater SYS_AUTOCONFIG 0; then
  param reset_all SYS_AUTOSTART CAL_*
fi

dataman start
commander start
. px4-rc.mavlink
"""


def config_text():
    return ("config_name = QuadTailsitter\n[QuadTailsitter]\n"
            "aircraftMatch = QuadTailsitter,PB50\nairspeedSource = body_axis\n"
            "pitotAxisBody = -Z\nactuatorSmoothingTimeConstantSec = 0.0\n"
            + "".join(f"channel{i} = sim/flightmodel/engine/ENGN_thro_use, floatArray, [{i}], [-1 1]\n"
                      for i in range(4)))


class ProfileTests(unittest.TestCase):
    def test_real_profile_has_exactly_211_literal_defaults(self):
        values = runner.parse_profile(PROFILE.read_bytes())
        self.assertEqual(len(values), 211)
        self.assertEqual(values["CAL_BARO0_ID"], "6620172")
        self.assertEqual(values["CA_ROTOR0_CT"], "6.5")
        self.assertEqual(values["FW_R_TC"], "1.5")
        self.assertNotIn("COM_FLIGHT_UUID", values)
        self.assertNotIn("CAL_ACC0_ID", values)

    def test_profile_ignores_comments_but_never_executes_inheritance(self):
        data = PROFILE.read_bytes() + b"\n# a comment\n"
        self.assertEqual(runner.parse_profile(data), runner.parse_profile(PROFILE.read_bytes()))

    def test_rejects_shell_injection_and_non_numeric_values(self):
        original = PROFILE.read_bytes()
        for value in (b"$(touch /tmp/unsafe)", b"1;id", b"NaN", b"inf", b"1e999", b"`id`", b"1 || id"):
            with self.subTest(value=value), self.assertRaises(runner.RunnerError):
                runner.parse_profile(original.replace(b"MAV_TYPE 20", b"MAV_TYPE " + value))

    def test_rejects_executable_lines_and_wrong_inheritance(self):
        for extra in (b"\nparam set MAV_TYPE 20", b"\necho unsafe", b"\n. /tmp/profile.sh"):
            with self.subTest(extra=extra), self.assertRaises(runner.RunnerError):
                runner.parse_profile(PROFILE.read_bytes() + extra)

    def test_duplicate_missing_and_malformed_defaults_fail(self):
        original = PROFILE.read_bytes()
        for data in (original + b"\nparam set-default MAV_TYPE 20\n",
                     original.replace(b"param set-default MAV_TYPE 20", b""),
                     original + b"\nparam set-default BROKEN 'unterminated"):
            with self.subTest(data=data[-70:]), self.assertRaises(runner.RunnerError):
                runner.parse_profile(data)

    def test_repository_default_profile(self):
        self.assertEqual(runner.find_profile(), PROFILE)

    def test_sibling_bundle_profile_and_explicit_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            here = Path(tmp).resolve()
            sibling = here / runner.PROFILE_NAME
            sibling.write_bytes(PROFILE.read_bytes())
            self.assertEqual(runner.find_profile(script=here / "run_compact_qtailsitter.py"), sibling)
            explicit = here / "custom-profile"
            explicit.write_bytes(PROFILE.read_bytes())
            self.assertEqual(runner.find_profile(str(explicit)), explicit)
            with self.assertRaises(runner.RunnerError):
                runner.find_profile(str(here / "missing"))


class ConfigTests(unittest.TestCase):
    def test_headerless_plugin_config_uses_virtual_global_section(self):
        runner.validate_config(config_text().encode())

    def test_wrong_global_selection_rejected_even_with_correct_section(self):
        with self.assertRaisesRegex(runner.RunnerError, "MUST"):
            runner.validate_config(config_text().replace("config_name = QuadTailsitter", "config_name = Alia250").encode())

    def test_axes_smoothing_motor_mapping_and_extra_channels_fail_closed(self):
        for old, new in (("body_axis", "xplane_indicated"), ("-Z", "+X"),
                         ("Sec = 0.0", "Sec = 0.1"), ("Sec = 0.0", "Sec = nan"),
                         ("[2], [-1 1]", "[3], [-1 1]"), ("[-1 1]", "[0 1]"),
                         ("floatArray", "float"), ("aircraftMatch = QuadTailsitter,PB50", "aircraftMatch = Cessna")):
            with self.subTest(new=new), self.assertRaises(runner.RunnerError):
                runner.validate_config(config_text().replace(old, new).encode())
        with self.assertRaises(runner.RunnerError):
            runner.validate_config((config_text() + "channel4 = bad\n").encode())

    def test_missing_smoothing_means_disabled(self):
        runner.validate_config(config_text().replace("actuatorSmoothingTimeConstantSec = 0.0\n", "").encode())

    def test_malformed_or_duplicate_ini_rejected(self):
        for text in ("malformed", config_text() + "channel0=duplicate\n"):
            with self.subTest(text=text), self.assertRaises(runner.RunnerError):
                runner.validate_config(text.encode())


class LogTests(unittest.TestCase):
    def inspect(self, text, root=Path("/mnt/h/X-Plane 12")):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "Log.txt"
            path.write_text(text)
            return runner.loaded_aircraft(path, root)

    def test_actual_log_record_format_ignores_ai_aircraft(self):
        text = (f"0:00:05.931 I/ACF: Loading airplane number 20 with Aircraft/Other/Other.acf\n"
                f"0:11:26.036 I/ACF: Loading airplane number 0 with {runner.MODEL}\r\n")
        self.assertTrue(self.inspect(text))

    def test_last_aircraft_zero_wins_not_substring_or_old_good_record(self):
        good = f"I/ACF: Loading airplane number 0 with {runner.MODEL}\n"
        for model in ("Aircraft/QuadTailsitter/QuadTailsitter.acf", str(runner.MODEL) + ".backup.acf",
                      "Aircraft/Fake/NewQuadTailsitter.acf"):
            with self.subTest(model=model), self.assertRaises(runner.RunnerError):
                self.inspect(good + f"I/ACF: Loading airplane number 0 with {model}\n")

    def test_missing_unknown_or_last_unparseable_log_has_no_proof(self):
        self.assertFalse(runner.loaded_aircraft(Path("/nonexistent/compact-test/Log.txt"), Path("/tmp")))
        for text in ("unrelated", f"I/ACF: Loading airplane number 0 with {runner.MODEL}\n"
                     "I/ACF: Loading airplane number 0 changed format\n"):
            self.assertFalse(self.inspect(text))

    def test_exact_windows_absolute_path_maps_only_selected_mount(self):
        self.assertTrue(self.inspect("I/ACF: Loading airplane number 0 with H:\\X-Plane 12/" + str(runner.MODEL)))
        with self.assertRaises(runner.RunnerError):
            self.inspect("I/ACF: Loading airplane number 0 with G:\\X-Plane 12/" + str(runner.MODEL))


class SourceAndNetworkTests(unittest.TestCase):
    def test_all_recognized_full_hashes_have_accurate_nonflight_labels(self):
        for source, label in runner.RECOGNIZED_SOURCES.items():
            self.assertEqual(len(source), 40)
            with mock.patch.object(runner, "git_read", side_effect=["/px4", source, "", ""]), contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(runner.validate_source(Path("/px4"), False), source)
            self.assertIn(label, out.getvalue())
            self.assertIn("No direct flight claim", out.getvalue())

    def test_unknown_requires_explicit_allow_and_warning(self):
        for allow in (False, True):
            with mock.patch.object(runner, "git_read", side_effect=["/px4", "f" * 40, "", ""]), contextlib.redirect_stdout(io.StringIO()) as out:
                if allow:
                    runner.validate_source(Path("/px4"), allow)
                    self.assertIn("NO flight claim", out.getvalue())
                else:
                    with self.assertRaises(runner.RunnerError):
                        runner.validate_source(Path("/px4"), allow)

    def test_dirty_source_rejected_even_with_opt_in(self):
        with mock.patch.object(runner, "git_read", side_effect=["/px4", "f" * 40, " M tracked"]):
            with self.assertRaisesRegex(runner.RunnerError, "clean"):
                runner.validate_source(Path("/px4"), True)

    def test_changed_initialized_submodule_rejected(self):
        with mock.patch.object(runner, "git_read", side_effect=["/px4", "f" * 40, "", "+abc deps/lib"]):
            with self.assertRaisesRegex(runner.RunnerError, "submodules"):
                runner.validate_source(Path("/px4"), True)

    def test_uninitialized_optional_submodules_warn_without_forcing_downloads(self):
        source = next(iter(runner.RECOGNIZED_SOURCES))
        with mock.patch.object(runner, "git_read", side_effect=["/px4", source, "", "-abc unused/NuttX"]), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(runner.validate_source(Path("/px4"), False), source)
        self.assertIn("uninitialized", out.getvalue())
        self.assertIn("downloads/updates are disabled", out.getvalue())

    def test_git_commands_are_read_only_with_no_optional_locks(self):
        with mock.patch.object(runner.subprocess, "run", return_value=mock.Mock(stdout="abc\n")) as call:
            self.assertEqual(runner.git_read(Path("/px4"), "rev-parse", "HEAD"), "abc")
        self.assertEqual(call.call_args.args[0], ["git", "--no-optional-locks", "-C", "/px4", "rev-parse", "HEAD"])

    def test_native_linux_defaults_to_localhost_without_route_lookup(self):
        with mock.patch.object(Path, "read_text", return_value="6.8.0-generic"), mock.patch.object(runner.subprocess, "run") as call:
            self.assertEqual(runner.default_host(), "127.0.0.1")
            call.assert_not_called()

    def test_wsl_defaults_to_gateway(self):
        with mock.patch.object(Path, "read_text", return_value="6.6.87.2-microsoft-standard-WSL2"), mock.patch.object(runner.subprocess, "run", return_value=mock.Mock(stdout="default via 172.23.128.1 dev eth0\n")):
            self.assertEqual(runner.default_host(), "172.23.128.1")

    def test_gateway_failure_falls_back_to_localhost(self):
        with mock.patch.object(Path, "read_text", return_value="microsoft"), mock.patch.object(runner.subprocess, "run", side_effect=FileNotFoundError):
            self.assertEqual(runner.default_host(), "127.0.0.1")

    def test_address_parser_rejects_shell_and_broadcast(self):
        self.assertEqual(runner.host_address("localhost"), "127.0.0.1")
        for address in ("localhost;id", "0.0.0.0", "255.255.255.255", "224.0.0.1", "::1"):
            with self.subTest(address=address), self.assertRaises(runner.argparse.ArgumentTypeError):
                runner.host_address(address)

    def test_environment_drops_parameter_replay_and_model_overrides(self):
        env = runner.launch_env("172.23.128.1", {"PATH": "/usr/bin", "PX4_PARAM_MC_ROLL_P": "9",
                                                "PX4_SYS_AUTOSTART": "4001", "PX4_SIMULATOR": "gz",
                                                "replay_mode": "ekf2", "R": "/bad", "px4_instance": "2"})
        self.assertEqual(env["PX4_SIMULATOR"], "xplane")
        self.assertEqual(env["PX4_SYS_AUTOSTART"], "5021")
        self.assertEqual(env["PX4_SIM_HOSTNAME"], "172.23.128.1")
        self.assertEqual(env["GIT_SUBMODULES_ARE_EVIL"], "1")
        self.assertNotIn("PX4_PARAM_MC_ROLL_P", env)
        self.assertNotIn("replay_mode", env)
        self.assertNotIn("R", env)

    def test_other_active_px4_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = Path(tmp)
            (proc / "42").mkdir()
            (proc / "42/comm").write_text("px4\n")
            (proc / "42/exe").symlink_to("/some/build/bin/px4")
            with self.assertRaisesRegex(runner.RunnerError, "PID 42"):
                runner.ensure_no_px4(proc)

    def test_other_users_executable_link_is_not_needed(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = Path(tmp)
            (proc / "1").mkdir()
            (proc / "1/comm").write_text("systemd\n")
            with mock.patch.object(Path, "resolve", side_effect=PermissionError("other user's exe")):
                runner.ensure_no_px4(proc)


class StateAndStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.values = runner.parse_profile(PROFILE.read_bytes())
        cls.profile_hash = runner.digest(PROFILE.read_bytes())

    def initialize(self, state):
        (state / "rootfs").mkdir(parents=True)
        (state / "metadata.json").write_text(json.dumps({"schema": 1, "profile_sha256": self.profile_hash}))
        (state / "rootfs/parameters.bson").write_bytes(b"preserved params")
        (state / "rootfs/.compact-profile-initialized").write_text(self.profile_hash + "\n")

    def test_new_state_is_first_and_initialized_state_is_gate_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / "state"
            self.assertEqual(runner.read_state(state, self.profile_hash), (None, True))
            self.initialize(state)
            self.assertFalse(runner.read_state(state, self.profile_hash)[1])

    def test_profile_change_fails_without_touching_saved_parameters(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp)
            self.initialize(state)
            before = (state / "rootfs/parameters.bson").read_bytes()
            with self.assertRaisesRegex(runner.RunnerError, "Profile hash changed"):
                runner.read_state(state, "changed")
            self.assertEqual((state / "rootfs/parameters.bson").read_bytes(), before)

    def test_foreign_or_incomplete_storage_never_silently_reinitialized(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp)
            (state / "parameters.bson").write_bytes(b"foreign")
            with self.assertRaisesRegex(runner.RunnerError, "without metadata"):
                runner.read_state(state, self.profile_hash)
            (state / "parameters.bson").unlink()
            self.initialize(state)
            (state / "rootfs/.compact-profile-initialized").unlink()
            with self.assertRaisesRegex(runner.RunnerError, "Incomplete"):
                runner.read_state(state, self.profile_hash)

    def test_corrupt_metadata_or_missing_bson_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp)
            (state / "metadata.json").write_text("invalid")
            with self.assertRaises(runner.RunnerError):
                runner.read_state(state, self.profile_hash)
            self.initialize(state)
            (state / "rootfs/parameters.bson").unlink()
            with self.assertRaises(runner.RunnerError):
                runner.read_state(state, self.profile_hash)

    def test_isolated_rootfs_cannot_alias_other_storage(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp)
            (state / "rootfs").symlink_to("/tmp")
            with self.assertRaisesRegex(runner.RunnerError, "symlink"):
                runner.read_state(state, self.profile_hash)

    def test_first_and_explicit_reset_apply_211_but_repeat_only_gates(self):
        for first, apply in ((True, True), (False, True), (False, False)):
            with self.subTest(first=first, apply=apply):
                startup = runner.make_startup(RCS, self.values, apply, first, self.profile_hash)
                frozen_sets = [line for line in startup.splitlines()
                               if line.startswith("param set ") and line.split()[2] in self.values]
                self.assertEqual(len(frozen_sets), 211 if apply else 0)
                self.assertEqual(startup.count("if ! param compare "), 211 + (not first))
                self.assertLess(startup.rindex("PROFILE DRIFT:"), startup.index("dataman start"))
                self.assertLess(startup.rindex("PROFILE DRIFT:"), startup.index("commander start"))
                self.assertNotIn("fs/parameters", startup)
                if not first:
                    self.assertLess(startup.index("Refusing automatic airframe reset"), startup.index("param reset_all"))
                else:
                    self.assertIn("param set SYS_AUTOCONFIG 0 || exit 1", startup)

    def test_unsupported_startup_layout_fails_closed(self):
        for text in ("unexpected startup", RCS + "\ndataman start\n"):
            with self.assertRaises(runner.RunnerError):
                runner.make_startup(text, self.values, True, True, self.profile_hash)

    def test_qgc_partner_is_explicit_and_other_links_unchanged(self):
        stock = GCS_LINE + "mavlink start -m onboard -o 14540\n"
        result = runner.make_mavlink(stock, "172.23.128.1")
        self.assertIn("-t 172.23.128.1 -o 14550", result)
        self.assertTrue(result.endswith("mavlink start -m onboard -o 14540\n"))

    def test_generated_gate_reports_all_drift_and_stops_before_modules(self):
        gate = runner.gate_script(self.values)
        script = """param() {
  if [ "$1" = compare ]; then
    [ "$2" != MC_ROLL_P ] && [ "$2" != FW_R_TC ]; return
  fi
  return 0
}
""" + gate + "echo FLIGHT_MODULE_STARTED\n"
        result = subprocess.run(["/bin/sh"], input=script, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("PROFILE DRIFT: MC_ROLL_P", result.stdout)
        self.assertIn("PROFILE DRIFT: FW_R_TC", result.stdout)
        self.assertNotIn("FLIGHT_MODULE_STARTED", result.stdout)

    def test_generated_repeat_startup_blocks_reset_flags_before_any_reset(self):
        startup = runner.make_startup(RCS, self.values, False, False, self.profile_hash)
        startup = startup.replace(". px4-alias.sh", "")
        # Shell-only stub: no PX4 binary is launched.
        stub = """param() {
  if [ "$1" = compare ] && [ "$2" = SYS_AUTOCONFIG ]; then return 1; fi
  if [ "$1" = reset_all ]; then echo DESTRUCTIVE_RESET; fi
  return 0
}
"""
        result = subprocess.run(["/bin/sh"], input=stub + startup, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Refusing automatic airframe reset", result.stdout)
        self.assertNotIn("DESTRUCTIVE_RESET", result.stdout)

    def test_actual_recognized_source_fixtures_offline_when_available(self):
        checkout = Path.home() / "PX4-Autopilot-Me"
        if not (checkout / ".git").exists():
            self.skipTest("Local read-only PX4 source fixtures unavailable")
        for commit in runner.RECOGNIZED_SOURCES:
            with self.subTest(commit=commit):
                path = "ROMFS/px4fmu_common/init.d-posix/"
                result = subprocess.run(["git", "--no-optional-locks", "-C", str(checkout),
                                         "show", f"{commit}:{path}rcS"], capture_output=True, text=True)
                if result.returncode:
                    self.skipTest(f"Local source fixture missing: {commit}")
                startup = runner.make_startup(result.stdout, self.values, False, False, self.profile_hash)
                syntax = subprocess.run(["/bin/sh", "-n"], input=startup, capture_output=True, text=True)
                self.assertEqual(syntax.returncode, 0, syntax.stderr)
                self.assertLess(startup.rindex("PROFILE DRIFT:"), startup.index("commander start"))
                self.assertLess(startup.index("Refusing automatic airframe reset"), startup.index("param reset_all"))
                self.assertNotIn("fs/parameters", startup)
                mav = subprocess.run(["git", "--no-optional-locks", "-C", str(checkout),
                                      "show", f"{commit}:{path}px4-rc.mavlink"], capture_output=True, text=True, check=True)
                self.assertIn("-t 172.23.128.1 -o 14550", runner.make_mavlink(mav.stdout, "172.23.128.1"))


class RunnerIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name).resolve()
        self.root = self.base / "X-Plane 12"
        self.px4 = self.base / "px4"
        self.state = self.base / "state with spaces"
        (self.root / runner.MODEL).parent.mkdir(parents=True)
        (self.root / runner.MODEL).write_bytes(b"private aircraft fixture, not shipped")
        (self.root / runner.CONFIG).parent.mkdir(parents=True)
        (self.root / runner.CONFIG).write_text(config_text())
        source = self.px4 / "ROMFS/px4fmu_common/init.d-posix"
        source.mkdir(parents=True)
        (source / "rcS").write_text(RCS)
        (source / "px4-rc.mavlink").write_text(GCS_LINE)
        build = self.px4 / "build/px4_sitl_default"
        (build / "etc/init.d-posix").mkdir(parents=True)
        (build / "etc/init.d-posix/rcS").write_text(RCS)
        (build / "bin").mkdir()
        (build / "bin/px4").write_bytes(b"mock binary")
        self.argv = ["--px4-path", str(self.px4), "--xplane-root", str(self.root),
                     "--profile", str(PROFILE), "--state-dir", str(self.state), "--host", "localhost"]
        for patcher in (mock.patch.object(runner, "MODEL_SHA256", runner.digest((self.root / runner.MODEL).read_bytes())),
                        mock.patch.object(runner, "validate_source", return_value=next(iter(runner.RECOGNIZED_SOURCES))),
                        mock.patch.object(runner, "ensure_no_px4")):
            patcher.start()
            self.addCleanup(patcher.stop)

    def invoke(self, *extra):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = runner.main(self.argv + list(extra))
        return code, out.getvalue(), err.getvalue()

    def test_check_and_dry_run_do_not_build_launch_or_create_state(self):
        for mode in ("--check", "--dry-run"):
            with self.subTest(mode=mode), mock.patch.object(runner.subprocess, "run") as call:
                code, out, err = self.invoke(mode, "--reset-params")
                self.assertEqual(code, 0, err)
                call.assert_not_called()
                self.assertFalse(self.state.exists())
                self.assertIn("no build, state mutation", out)
                self.assertIn("no loaded-aircraft proof", out)

    def test_unreliable_loaded_identity_requires_confirmation_before_mutation(self):
        with mock.patch.object(runner.subprocess, "run") as call:
            code, _, err = self.invoke()
            self.assertEqual(code, 1)
            self.assertIn("--confirm-aircraft-loaded", err)
            self.assertFalse(self.state.exists())
            call.assert_not_called()

    def test_wrong_loaded_aircraft_not_bypassed_by_confirmation(self):
        (self.root / "Log.txt").write_text("I/ACF: Loading airplane number 0 with Aircraft/Old/Old.acf\n")
        code, _, err = self.invoke("--confirm-aircraft-loaded", "--check")
        self.assertEqual(code, 1)
        self.assertIn("NOT the compact model", err)

    def test_model_hash_mismatch_fails_before_state_mutation(self):
        (self.root / runner.MODEL).write_bytes(b"legacy aircraft")
        code, _, err = self.invoke("--check")
        self.assertEqual(code, 1)
        self.assertIn("SHA256 mismatch", err)
        self.assertFalse(self.state.exists())

    def test_build_then_interactive_inherited_terminal_and_metadata(self):
        with mock.patch.object(runner.subprocess, "run", return_value=mock.Mock(returncode=0)) as call:
            code, _, err = self.invoke("--confirm-aircraft-loaded", "--qgc-host", "172.23.128.1")
        self.assertEqual(code, 0, err)
        self.assertEqual(call.call_count, 2)
        self.assertEqual(call.call_args_list[0].args[0], ["make", "-j2", "px4_sitl_default"])
        launch = call.call_args_list[1]
        self.assertEqual(launch.args[0][1:5], ["-w", str(self.state / "rootfs"), "-s", "../startup.sh"])
        self.assertEqual(launch.kwargs["env"]["PX4_SIM_HOSTNAME"], "127.0.0.1")
        self.assertNotIn("stdout", launch.kwargs)
        self.assertNotIn("stdin", launch.kwargs)
        self.assertNotIn("capture_output", launch.kwargs)
        meta = json.loads((self.state / "metadata.json").read_text())
        self.assertEqual(meta["profile_sha256"], runner.digest(PROFILE.read_bytes()))
        self.assertIn(meta["source_commit"], runner.RECOGNIZED_SOURCES)
        self.assertIn("-t 172.23.128.1 -o 14550", (self.state / "mavlink.sh").read_text())

    def test_profile_hash_change_blocks_even_explicit_reset(self):
        self.state.mkdir()
        (self.state / "metadata.json").write_text(json.dumps({"schema": 1, "profile_sha256": "other"}))
        with mock.patch.object(runner.subprocess, "run") as call:
            code, _, err = self.invoke("--reset-params", "--confirm-aircraft-loaded")
            self.assertEqual(code, 1)
            self.assertIn("Profile hash changed", err)
            call.assert_not_called()

    def test_failed_build_does_not_prevent_retry_or_remove_work(self):
        with mock.patch.object(runner.subprocess, "run", side_effect=subprocess.CalledProcessError(2, "make")):
            code, _, _ = self.invoke("--confirm-aircraft-loaded")
        self.assertEqual(code, 1)
        code, _, err = self.invoke("--check")
        self.assertEqual(code, 0, err)

    def test_xplane_root_required_and_environment_supported(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            args = runner.parser().parse_args(["--check"])
            with self.assertRaisesRegex(runner.RunnerError, "explicitly"):
                runner.run(args)
        with mock.patch.dict(os.environ, {"XPLANE_ROOT": str(self.root)}):
            self.assertEqual(runner.parser().parse_args([]).xplane_root, str(self.root))


if __name__ == "__main__":
    unittest.main()
