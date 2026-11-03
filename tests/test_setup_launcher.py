"""Offline launcher checks: no Git, network, temporary files, or setup actions."""

import subprocess
import unittest
from pathlib import Path


LAUNCHER = Path(__file__).resolve().parents[1] / "setup" / "setup_px4_sitl.sh"
SOURCE = LAUNCHER.read_text(encoding="utf-8")
# Stop before uninstall or any checkout/setup side effects, without a test-only CLI.
OPTIONS = SOURCE.split('\nif [ "$UNINSTALL_MODE" = true ]; then', 1)[0]
OFFICIAL_REPO = "https://github.com/PX4/PX4-Autopilot.git"
RETIRED = (
    ("ekf-gsf", "27533", "closed without merging"),
    ("vtol-handoff", "27601", "already merged"),
    ("tailsitter-fw-frame", "27793", "already merged"),
)

# All test shell operations are builtins. Even accidental setup execution cannot
# find Git, network clients, or filesystem-mutating commands on PATH.
SHELL_GUARD = r"""
PATH=/nonexistent/px4xplane-test-commands
export PATH
command_not_found_handle() {
    printf 'Forbidden external command: %s\n' "$*" >&2
    exit 97
}
cat() {
    local line
    while IFS= read -r line; do printf '%s\n' "$line"; done
}
"""


def shell(script, args=(), env=None):
    return subprocess.run(
        ["/bin/bash", "--noprofile", "--norc", "-c", SHELL_GUARD + script,
         str(LAUNCHER), *args],
        env={"HOME": "/nonexistent/px4xplane-test-home", "LC_ALL": "C", **(env or {})},
        input="",
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )


def function_source(name):
    return name + "() {" + SOURCE.split(name + "() {", 1)[1].split("\n}\n", 1)[0] + "\n}\n"


class SetupLauncherTest(unittest.TestCase):
    def test_router_and_parameter_reset_require_explicit_flags(self):
        report='\nprintf "router=%s reset_params=%s qgc=%s\\n" "$USE_MAVLINK_ROUTER" "$RESET_PARAMS_MODE" "$QGC_HOST"\n'
        for flags,expected in (((), 'router=false reset_params=false qgc='),
                               (('--reset-config',), 'router=false reset_params=false qgc='),
                               (('--reset-params','--with-mavlink-router','--qgc-host','192.168.1.25'),
                                'router=true reset_params=true qgc=192.168.1.25')):
            with self.subTest(flags=flags):
                result=shell(OPTIONS+report,flags)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertEqual(result.stdout.strip(),expected)

    def selection(self, *args, env=None):
        result = shell(OPTIONS + '\nprint_launcher_selection\n'
                       'printf "path=%s\\ninstall=%s\\nsync=%s\\n" '
                       '"$PX4_PATH_OVERRIDE" "${1:-}" "$SYNC_MODE"\n', args, env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("Forbidden external command", result.stderr)
        return result.stdout

    def test_help_is_safe_and_explains_two_modes(self):
        for flag in ("--help", "-h"):
            with self.subTest(flag=flag):
                result = shell(SOURCE, (flag,))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertIn("--official (default; no experimental patches)", result.stdout)
                self.assertIn("--validation (experimental TECS PR #27670 only)", result.stdout)
                self.assertIn("Custom repository, branch, and checkout", result.stdout)
                self.assertIn("PX4XPLANE_SKIP_LAUNCHER_UPDATE=1", result.stdout)
                for guard, number, _ in RETIRED:
                    self.assertIn(f"--with-{guard}-guard", result.stdout)
                    self.assertIn(f"--without-{guard}-guard", result.stdout)
                    self.assertIn(f"PR #{number}", result.stdout)

    def test_default_and_official_alias_have_no_experimental_guards(self):
        for args in ((), ("--official",), ("--exact-pr",)):
            with self.subTest(args=args):
                output = self.selection(*args)
                self.assertIn("Mode: official", output)
                self.assertIn(f"Repository: {OFFICIAL_REPO}", output)
                self.assertIn("Branch: main", output)
                self.assertIn("#27670: disabled", output)
                self.assertIn("never applied", output)
                self.assertNotIn("prompt", output.lower())

    def test_validation_and_explicit_tecs_select_only_tecs(self):
        for flag in ("--validation", "--with-tecs-alt-guard"):
            with self.subTest(flag=flag):
                output = self.selection(flag)
                self.assertIn("Mode: validation", output)
                self.assertIn("#27670: enabled (experimental, developer validation only)", output)
                self.assertIn("#27533 closed unmerged; #27601 and #27793 merged; never applied", output)

    def test_last_mode_or_tecs_flag_wins(self):
        cases = (
            (("--validation", "--official"), "official", "disabled"),
            (("--official", "--validation"), "validation", "enabled"),
            (("--validation", "--exact-pr"), "official", "disabled"),
            (("--validation", "--without-tecs-alt-guard"), "validation", "disabled"),
            (("--without-tecs-alt-guard", "--validation"), "validation", "enabled"),
            (("--official", "--with-tecs-alt-guard"), "validation", "enabled"),
        )
        for args, mode, state in cases:
            with self.subTest(args=args):
                output = self.selection(*args)
                self.assertIn(f"Mode: {mode}", output)
                self.assertIn(f"#27670: {state}", output)

    def test_retired_positive_flags_fail_before_destructive_actions(self):
        for guard, number, reason in RETIRED:
            flag = f"--with-{guard}-guard"
            for args in ((flag,), ("--uninstall", flag),
                         ("--repair", "--reset-config", "--validation", flag),
                         ("--restore-official", flag)):
                with self.subTest(args=args):
                    result = shell(SOURCE, args)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn(flag + " is retired", result.stderr)
                    self.assertIn(f"PR #{number}", result.stderr)
                    self.assertIn(reason, result.stderr)
                    self.assertIn("--official (default)", result.stderr)
                    self.assertNotIn("Forbidden external command", result.stderr)
                    self.assertEqual(result.stdout, "")

    def test_retired_negative_flags_remain_no_ops(self):
        aliases = tuple(f"--without-{guard}-guard" for guard, _, _ in RETIRED)
        self.assertEqual(self.selection(*aliases), self.selection())
        self.assertEqual(self.selection("--validation", *aliases), self.selection("--validation"))

    def test_custom_options_survive_either_mode_in_either_order(self):
        custom = ("--px4-repo", "https://example.invalid/owner/PX4.git",
                  "--px4-branch", "developer-branch", "--px4-path", "/checkout with spaces/PX4")
        for mode in ("--official", "--validation", "--exact-pr"):
            for args in ((*custom, mode), (mode, *custom)):
                with self.subTest(args=args):
                    output = self.selection(*args)
                    self.assertIn("Repository: https://example.invalid/owner/PX4.git", output)
                    self.assertIn("Branch: developer-branch", output)
                    self.assertIn("path=/checkout with spaces/PX4", output)
        self.assertIn("install=/install with spaces", self.selection("/install with spaces"))

    def test_environment_overrides_remain_supported(self):
        env = {
            "PX4XPLANE_PX4_REPO": "https://example.invalid/env/PX4.git",
            "PX4XPLANE_PX4_BRANCH": "env-branch",
            "PX4XPLANE_PX4_PATH": "/env/PX4",
        }
        for args in ((), ("--official",), ("--validation",)):
            with self.subTest(args=args):
                output = self.selection(*args, env=env)
                self.assertIn("Repository: https://example.invalid/env/PX4.git", output)
                self.assertIn("Branch: env-branch", output)
                self.assertIn("path=/env/PX4", output)

    def test_no_sync_flag_is_preserved(self):
        for args in (("--no-sync",), ("--official", "--no-sync"),
                     ("--no-sync", "--official"), ("--no-sync", "--exact-pr"),
                     ("--validation", "--no-sync", "--without-tecs-alt-guard")):
            with self.subTest(args=args):
                self.assertIn("sync=false", self.selection(*args))
        self.assertIn("sync=true", self.selection("--sync"))

    def test_tecs_without_sync_fails_before_setup_in_either_order(self):
        for flag in ("--validation", "--with-tecs-alt-guard"):
            for args in ((flag, "--no-sync"), ("--no-sync", flag),
                         ("--repair", "--reset-config", flag, "--no-sync")):
                with self.subTest(args=args):
                    result = shell(SOURCE, args)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn("cannot apply experimental TECS guard PR #27670 without sync", result.stderr)
                    self.assertIn("--without-tecs-alt-guard", result.stderr)
                    self.assertNotIn("Forbidden external command", result.stderr)
                    self.assertEqual(result.stdout, "")
        self.assertIn("sync=true", self.selection("--validation", "--no-sync", "--sync"))

    def test_updater_skips_no_sync_or_explicit_environment_opt_out(self):
        mocks = r"""
readlink() { echo /nonexistent/launcher; }
mktemp() { echo /nonexistent/launcher-download; }
curl() { LAUNCHER_DOWNLOAD_ATTEMPTED=true; return 1; }
rm() { :; }
warning() { :; }
"""
        cases = (
            (("--no-sync",), {}, False),
            (("--no-sync", "--official"), {}, False),
            ((), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "1"}, False),
            (("--validation",), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "1"}, False),
            ((), {}, True),
            ((), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "0"}, True),
        )
        for args, env, attempted in cases:
            with self.subTest(args=args, env=env):
                result = shell(OPTIONS + mocks + function_source("update_launcher_script_if_needed")
                               + '\nupdate_launcher_script_if_needed\n'
                               'printf "download_attempted=%s\\n" "${LAUNCHER_DOWNLOAD_ATTEMPTED:-false}"\n',
                               args, env)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, f"download_attempted={str(attempted).lower()}\n")
                self.assertEqual(result.stderr, "")

    def test_no_sync_uses_local_head_without_remote_check_or_prompt(self):
        detection = SOURCE.split("    # === Auto-Sync Detection ===\n", 1)[1].split(
            '\n    highlight "Skipping initial setup.', 1
        )[0]
        mocks = r"""
cd() { :; }
highlight() { echo "$*"; }
info() { echo "$*"; }
warning() { echo "$*"; }
success() { echo "$*"; }
read() { echo 'Unexpected update prompt' >&2; exit 97; }
sync_px4_repo_to_remote() { echo 'Unexpected sync' >&2; exit 97; }
git() {
    if [ "$#" = 2 ] && [ "$1" = rev-parse ] && [ "$2" = HEAD ]; then
        echo offline-current-head
    else
        echo "Unexpected Git operation: $*" >&2
        exit 97
    fi
}
"""
        for args in (("--no-sync",), ("--official", "--no-sync"),
                     ("--no-sync", "--official")):
            with self.subTest(args=args):
                result = shell(OPTIONS + mocks + detection
                               + '\nprintf "local=%s remote=%s\\n" "$LOCAL_HASH" "$REMOTE_HASH"\n', args)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertIn("Using local HEAD offline-current-head; remote not checked (--no-sync).", result.stdout)
                self.assertIn("local=offline-current-head remote=offline-current-head", result.stdout)
                self.assertNotIn("up-to-date", result.stdout)
        self.assertNotIn("SKIP_UPDATE_CHECK", SOURCE)

    def test_unknown_and_missing_value_flags_fail_safely(self):
        for args in (("--unknown-option",), ("--px4-repo",),
                     ("--px4-branch",), ("--px4-path",)):
            with self.subTest(args=args):
                result = shell(SOURCE, args)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertNotIn("Forbidden external command", result.stderr)
                self.assertIn("Usage:", result.stdout)

    def test_restore_summary_reports_actual_official_target(self):
        output = self.selection("--validation", "--px4-repo", "https://example.invalid/PX4.git",
                                "--px4-branch", "custom", "--restore-official")
        self.assertIn("Mode: restore-official (no experimental patches)", output)
        self.assertIn(f"Repository: {OFFICIAL_REPO}", output)
        self.assertIn("Branch: main", output)
        self.assertIn("#27670: disabled", output)

    def test_obsolete_refs_and_automatic_guard_prompts_are_absent(self):
        for number in ("27533", "27601", "27793"):
            self.assertNotIn(f"refs/pull/{number}/head", SOURCE)
            self.assertNotIn(f"pr-{number}-", SOURCE)
        for guard in ("EKF_GSF", "VTOL_HANDOFF", "TAILSITTER_FW_FRAME"):
            self.assertNotIn(f"APPLY_{guard}_GUARD", SOURCE)
            self.assertNotIn(f"PX4XPLANE_{guard}_GUARD_REF", SOURCE)
        self.assertNotIn("prompt_for_pending_px4_fixes", SOURCE)
        self.assertNotIn("prompt_for_tecs_alt_guard", SOURCE)

    def test_sync_fetches_and_applies_only_explicit_tecs(self):
        mocks = r"""
cd() { :; }
info() { :; }
warning() { :; }
success() { :; }
progress() { :; }
reset_saved_sitl_parameters() { :; }
require_clean_px4_checkout() { :; }
require_no_running_px4() { :; }
git() {
    case "$1" in
        status) ;;
        rev-list) echo 0 ;;
        rev-parse) echo fake-head ;;
        remote) [ "$2" != get-url ] || echo "$REPO_URL" ;;
        *) printf 'git'; printf ' %s' "$@"; printf '\n' ;;
    esac
}
fetch_guard_ref() {
    if [ "$1" = true ]; then printf 'guard-fetch %s %s\n' "$2" "$3"; fi
    return 0
}
apply_guard_commits() {
    if [ "$2" = true ]; then printf 'guard-apply %s\n' "$3"; fi
    return 0
}
"""
        for args, enabled in (((), False), (("--official",), False),
                              (("--validation",), True),
                              (("--validation", "--without-tecs-alt-guard"), False)):
            with self.subTest(args=args):
                result = shell(OPTIONS + mocks + function_source("sync_px4_repo_to_remote")
                               + '\nsync_px4_repo_to_remote false\n', args)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertEqual("guard-fetch" in result.stdout, enabled)
                self.assertEqual("guard-apply" in result.stdout, enabled)
                if enabled:
                    self.assertIn("guard-fetch refs/pull/27670/head pr-27670-", result.stdout)
                    self.assertIn("guard-apply pr-27670-", result.stdout)
                for number in ("27533", "27601", "27793"):
                    self.assertNotIn(number, result.stdout)


if __name__ == "__main__":
    unittest.main()
