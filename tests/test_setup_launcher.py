"""Offline launcher checks: no Git, network, temporary files, or setup actions."""

import subprocess
import unittest
from pathlib import Path


LAUNCHER = Path(__file__).resolve().parents[1] / "setup" / "setup_px4_sitl.sh"
SOURCE = LAUNCHER.read_text(encoding="utf-8")
# Stop before uninstall or any checkout/setup side effects, without a test-only CLI.
OPTIONS = SOURCE.split('\nif [ "$UNINSTALL_MODE" = true ]; then', 1)[0]
OFFICIAL_REPO = "https://github.com/PX4/PX4-Autopilot.git"
MAINTAINED_REPO = "https://github.com/alireza787b/PX4-Autopilot-Me.git"
AIRFRAMES = ('xplane_ehang184', 'xplane_alia250', 'xplane_cessna172', 'xplane_tb2', 'xplane_qtailsitter')
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
    def test_router_cleanup_never_inherits_an_unowned_pid(self):
        result = shell(OPTIONS + '\nprintf "%s" "$MAVLINK_ROUTER_PID"\n',
                       env={'MAVLINK_ROUTER_PID': '12345'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '')

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

    def test_help_is_safe_and_explains_source_modes(self):
        for flag in ("--help", "-h"):
            with self.subTest(flag=flag):
                result = shell(SOURCE, (flag,))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertIn("--maintained (default; maintained X-Plane airframes)", result.stdout)
                self.assertIn("--validation (experimental TECS PR #27670 only)", result.stdout)
                self.assertIn("Custom repository/branch overrides", result.stdout)
                self.assertIn("--airframe", result.stdout)
                self.assertIn("PX4XPLANE_LAUNCHER_BASE_URL", result.stdout)
                self.assertIn("PX4XPLANE_SKIP_LAUNCHER_UPDATE=1", result.stdout)
                for guard, number, _ in RETIRED:
                    self.assertIn(f"--with-{guard}-guard", result.stdout)
                    self.assertIn(f"--without-{guard}-guard", result.stdout)
                    self.assertIn(f"PR #{number}", result.stdout)

    def test_maintained_default_and_official_alias_have_no_experimental_guards(self):
        for args, mode, repo, branch in (((), 'maintained', MAINTAINED_REPO, 'px4xplane-airframes'),
                                       (('--maintained',), 'maintained', MAINTAINED_REPO, 'px4xplane-airframes'),
                                       (('--official',), 'official', OFFICIAL_REPO, 'main'),
                                       (('--exact-pr',), 'official', OFFICIAL_REPO, 'main')):
            with self.subTest(args=args):
                output = self.selection(*args)
                self.assertIn(f"Mode: {mode}", output)
                self.assertIn(f"Repository: {repo}", output)
                self.assertIn(f"Branch: {branch}", output)
                self.assertIn('PX4-XPlane launcher ', output)
                self.assertIn('Checkout:', output)
                self.assertNotIn('TECS', output)
                self.assertNotIn('Retired', output)
                for number in ('22493', '27533', '27601', '27793'):
                    self.assertNotIn(number, output)
                self.assertNotIn("prompt", output.lower())

    def test_validation_and_explicit_tecs_select_only_tecs(self):
        for flag in ("--validation", "--with-tecs-alt-guard"):
            with self.subTest(flag=flag):
                output = self.selection(flag)
                self.assertIn("Mode: validation", output)
                self.assertIn('TECS PR #27670: enabled (experimental)', output)
                for number in ('27533', '27601', '27793'):
                    self.assertNotIn(number, output)

    def test_intro_is_compact_ascii_and_keeps_requirements_prompt(self):
        intro = SOURCE.split('# === Introductory Information ===\n', 1)[1].split(
            '# === Prerequisites Check ===', 1)[0]
        intro.encode('ascii')
        function_source('print_launcher_selection').encode('ascii')
        for args in ((), ('--official',), ('--validation',)):
            result = shell(OPTIONS + '\nCLONE_PATH=/local/PX4\n'
                           'highlight() { printf "%s\\n" "$*"; }\n' + intro + '\n:\n', args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Checkout: /local/PX4', result.stdout)
            self.assertIn('Requirements:', result.stdout)
            self.assertIn('Press Enter to continue', result.stdout)
            self.assertLessEqual(len(result.stdout.splitlines()), 14)
            for obsolete in ('Author:', 'Current Status:', 'Supported Aircraft:', 'Retired', 'merged'):
                self.assertNotIn(obsolete, result.stdout)
            self.assertEqual('TECS PR #27670: enabled' in result.stdout, args == ('--validation',))

    def test_conflicting_modes_fail_in_either_order(self):
        cases = (
            ("--validation", "--official"), ("--validation", "--exact-pr"),
            ("--maintained", "--official"), ("--maintained", "--validation"),
            ("--official", "--with-tecs-alt-guard"),
            ("--validation", "--without-tecs-alt-guard"),
        )
        for pair in cases:
            for args in (pair, pair[::-1]):
                with self.subTest(args=args):
                    result = shell(SOURCE, args)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn('Error:', result.stderr)
                    self.assertNotIn('Forbidden external command', result.stderr)

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
                    self.assertIn("--official", result.stderr)
                    self.assertNotIn("Forbidden external command", result.stderr)
                    self.assertEqual(result.stdout, "")

    def test_retired_negative_flags_remain_no_ops(self):
        aliases = tuple(f"--without-{guard}-guard" for guard, _, _ in RETIRED)
        self.assertEqual(self.selection(*aliases), self.selection())
        self.assertEqual(self.selection("--validation", *aliases), self.selection("--validation"))

    def test_custom_options_are_separate_from_named_modes(self):
        custom = ("--px4-repo", "https://example.invalid/owner/PX4.git",
                  "--px4-branch", "developer-branch", "--px4-path", "/checkout with spaces/PX4")
        output = self.selection(*custom)
        self.assertIn('Mode: custom', output)
        self.assertIn('Repository: https://example.invalid/owner/PX4.git', output)
        self.assertIn('Branch: developer-branch', output)
        self.assertIn('path=/checkout with spaces/PX4', output)
        for mode in ("--official", "--validation", "--exact-pr", "--maintained", "--restore-official"):
            for args in ((*custom, mode), (mode, *custom)):
                with self.subTest(args=args):
                    result = shell(SOURCE, args)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn('custom PX4 repo/branch overrides', result.stderr)
                    self.assertNotIn('Forbidden external command', result.stderr)
        self.assertIn("install=/install with spaces", self.selection("/install with spaces"))

    def test_environment_overrides_remain_supported(self):
        env = {
            "PX4XPLANE_PX4_REPO": "https://example.invalid/env/PX4.git",
            "PX4XPLANE_PX4_BRANCH": "env-branch",
            "PX4XPLANE_PX4_PATH": "/env/PX4",
        }
        for args in ((),):
            with self.subTest(args=args):
                output = self.selection(*args, env=env)
                self.assertIn("Repository: https://example.invalid/env/PX4.git", output)
                self.assertIn("Branch: env-branch", output)
                self.assertIn("path=/env/PX4", output)
        for mode in ('--official', '--validation', '--maintained'):
            result = shell(SOURCE, (mode,), env)
            self.assertEqual(result.returncode, 1)
            self.assertNotIn('Forbidden external command', result.stderr)
        output = self.selection('--official', env={'PX4XPLANE_OFFICIAL_PX4_BRANCH': 'not-main',
                                                   'PX4XPLANE_DEFAULT_PX4_BRANCH': 'not-main'})
        self.assertIn('Branch: main', output)

    def test_no_sync_flag_is_preserved(self):
        for args in (("--no-sync",), ("--official", "--no-sync"),
                     ("--no-sync", "--official"), ("--no-sync", "--exact-pr"),
                     ("--no-sync", "--maintained"),
                     ("--no-sync", "--px4-path", "/local/PX4"),
                     ("--no-sync", "--px4-repo", "/local/remote.git"),
                     ("--no-sync", "--px4-branch", "feature")):
            with self.subTest(args=args):
                self.assertIn("sync=false", self.selection(*args))
                self.assertIn("sync=false", self.selection(*args[1:], args[0]))
        self.assertIn("sync=true", self.selection("--sync"))

    def test_tecs_without_sync_fails_before_setup_in_either_order(self):
        for flag in ("--validation", "--with-tecs-alt-guard"):
            for args in ((flag, "--no-sync"), ("--no-sync", flag),
                         ("--repair", "--reset-config", flag, "--no-sync")):
                with self.subTest(args=args):
                    result = shell(SOURCE, args)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn("--no-sync cannot be combined", result.stderr)
                    self.assertNotIn("Forbidden external command", result.stderr)
                    self.assertEqual(result.stdout, "")
        self.assertIn("sync=true", self.selection("--validation", "--no-sync", "--sync"))

    def test_updater_skips_no_sync_or_explicit_environment_opt_out(self):
        mocks = r"""
ensure_sitl_runner() { echo 'PAIR_UPDATE'; return 0; }
refresh_global_command() { :; }
success() { :; }
"""
        cases = (
            (("--no-sync",), {}, False),
            (("--no-sync", "--official"), {}, False),
            ((), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "1"}, False),
            (("--validation",), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "1"}, False),
            ((), {}, False),
            ((), {"PX4XPLANE_SKIP_LAUNCHER_UPDATE": "0"}, False),
            (("--update-launcher",), {}, True),
        )
        for args, env, attempted in cases:
            with self.subTest(args=args, env=env):
                result = shell(OPTIONS + mocks + function_source("update_launcher_script_if_needed")
                               + '\nupdate_launcher_script_if_needed\n',
                               args, env)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, 'PAIR_UPDATE\n' if attempted else '')
                self.assertEqual(result.stderr, "")

    def test_no_sync_uses_local_head_without_remote_check_or_prompt(self):
        detection = SOURCE.split("    # === Auto-Sync Detection ===\n", 1)[1].split(
            '\n    ensure_sitl_dependencies', 1
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
                     ("--px4-branch",), ("--px4-path",), ("--qgc-host",)):
            with self.subTest(args=args):
                result = shell(SOURCE, args)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertNotIn("Forbidden external command", result.stderr)
                self.assertIn("Usage:", result.stdout)

    def test_restore_summary_reports_actual_official_target(self):
        output = self.selection("--restore-official")
        self.assertIn("Mode: restore-official", output)
        self.assertIn(f"Repository: {OFFICIAL_REPO}", output)
        self.assertIn("Branch: main", output)
        self.assertNotIn('TECS', output)

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
update_submodules_safely() { :; }
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
                              (("--maintained",), False)):
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

    def test_list_and_airframe_validation_have_no_side_effects(self):
        result = shell(SOURCE, ('--list-airframes',))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.splitlines(), list(AIRFRAMES))
        for name in AIRFRAMES:
            result = shell(OPTIONS + '\nprintf "%s" "$AIRFRAME"\n', ('--airframe', name))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, name)
        for args in (('--airframe', 'QuadTailsitter'), ('--airframe',),
                     ('--airframe', '--no-sync'), ('--px4-path', '--no-sync')):
            result = shell(SOURCE, args)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertNotIn('Forbidden external command', result.stderr)

    def test_maintenance_conflicts_fail_before_setup(self):
        for flag in ('--repair', '--restore-official', '--validation', '--update-launcher'):
            for args in ((flag, '--no-sync'), ('--no-sync', flag)):
                result = shell(SOURCE, args)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertNotIn('Forbidden external command', result.stderr)
        for args in (('--repair', '--restore-official'), ('--restore-official', '--validation'),
                     ('--restore-official', '--reset-params'),
                     ('--update-launcher', '--airframe', 'xplane_tb2')):
            result = shell(SOURCE, args)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertNotIn('Forbidden external command', result.stderr)
        result = shell(SOURCE, ('--update-launcher',), {'PX4XPLANE_SKIP_LAUNCHER_UPDATE': '1'})
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertNotIn('Forbidden external command', result.stderr)

    def test_initial_and_repair_dependencies_are_sitl_only(self):
        setup = function_source('ensure_sitl_dependencies')
        mocks = r'''
highlight() { :; }
warning() { :; }
info() { :; }
success() { :; }
bash() { printf 'dependency-command'; printf ' %s' "$@"; printf '\n'; }
check_sitl_dependencies() { return 0; }
'''
        result = shell('SYNC_MODE=true\n' + mocks + setup + '\nensure_sitl_dependencies true\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('dependency-command Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools', result.stdout)
        self.assertNotIn('GIT_SUBMODULES_ARE_EVIL', SOURCE)

    def test_dependency_recovery_is_consented_rechecked_and_offline_safe(self):
        mocks = r'''
warning() { echo "$*"; }
progress() { echo "$*"; }
confirm() { [ "$CONSENT" = yes ]; }
check_sitl_dependencies() { [ "${INSTALLED:-false}" = true ]; }
bash() { printf 'official %s\n' "$*"; INSTALLED=true; }
'''
        function = function_source('ensure_sitl_dependencies')
        for sync, consent, expected in (('true', 'yes', 0), ('true', 'no', 1), ('false', 'yes', 1)):
            result = shell(f'SYNC_MODE={sync}\nCONSENT={consent}\n' + mocks + function + '\nensure_sitl_dependencies\n')
            self.assertEqual(result.returncode, expected, result.stdout+result.stderr)
            self.assertEqual('official Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools' in result.stdout,
                             sync == 'true' and consent == 'yes')

    def test_targeted_clean_requires_consent_and_retries_at_most_once(self):
        mocks = r'''
warning() { echo "$*"; }
confirm() { [ "$CONSENT" = yes ]; }
attempt=0
build_sitl() { attempt=$((attempt+1)); echo "build-attempt=$attempt"; return 1; }
cmake() { printf 'targeted-clean'; printf ' %s' "$@"; printf '\n'; return "$CLEAN_STATUS"; }
'''
        for consent, clean_status, attempts in (('no', 0, 1), ('yes', 0, 2), ('yes', 23, 1)):
            result = shell(f'CLONE_PATH="/checkout with spaces"\nCONSENT={consent}\nCLEAN_STATUS={clean_status}\n'
                           + mocks + function_source('build_with_recovery') + '\nbuild_with_recovery\n')
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            self.assertEqual(result.stdout.count('build-attempt='), attempts)
            self.assertEqual('targeted-clean --build /checkout with spaces/build/px4_sitl_default --target clean'
                             in result.stdout, consent == 'yes')
        self.assertNotIn('make clean', SOURCE)
        self.assertNotIn('make distclean', SOURCE)


if __name__ == "__main__":
    unittest.main()
