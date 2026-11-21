"""Offline provenance checks against disposable, entirely local Git fixtures."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / "setup/setup_px4_sitl.sh").read_text()
PROFILE = "ROMFS/px4fmu_common/init.d-posix/airframes/5021_xplane_qtailsitter"
REQUESTED_URL = "https://example.invalid/px4.git"
CACHED_REF = "refs/remotes/px4xplane/main"


def function_source(name):
    return name + "() {" + SOURCE.split(name + "() {", 1)[1].split("\n}\n", 1)[0] + "\n}\n"


class NoSyncProvenanceTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="px4xplane-provenance-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repo = self.root / "checkout"
        self.repo.mkdir()
        self.env = {
            "PATH": "/usr/bin:/bin", "HOME": str(self.root), "LC_ALL": "C",
            "GIT_CONFIG_NOSYSTEM": "1", "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_ALLOW_PROTOCOL": "file", "GIT_TERMINAL_PROMPT": "0",
            "GIT_AUTHOR_NAME": "Test", "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_NAME": "Test", "GIT_COMMITTER_EMAIL": "test@example.invalid",
            "CLONE_PATH": str(self.repo), "REPO_URL": REQUESTED_URL,
            "BRANCH_NAME": "main", "PX4_REMOTE_NAME": "px4xplane",
            "APPLY_TECS_ALT_FRAME_GUARD": "false", "SYNC_MODE": "false",
        }
        self.git("init", "--quiet", "-b", "main")
        self.profile = self.repo / PROFILE
        self.profile.parent.mkdir(parents=True)
        self.profile.write_bytes(b"param set-default TEST_VALUE 1\n")
        self.commit()
        self.git("remote", "add", "px4xplane", REQUESTED_URL)
        self.git("update-ref", CACHED_REF, "HEAD")
        self.cached_commit = self.git("rev-parse", "HEAD").strip()

    def git(self, *args):
        return subprocess.run(
            ["git", "-C", str(self.repo), *args], env=self.env, check=True,
            text=True, capture_output=True, timeout=10,
        ).stdout

    def commit(self):
        self.git("add", ".")
        self.git("-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null",
                 "commit", "--quiet", "--allow-empty", "-m", "Fixture")

    def snapshot(self):
        return {str(path.relative_to(self.repo)): path.read_bytes()
                for path in self.repo.rglob("*") if path.is_file()}

    def report(self, sync="false"):
        before = self.snapshot()
        # Whitelist read-only Git calls; prevent prompts and every other external command.
        guard = r'''
info() { printf '[INFO] %s\n' "$1"; }
warning() { printf '[WARNING] %s\n' "$1"; }
read() { printf 'UNEXPECTED_PROMPT\n' >&2; return 97; }
confirm() { printf 'UNEXPECTED_PROMPT\n' >&2; return 97; }
command_not_found_handle() { printf 'FORBIDDEN_COMMAND %s\n' "$*" >&2; return 97; }
git() {
    if [ "$SYNC_MODE" != false ] || [ "$GIT_NO_LAZY_FETCH" != 1 ] ||
        [ "$GIT_OPTIONAL_LOCKS" != 0 ] || [ "$1" != -C ] || [ "$2" != "$CLONE_PATH" ]; then
        printf 'FORBIDDEN_GIT_ENV\n' >&2; return 97
    fi
    case "$3:$4" in
        rev-parse:--verify|remote:get-url|cat-file:-t) ;;
        hash-object:--no-filters)
            [ "$#" = 6 ] && [ "$5" = -- ] || { printf 'FORBIDDEN_HASH\n' >&2; return 97; } ;;
        *) printf 'FORBIDDEN_GIT %s\n' "$*" >&2; return 97 ;;
    esac
    /usr/bin/git "$@"
}
PATH=/nonexistent/px4xplane-test-commands
export PATH
'''
        result = subprocess.run(
            ["/bin/bash", "--noprofile", "--norc", "-c", guard
             + function_source("airframe_file_for_platform")
             + function_source("report_no_sync_airframe")
             + '\nreport_no_sync_airframe xplane_qtailsitter\n'],
            env={**self.env, "SYNC_MODE": sync}, input="", text=True,
            capture_output=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        self.assertEqual(self.snapshot(), before, "Reporter changed the checkout")
        return result.stdout

    def test_matching_profile_reports_requested_and_actual(self):
        output = self.report()
        self.assertIn(f"Offline requested source: {REQUESTED_URL}@main", output)
        self.assertIn(f"Offline actual local HEAD: {self.cached_commit}", output)
        self.assertIn("matches the cached requested branch", output)
        self.assertNotIn("[WARNING]", output)

    def test_dirty_profile_warns_without_prompt_or_mutation(self):
        self.profile.write_bytes(b"param set-default TEST_VALUE 2\n")
        output = self.report()
        self.assertIn("[WARNING] Local airframe script differs", output)
        self.assertIn("continuing with local source and saved tuning", output)

    def test_raw_bytes_are_compared_without_filters(self):
        self.git("config", "core.autocrlf", "true")
        self.profile.write_bytes(self.profile.read_bytes().replace(b"\n", b"\r\n"))
        self.assertIn("[WARNING] Local airframe script differs", self.report())

    def test_missing_cached_branch_is_informational(self):
        self.git("update-ref", "-d", CACHED_REF)
        output = self.report()
        self.assertIn("unavailable: no locally cached requested branch", output)
        self.assertNotIn("[WARNING]", output)

    def test_unverified_remote_is_not_compared(self):
        for url in (REQUESTED_URL + "/", "https://elsewhere.invalid/px4.git"):
            with self.subTest(url=url):
                self.git("remote", "set-url", "px4xplane", url)
                output = self.report()
                self.assertIn("unavailable: remote px4xplane does not verify the exact requested URL", output)
                self.assertNotIn("cached blob", output)
        self.git("remote", "remove", "px4xplane")
        self.assertIn("does not verify the exact requested URL", self.report())

    def test_missing_local_profile_is_informational(self):
        self.profile.unlink()
        self.assertIn("unavailable: selected local airframe script is missing", self.report())

    def test_missing_cached_profile_is_informational(self):
        content = self.profile.read_bytes()
        self.profile.unlink()
        self.commit()
        self.git("update-ref", CACHED_REF, "HEAD")
        self.profile.write_bytes(content)
        self.assertIn("unavailable: selected script is not a cached blob", self.report())

    def test_cached_ref_is_explicitly_not_current_remote_truth(self):
        (self.repo / "unrelated.txt").write_text("Local developer work\n")
        self.commit()
        actual = self.git("rev-parse", "HEAD").strip()
        self.assertNotEqual(actual, self.cached_commit)
        output = self.report()
        self.assertIn(f"Offline actual local HEAD: {actual}", output)
        self.assertIn(f"{CACHED_REF} at {self.cached_commit}", output)
        self.assertIn("remote freshness not checked (--no-sync)", output)
        self.assertIn("matches the cached requested branch", output)
        self.assertNotIn("[WARNING]", output)

    def test_sync_mode_does_not_report_or_call_git(self):
        self.assertEqual(self.report(sync="true"), "")

    def test_call_is_in_selected_build_path_and_api_stays_three(self):
        selection = SOURCE.split("# === Platform Selection and Build ===", 1)[1]
        self.assertEqual(SOURCE.count('report_no_sync_airframe "$PLATFORM"'), 1)
        self.assertLess(selection.index('report_no_sync_airframe "$PLATFORM"'),
                        selection.index("build_with_recovery || exit 1"))
        self.assertIn("LAUNCHER_API_VERSION=3\n", SOURCE)
        self.assertIn('--source-label "$REPO_URL@$BRANCH_NAME:$APPLY_TECS_ALT_FRAME_GUARD"', SOURCE)


if __name__ == "__main__":
    unittest.main()
