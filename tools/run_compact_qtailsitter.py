#!/usr/bin/env python3
"""Owner-only compact QuadTailsitter SITL launcher (no aircraft distributed).

The separately supplied profile is data, never executed as a shell script.
Only the selected PX4 checkout's normal make target and generated local startup
are executed. This launcher neither installs plugins nor changes Git history.
"""

import argparse
import configparser
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


PROFILE_NAME = "5021_xplane_qtailsitter"
PROFILE_COUNT = 211
MODEL = Path("Aircraft/NewQuadTailsitter/NewQuadTailsitter.acf")
MODEL_SHA256 = "c70590c45d9caeb2f1521c2941da9485d87b5e5d68332a695f2109bcae17b918"
CONFIG = Path("Resources/plugins/px4xplane/64/config.ini")
RECOGNIZED_SOURCES = {
    "649929706fb34bf0b06dbd2df24e04d5d452d616": "base of flown logging child; not a direct flight claim",
    "e4aa189ade1e78baa1f202f7024c9a3ea07d09a3": "source-reviewed upstream EKF #29578; NOT flown",
    "caa183a4059bcca4189776f8b621fe806f4b311a": "profile draft PR #29598; source-reviewed, NOT flown",
    "8b673ec32724c2829f3abefc77963fbec8ac680e": "PR #29598 naming correction only; source-reviewed, NOT flown",
}
NUMBER = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?\Z")
PARAM_NAME = re.compile(r"[A-Z][A-Z0-9_]{0,31}\Z")
LOAD_RECORD = re.compile(r"I/ACF: Loading airplane number\s+0\b(.*)$")
STATE_DEFAULT = "~/.local/state/px4xplane/compact-qtailsitter"


class RunnerError(Exception):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def parse_profile(data):
    """Accept literal numeric defaults and the one known inheritance directive."""
    values = {}
    for lineno, line in enumerate(data.decode("utf-8").splitlines(), 1):
        try:
            words = shlex.split(line, comments=True, posix=True)
        except ValueError as exc:
            raise RunnerError(f"Profile line {lineno}: {exc}") from exc
        if not words or words == [".", "${R}etc/init.d/rc.vtol_defaults"]:
            continue
        if (len(words) != 4 or words[:2] != ["param", "set-default"]
                or not PARAM_NAME.fullmatch(words[2]) or not NUMBER.fullmatch(words[3])):
            raise RunnerError(f"Profile line {lineno}: expected literal numeric param set-default")
        if words[2] in values:
            raise RunnerError(f"Duplicate profile parameter: {words[2]}")
        # PX4 parameters are int32 or float32, not arbitrary-size numeric literals.
        value = float(words[3])
        if not (-3.402823466e38 <= value <= 3.402823466e38):
            raise RunnerError(f"Profile line {lineno}: number out of range")
        values[words[2]] = words[3]
    if len(values) != PROFILE_COUNT:
        raise RunnerError(f"Expected {PROFILE_COUNT} frozen defaults, found {len(values)}")
    return values


def find_profile(explicit=None, script=None):
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not path.is_file():
            raise RunnerError(f"Profile not found: {path}")
        return path
    here = Path(script or __file__).resolve().parent
    # Repository layout, then the same layout or a sibling profile in an owner bundle.
    candidates = [here.parent / "config/px4_params" / PROFILE_NAME,
                  here / "config/px4_params" / PROFILE_NAME,
                  here / PROFILE_NAME, here.parent / PROFILE_NAME]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RunnerError("Frozen profile not found; supply --profile PATH")


def validate_config(data):
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=(";", "#"))
    # Plugin INI has unsectioned global settings. Do not modify the installed file.
    try:
        parser.read_string("[global]\n" + data.decode("utf-8-sig"))
        if parser.get("global", "config_name") != "QuadTailsitter":
            raise RunnerError("Global config_name MUST be QuadTailsitter")
        section = parser["QuadTailsitter"]
        if section.get("airspeedSource", "") != "body_axis" or section.get("pitotAxisBody", "") != "-Z":
            raise RunnerError("QuadTailsitter requires airspeedSource=body_axis and pitotAxisBody=-Z")
        smoothing = section.get("actuatorSmoothingTimeConstantSec", "0")
        if not NUMBER.fullmatch(smoothing) or float(smoothing) != 0:
            raise RunnerError("QuadTailsitter actuator smoothing must be disabled")
        channels = {key for key in section if key.startswith("channel")}
        if channels != {f"channel{i}" for i in range(4)}:
            raise RunnerError("QuadTailsitter must have exactly four motor channels (0..3)")
        for i in range(4):
            fields = [field.strip() for field in section[f"channel{i}"].split(",")]
            if (len(fields) != 4 or fields[:2] != ["sim/flightmodel/engine/ENGN_thro_use", "floatArray"]
                    or re.sub(r"\s+", "", fields[2]) != f"[{i}]"
                    or not re.fullmatch(r"\[\s*-1(?:\.0*)?\s+\+?1(?:\.0*)?\s*\]", fields[3])):
                raise RunnerError(f"QuadTailsitter channel{i} must map motor {i} with range [-1 1]")
        tokens = [token.strip().casefold() for token in section.get("aircraftMatch", "").split(",")]
        if not any(token and token in str(MODEL).casefold() for token in tokens):
            raise RunnerError("QuadTailsitter aircraftMatch does not include the compact model")
    except (configparser.Error, KeyError) as exc:
        raise RunnerError(f"Invalid plugin config: {exc}") from exc


def loaded_aircraft(log, root):
    """Return match/unknown, refusing any identifiable, different aircraft-0 path."""
    if not log.is_file():
        return False
    last = None
    for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
        match = LOAD_RECORD.search(line)
        if match:
            # Keep an unparseable last record; never fall back to an older good one.
            last = match.group(1).strip()
    if last is None or not last.startswith("with "):
        return False
    path = last[5:].strip().replace("\\", "/")
    if not path or not path.lower().endswith(".acf"):
        return False
    expected = MODEL.as_posix()
    if path == expected or path == (root / MODEL).as_posix():
        return True
    # X-Plane under Windows logs a drive path; map only the selected WSL mount.
    if re.match(r"^[A-Za-z]:/", path):
        path = "/mnt/" + path[0].lower() + path[2:]
        if path.casefold() == (root / MODEL).as_posix().casefold():
            return True
    raise RunnerError(f"Last aircraft-0 load is NOT the compact model: {last}")


def git_read(px4, *args):
    result = subprocess.run(["git", "--no-optional-locks", "-C", str(px4), *args],
                            check=True, capture_output=True, text=True)
    return result.stdout.strip()


def validate_source(px4, allow):
    if git_read(px4, "rev-parse", "--show-toplevel") != str(px4):
        raise RunnerError("--px4-path must identify the PX4 checkout root")
    source = git_read(px4, "rev-parse", "HEAD")
    if git_read(px4, "status", "--porcelain", "--untracked-files=normal", "--ignore-submodules=none"):
        raise RunnerError("PX4 checkout must be clean, including submodules; no Git changes were made")
    submodules = git_read(px4, "submodule", "status", "--recursive")
    if any(line.startswith(("+", "U")) for line in submodules.splitlines()):
        raise RunnerError("Initialized PX4 submodules must match their recorded commits; "
                          "this runner never updates them")
    if any(line.startswith("-") for line in submodules.splitlines()):
        print("WARNING: some submodules are uninitialized. Required build dependencies must already "
              "be available; automatic submodule downloads/updates are disabled.")
    if source not in RECOGNIZED_SOURCES:
        if not allow:
            raise RunnerError(f"Source {source} is not allowlisted; review it and explicitly use "
                              "--allow-unvalidated-source for a draft (NO flight claim)")
        print(f"WARNING: unvalidated source {source}; NO flight claim.")
    else:
        print(f"Recognized source: {RECOGNIZED_SOURCES[source]}. No direct flight claim.")
    return source


def ensure_no_px4(proc=Path("/proc")):
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            name = (entry / "comm").read_text().strip()
        except FileNotFoundError:
            continue
        except PermissionError as exc:
            raise RunnerError(f"Cannot exclude another active PX4 process: {entry}") from exc
        # Like pgrep -x, use the executable name, not command-line paths.
        # /proc/<pid>/exe is often unreadable for unrelated users' processes.
        if name == "px4":
            raise RunnerError(f"Another active PX4 process exists (PID {entry.name}); stop it first")


def host_address(value):
    if value == "localhost":
        return "127.0.0.1"
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError("Use localhost or an IPv4 NAT gateway address") from exc
    if address.is_unspecified or address.is_multicast or int(address) == 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("Use a unicast IPv4 address")
    return str(address)


def default_host():
    try:
        if "microsoft" not in Path("/proc/sys/kernel/osrelease").read_text().lower():
            return "127.0.0.1"
        result = subprocess.run(["ip", "-4", "route", "show", "default"],
                                check=True, capture_output=True, text=True)
        for line in result.stdout.splitlines():
            fields = line.split()
            if fields[:2] == ["default", "via"]:
                return host_address(fields[2])
    except (OSError, subprocess.CalledProcessError, argparse.ArgumentTypeError, IndexError):
        pass
    return "127.0.0.1"


def read_state(state, profile_hash):
    metadata_path = state / "metadata.json"
    marker = state / "rootfs/.compact-profile-initialized"
    params = state / "rootfs/parameters.bson"
    backup = state / "rootfs/parameters_backup.bson"
    if (state / "rootfs").is_symlink():
        raise RunnerError("Isolated rootfs must not be a symlink to another runner's storage")
    if not metadata_path.exists():
        if state.exists() and any(item.name != ".runner.lock" for item in state.iterdir()):
            raise RunnerError("State exists without metadata; preserve it and select a fresh --state-dir")
        return None, True
    try:
        metadata = json.loads(metadata_path.read_text())
    except (ValueError, OSError) as exc:
        raise RunnerError(f"Cannot read state metadata; existing state preserved: {exc}") from exc
    if not isinstance(metadata, dict) or metadata.get("schema") != 1:
        raise RunnerError("Unknown state metadata; existing state preserved")
    if metadata.get("profile_sha256") != profile_hash:
        raise RunnerError("Profile hash changed; state preserved. Review the profile or select a fresh "
                          "--state-dir; --reset-params does not bypass this check")
    if marker.exists():
        if marker.read_text().strip() != profile_hash or not params.is_file():
            raise RunnerError("Initialized state is incomplete; preserve and inspect it before retrying")
        return metadata, False
    if params.exists() or backup.exists():
        raise RunnerError("Incomplete initialization contains parameters; preserve and inspect state. "
                          "Select a fresh --state-dir instead of silently overwriting work")
    return metadata, True


def gate_script(values):
    lines = ["compact_drift=0"]
    for name, value in values.items():
        lines += [f"if ! param compare {name} {value}; then",
                  f"  echo 'PROFILE DRIFT: {name} expected {value}'",
                  f"  param show {name}", "  compact_drift=1", "fi"]
    lines += ['if [ "$compact_drift" != 0 ]; then',
              "  echo 'Profile gate failed; saved parameters preserved. Review drift or use --reset-params.'",
              "  exit 1", "fi"]
    return "\n".join(lines) + "\n"


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RunnerError(f"Unsupported PX4 startup layout (expected one {old.strip()!r})")
    return text.replace(old, new, 1)


def make_startup(rcs, values, apply, first, profile_hash):
    """Adapt the stock startup in state, keeping the PX4 source and build untouched."""
    start = "# Load parameters\n"
    end = "# exit early when the minimal shell is requested\n"
    if rcs.count(start) != 1 or rcs.count(end) != 1 or rcs.index(start) >= rcs.index(end):
        raise RunnerError("Unsupported PX4 parameter startup layout")
    load = """# Load only this runner's isolated parameters; no automatic backup recovery.
set PARAM_FILE parameters.bson
set PARAM_BACKUP_FILE parameters_backup.bson
param select $PARAM_FILE || exit 1
if [ -f "$PARAM_FILE" ]; then
  param import || exit 1
fi
param select-backup $PARAM_BACKUP_FILE || exit 1
"""
    if not first:
        # Prevent rcS's auto-airframe reset from erasing ordinary user parameters.
        load += """if ! param compare SYS_AUTOSTART 5021 || ! param compare SYS_AUTOCONFIG 0; then
  echo 'Refusing automatic airframe reset; inspect SYS_AUTOSTART/SYS_AUTOCONFIG. State preserved.'
  exit 1
fi
"""
    rcs = rcs[:rcs.index(start)] + load + "\n" + rcs[rcs.index(end):]
    hook = "\n# Compact profile gate: before simulator, commander, or actuators start.\n"
    if apply:
        hook += "\n".join(f"param set {name} {value} || exit 1" for name, value in values.items()) + "\n"
    hook += gate_script(values)
    if apply:
        if first:
            hook += "param set SYS_AUTOCONFIG 0 || exit 1\n"
        hook += "param save || exit 1\n"
        hook += f"printf '%s\\n' {shlex.quote(profile_hash)} > .compact-profile-initialized || exit 1\n"
    hook += "echo 'Compact profile: all 211 values passed (not flight validation).'\n\ndataman start\n"
    rcs = replace_once(rcs, "\ndataman start\n", hook)
    # Override only the GCS startup script, leaving the normal offboard links intact.
    rcs = replace_once(rcs, ". px4-rc.mavlink\n", '. ../mavlink.sh\n')
    return rcs


def make_mavlink(stock, qgc):
    # Recognized revisions use this exact GCS start command.
    old = "mavlink start -x -u $udp_gcs_port_local -r 4000000 -f $mavlink_network_interface_arg\n"
    new = ("mavlink start -x -u $udp_gcs_port_local -r 4000000 -f "
           f"-t {qgc} -o 14550 $mavlink_network_interface_arg\n")
    return replace_once(stock, old, new)


def launch_env(host, base=None):
    # Inherited PX4_PARAM_*, replay settings, or another model must not bypass the gate.
    env = {key: value for key, value in (os.environ if base is None else base).items()
           if not key.startswith("PX4_") and key not in {"replay_mode", "R", "px4_instance"}}
    env.update(PX4_SIMULATOR="xplane", PX4_SYS_AUTOSTART="5021", PX4_SIM_HOSTNAME=host,
               PX4_SIM_MODEL="xplane_qtailsitter")
    # Do not let make silently initialize/update missing submodules over the network.
    env["GIT_SUBMODULES_ARE_EVIL"] = "1"
    return env


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--px4-path", default="~/PX4-Autopilot-Me")
    result.add_argument("--xplane-root", default=os.environ.get("XPLANE_ROOT"),
                        help="Required explicitly or via XPLANE_ROOT; commonly /mnt/h/X-Plane 12")
    result.add_argument("--profile", help="Frozen 211-default profile; repository or sibling bundle by default")
    result.add_argument("--host", type=host_address, help="X-Plane IPv4 (default: NAT gateway or localhost)")
    result.add_argument("--qgc-host", type=host_address, help="QGC IPv4 (default: --host)")
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Read-only prerequisites check; no build or parameter gate")
    mode.add_argument("--dry-run", action="store_true", help="Read-only check and command preview; no build or launch")
    result.add_argument("--reset-params", action="store_true",
                        help="Explicitly reapply only the 211 profile values, preserving all other parameters")
    result.add_argument("--allow-unvalidated-source", action="store_true",
                        help="Permit a reviewed, clean draft PX4 commit; NO flight claim")
    result.add_argument("--state-dir", default=STATE_DEFAULT)
    result.add_argument("--confirm-aircraft-loaded", action="store_true",
                        help="Confirm exact aircraft is loaded when Log.txt cannot identify aircraft 0")
    return result


def operator_check(root, config_hash, known):
    print(f"Aircraft on disk: {root / MODEL}\nACF SHA256: {MODEL_SHA256}")
    print(f"Plugin config: {root / CONFIG}\nConfig SHA256: {config_hash}")
    if known:
        print("Last Log.txt aircraft-0 load path matches exactly. This is NOT live loaded-memory proof.")
    else:
        print("Log.txt does not reliably identify aircraft 0; no loaded-aircraft proof is claimed.")
    print("OPERATOR: load/verify the exact aircraft above, select QuadTailsitter in px4xplane, "
          "enable its connection, and keep that aircraft loaded. On-disk hashes do not prove live physics.")


def run(args):
    if not args.xplane_root:
        raise RunnerError("Set --xplane-root '/mnt/h/X-Plane 12' or XPLANE_ROOT explicitly")
    root = Path(args.xplane_root).expanduser().resolve()
    px4 = Path(args.px4_path).expanduser().resolve()
    state = Path(args.state_dir).expanduser().resolve()
    profile = find_profile(args.profile)
    profile_bytes = profile.read_bytes()
    values = parse_profile(profile_bytes)
    profile_hash = digest(profile_bytes)
    if digest((root / MODEL).read_bytes()) != MODEL_SHA256:
        raise RunnerError("Owner-supplied compact ACF SHA256 mismatch; legacy/substitute models are forbidden")
    config_bytes = (root / CONFIG).read_bytes()
    validate_config(config_bytes)
    known = loaded_aircraft(root / "Log.txt", root)
    operator_check(root, digest(config_bytes), known)
    source = validate_source(px4, args.allow_unvalidated_source)
    ensure_no_px4()
    metadata, first = read_state(state, profile_hash)
    host = args.host or default_host()
    qgc = args.qgc_host or host
    env = launch_env(host)
    build = px4 / "build/px4_sitl_default"
    startup_source = px4 / "ROMFS/px4fmu_common/init.d-posix"
    rcs = (startup_source / "rcS").read_text()
    startup = make_startup(rcs, values, first or args.reset_params, first, profile_hash)
    mavlink = make_mavlink((startup_source / "px4-rc.mavlink").read_text(), qgc)
    command = [str(build / "bin/px4"), "-w", str(state / "rootfs"),
               "-s", "../startup.sh", str(build / "etc")]
    print(f"Source: {source}\nProfile: {profile}\nProfile SHA256: {profile_hash}")
    print(f"Isolated state: {state}\nX-Plane host: {host}; QGC host: {qgc}")
    print("Parameter action: " + ("apply 211 then gate" if first or args.reset_params else "gate only; preserve saved parameters"))
    print("Build (existing checkout/cache): make -j2 px4_sitl_default")
    print("Launch: " + shlex.join(command))
    if args.check or args.dry_run:
        print("Read-only check complete; no build, state mutation, or live parameter gate performed.")
        if not known and not args.confirm_aircraft_loaded:
            print("Before launch, verify the aircraft and pass --confirm-aircraft-loaded if the log remains unavailable.")
        return 0
    if not known and not args.confirm_aircraft_loaded:
        raise RunnerError("Verify the exact loaded aircraft, then pass --confirm-aircraft-loaded")
    state.mkdir(parents=True, exist_ok=True)
    # A runner lock spans build and the interactive process; /proc also excludes other runners.
    with (state / ".runner.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RunnerError("Another compact runner owns this state") from exc
        if read_state(state, profile_hash) != (metadata, first):
            raise RunnerError("State changed while acquiring runner lock; rerun without discarding it")
        ensure_no_px4()
        subprocess.run(["make", "-j2", "px4_sitl_default"], cwd=px4, env=env, check=True)
        if validate_source(px4, args.allow_unvalidated_source) != source:
            raise RunnerError("PX4 source changed during build; refusing launch")
        ensure_no_px4()
        # The generated build startup must match the source inspected above.
        if (build / "etc/init.d-posix/rcS").read_text() != rcs:
            raise RunnerError("Built rcS differs from inspected source; refusing cached/stale startup")
        if not (build / "bin/px4").is_file():
            raise RunnerError("Build did not produce bin/px4")
        if (digest((root / MODEL).read_bytes()) != MODEL_SHA256
                or (root / CONFIG).read_bytes() != config_bytes or profile.read_bytes() != profile_bytes):
            raise RunnerError("Aircraft, plugin config, or profile changed during build; refusing launch")
        if not loaded_aircraft(root / "Log.txt", root) and not args.confirm_aircraft_loaded:
            raise RunnerError("Loaded-aircraft log became unreliable during build; verify before retrying")
        (state / "rootfs").mkdir(exist_ok=True)
        record = dict(metadata or {}, schema=1, profile_sha256=profile_hash, source_commit=source,
                      profile_path=str(profile), model_sha256=MODEL_SHA256,
                      binary_sha256=digest((build / "bin/px4").read_bytes()),
                      config_sha256=digest(config_bytes), startup_sha256=digest(startup.encode()))
        (state / "startup.sh").write_text(startup)
        (state / "mavlink.sh").write_text(mavlink)
        temporary = state / "metadata.json.tmp"
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        temporary.replace(state / "metadata.json")
        print("Starting interactive PX4 with inherited terminal; no automatic arming or flight commands.", flush=True)
        # Inherit stdin/stdout/stderr directly, including the caller's PTY.
        return subprocess.run(command, cwd=state / "rootfs", env=env).returncode


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return run(args)
    except (RunnerError, OSError, UnicodeError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
