#!/usr/bin/env python3
"""Build and execute VGSolar protocol/safety/state-machine host tests without hardware."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TESTS = [
    "src/drivers/distance_sensor/dyp_a02/test/protocol_test.cpp",
    "src/modules/vgsolar_inputs/test/processing_test.cpp",
    "src/modules/vgsolar_encoders/test/processing_test.cpp",
    "src/modules/vgsolar_safety/test/processing_test.cpp",
    "src/modules/vgsolar_link/test/protocol_test.cpp",
    "src/modules/vgsolar_link/test/status_test.cpp",
    "src/modules/vgsolar_mode/test/logic_test.cpp",
    "src/modules/vgsolar_mode/test/robot_test.cpp",
    "src/modules/vgsolar_esc/test/protocol_test.cpp",
    "src/modules/rover_differential/DifferentialActControl/test/output_gate_test.cpp",
    "src/lib/rover_control/test/wheel_feedback_test.cpp",
]
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sanitize", action="store_true", help="Enable AddressSanitizer and UndefinedBehaviorSanitizer")
    parser.add_argument("--mode-integration", action="store_true", help="Also run production mode with deterministic host transport (requires generated PX4 headers)")
    parser.add_argument("--generated-dir", type=Path, default=ROOT / "build/vg_vgsolar_default", help="Build directory containing generated uORB/topics headers")
    parser.add_argument("--all", action="store_true", help="Include mode integration, NCU client and PWM resource checks (Linux host)")
    args = parser.parse_args()
    if args.all:
        args.mode_integration = True
    tests = list(TESTS)
    if args.mode_integration:
        if not (args.generated_dir / "uORB/topics/vgsolar_mode_status.h").is_file():
            parser.error("Build vg_vgsolar_default first or pass --generated-dir with generated topic headers")
        tests.append("src/modules/vgsolar_mode/test/mode_integration_test.cpp")
    flags = ["-std=c++14", "-Wall", "-Wextra", "-Werror", "-g"]
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    failures = 0
    with tempfile.TemporaryDirectory(prefix="vgsolar-tests-") as directory:
        for index, source in enumerate(tests):
            print(f"TEST {index + 1}/{len(tests)}: {source}", flush=True)
            binary = Path(directory) / f"test-{index}"
            includes = []
            extra_sources = []
            if source.endswith("mode_integration_test.cpp"):
                includes = ["-I" + str(ROOT / "src/modules/vgsolar_mode/test/host"),
                            "-I" + str(args.generated_dir), "-I" + str(ROOT / "src/lib"), "-I" + str(ROOT / "src/lib/matrix"), "-I" + str(ROOT / "src")]
                extra_sources = [str(ROOT / "src/lib/geo/geo.cpp")]
            build = subprocess.run(["g++", *flags, *includes, str(ROOT / source), *extra_sources, "-o", str(binary)])
            if build.returncode or subprocess.run([str(binary)]).returncode:
                failures += 1
    print(f"RESULT: {len(tests) - failures}/{len(tests)} passed", flush=True)
    if args.all:
        for script in ("ncu_client_test.py", "check_pwm_bench.py", "check_boot_led.py"):
            print(f"HOST CHECK: {script}", flush=True)
            result = subprocess.run([sys.executable, "-B", str(ROOT / "Tools/vgsolar" / script)])
            if result.returncode:
                failures += 1
        print("ALL CHECKS: " + ("passed" if failures == 0 else "failed"), flush=True)
    return int(failures != 0)
if __name__ == "__main__":
    raise SystemExit(main())
