import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.reference import ControllerState, step  # noqa: E402


def f32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


def compile_library(compiler, library_path):
    source = ROOT / "generated_code" / "aurora_arm_control.c"
    include = ROOT / "generated_code"
    compiler_path = Path(compiler)
    if compiler_path.name.lower() == "zig.exe":
        command = [
            str(compiler_path), "cc", "-shared", "-O2", f"-I{include}",
            str(source), "-o", str(library_path),
        ]
    elif compiler_path.name.lower() == "cl.exe":
        command = [
            str(compiler_path), "/nologo", "/LD", "/O2", f"/I{include}",
            str(source), f"/link", f"/OUT:{library_path}",
        ]
    else:
        command = [
            str(compiler_path), "-shared", "-O2", f"-I{include}",
            str(source), "-o", str(library_path),
        ]
    subprocess.run(command, check=True, cwd=ROOT)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=os.environ.get("AURORA_CC"))
    parser.add_argument("--samples", type=int, default=10000)
    args = parser.parse_args()
    compiler = args.cc
    if compiler is None:
        compiler = next(
            (shutil.which(name) for name in ("cc", "gcc", "clang", "cl")
             if shutil.which(name)),
            None,
        )
    if compiler is None:
        try:
            import ziglang

            candidate = Path(ziglang.__file__).parent / "zig.exe"
            if candidate.exists():
                compiler = str(candidate)
        except ImportError:
            pass
    if not compiler or not Path(compiler).exists():
        raise SystemExit("No C compiler found; pass --cc or set AURORA_CC")

    with tempfile.TemporaryDirectory(prefix="aurora-arm-sil-") as temp_dir:
        library_path = Path(temp_dir) / "aurora_arm_control.dll"
        compile_library(compiler, library_path)
        library = ctypes.CDLL(str(library_path))
        run_samples(library, args.samples, compiler)


def run_samples(library, sample_count, compiler):
    controller_type = type(
        "AuroraControllerState",
        (ctypes.Structure,),
        {"_fields_": [("speed_integral_v", ctypes.c_float)]},
    )
    native_state = controller_type()
    native = library.aurora_controller_step
    native.argtypes = [
        ctypes.POINTER(controller_type), ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_int,
    ]
    native.restype = ctypes.c_float
    library.aurora_controller_init.argtypes = [ctypes.POINTER(controller_type)]
    library.aurora_controller_init(ctypes.byref(native_state))
    reference_state = ControllerState()
    rng = random.Random(15066)
    max_error = 0.0
    max_output = 0.0
    samples = []
    for index in range(sample_count):
        values = (
            0.1 * math.sin(index * 0.017),
            0.08 * math.sin(index * 0.013),
            72.0 * math.sin(index * 0.031),
            7.9 * math.sin(index * 0.047),
            0.25 + 0.1 * math.sin(index * 0.007),
        )
        if index % 11 == 0:
            values = (
                rng.uniform(-0.1, 0.1), rng.uniform(-0.08, 0.08),
                rng.uniform(-72.0, 72.0), rng.uniform(-7.9, 7.9),
                rng.uniform(0.15, 0.35),
            )
        theta_ref, theta, omega, current, load = map(f32, values)
        collaborative = int(index % 5 != 0)
        reference_state.speed_integral_v = f32(
            reference_state.speed_integral_v
        )
        expected = f32(step(
            reference_state, theta_ref, theta, omega, current, load,
            bool(collaborative),
        ))
        actual = native(
            ctypes.byref(native_state), theta_ref, theta, omega,
            current, load, collaborative,
        )
        error = abs(actual - expected)
        max_error = max(max_error, error)
        max_output = max(max_output, abs(expected))
        samples.append((expected, actual))
    epsilon = 2.0 ** -23
    relative_scaled_limit = 16.0 * epsilon * max(1.0, max_output)
    result = {
        "compiler": Path(compiler).name,
        "samples": sample_count,
        "max_absolute_error_v": max_error,
        "float32_machine_epsilon": epsilon,
        "max_output_magnitude_v": max_output,
        "scaled_tolerance_v_16eps": relative_scaled_limit,
        "error_in_machine_epsilons": max_error / epsilon,
        "pass": max_error <= relative_scaled_limit,
        "scope": "C generated controller vs Python controller, identical float32 inputs",
    }
    results_path = ROOT / "tests" / "SIL.json"
    results_path.write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    if not result["pass"]:
        raise SystemExit("SIL mismatch exceeds declared float32 tolerance")


if __name__ == "__main__":
    main()
