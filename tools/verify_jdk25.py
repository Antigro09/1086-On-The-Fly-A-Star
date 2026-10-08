#!/usr/bin/env python3
"""Compile/package/run the unchanged CPU acceptance suite using an explicit JDK25.

No download, native WPILib, NetworkTables server, robot controller or global install.
All generated outputs stay in this checkout's ignored build directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PINS = {
    "Geometry.java": "e94a4b1ce2d245dbdddda03fb25c6d7be77aeeedf62b831804d5311984f1bdc2",
    "PlannerBackend.java": "5f6c6e709516c522c2bfc765e8d7a241c274cb4ac1d2e1d03e5d0599980665aa",
    "World.java": "e1dbc25da7c754959067b628016663cf0799c4d32e29974a78db4e94129d494a",
}
FIXTURE_PINS = {
    "ObstacleEnvelopeBuilder.java": "8a07be95384fa2f519f01d9b4b678fc4cb3910d839b0b084d53a2443bf8188c4",
    "PlannerValidation.java": "5ab942113a6b0eac6674de11bb56b224f553f5516e46757706c642bc302b189c",
}
FIXTURE_SOURCE_COMMIT = "0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home", type=Path, required=True)
    parser.add_argument("--release", type=int, choices=(17, 25), default=17)
    args = parser.parse_args()
    home = args.java_home.resolve()
    out = ROOT / "build" / "compatibility" / f"jdk25-release{args.release}"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    commands = []

    def run(command):
        command = [str(x) for x in command]
        commands.append(command)
        process = subprocess.run(command, cwd=ROOT, text=True, capture_output=True,
                                 check=False, timeout=60)
        output = process.stdout + process.stderr
        print(output, end="")
        if process.returncode:
            raise RuntimeError(f"Command failed ({process.returncode}): {command[0]}")
        return output.strip()

    compiler = run([home / "bin" / "javac", "-version"])
    if not compiler.startswith("javac 25."):
        raise ValueError("Explicit Java25 compiler required")
    runtime = run([home / "bin" / "java", "-version"])
    sources = ROOT / "vendor" / "world-state-api-src" / "org" / "frcworldstate" / "core"
    for name, expected in PINS.items():
        if hashlib.sha256((sources / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"World-State owner source pin differs: {name}")
    fixtures = ROOT / "vendor" / "world-state-test-fixtures-src" / "org" / "frcworldstate" / "core"
    for name, expected in FIXTURE_PINS.items():
        if hashlib.sha256((fixtures / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"World-State test-only export differs: {name}")

    jars = {}
    groups = [
        ("world-state-api", list(sources.glob("*.java")), []),
        ("geometric-core", list((ROOT / "src" / "main" / "java").rglob("*.java")), []),
        ("world-state-adapter", list((ROOT / "src" / "worldStateAdapter" / "java").rglob("*.java")),
         ["world-state-api", "geometric-core"]),
        ("contract-suite", list((ROOT / "src" / "contractTest" / "java").rglob("*.java")),
         ["world-state-api", "geometric-core", "world-state-adapter"]),
        ("owner-integration-fixtures", list(fixtures.glob("*.java")), ["world-state-api"]),
        ("owner-integration-suite", list((ROOT / "src" / "integrationTest" / "java").rglob("*.java")),
         ["world-state-api", "geometric-core", "world-state-adapter", "owner-integration-fixtures"]),
    ]
    for name, source_files, dependencies in groups:
        classes = out / name
        classes.mkdir(exist_ok=True)
        command = [home / "bin" / "javac", "--release", args.release, "-Xlint:all", "-Werror", "-d", classes]
        if dependencies:
            command += ["-cp", ":".join(str(jars[d]) for d in dependencies)]
        run(command + sorted(source_files))
        jar = out / f"{name}.jar"
        run([home / "bin" / "jar", "--create", "--date=2026-10-08T00:00:00Z", "--file", jar, "-C", classes, "."])
        jars[name] = jar
        with zipfile.ZipFile(jar) as archive:
            for entry in archive.namelist():
                if not entry.endswith(".class"):
                    continue
                data = archive.read(entry)
                if struct.unpack(">H", data[6:8])[0] != args.release + 44:
                    raise ValueError(f"Unexpected bytecode release: {entry}")
                if name in ("geometric-core", "world-state-adapter") and (
                    entry.startswith(("org/wpilib/", "edu/wpi/", "org/frcworldstate/", "pathplanning/control/"))
                    or b"org/wpilib/" in data or b"edu/wpi/" in data or b"com/pathplanner/" in data
                ):
                    raise ValueError(f"Production boundary leaked WPILib/owner/season classes: {entry}")

    classpath = ":".join(str(jars[g[0]]) for g in groups)
    suite = run([home / "bin" / "java", "-Xmx128m", "-cp", classpath,
                 "pathplanning.geometric.ContractSuite"])
    if "PASS: 17 acceptance groups" not in suite:
        raise ValueError("Acceptance-suite summary changed")
    integration = run([home / "bin" / "java", "-Xmx128m", "-cp", classpath,
                       "pathplanning.backend.WorldStateEnvelopeIntegrationSuite"])
    if "PASS: 20 World-State envelope/adapter/owner-validator checks" not in integration:
        raise ValueError("World-State integration summary changed")
    smoke = run([home / "bin" / "java", "-Xmx128m", "-cp", jars["geometric-core"],
                 "pathplanning.geometric.PlannerSmokeExample"])
    dependencies = run([home / "bin" / "jdeps", "--multi-release", "25", "--recursive",
                        "--class-path", ":".join(str(jars[n]) for n in ("world-state-api", "geometric-core")),
                        jars["world-state-adapter"]])
    if any(word in dependencies for word in ("org.wpilib", "edu.wpi", "not found")):
        raise ValueError("Unexpected production dependency")
    record = {
        "host": platform.platform(), "compiler": compiler, "runtime": runtime,
        "java_release": args.release, "bytecode_major": args.release + 44,
        "contract": "frc-planner/1",
        "api_export_source": {'repository': 'https://github.com/Antigro09/1086-On-The-Fly-A-Star', 'revision': '59ad897d895315a751df67c5751e30370850a784', 'path': 'vendor/world-state-api-src', 'note': 'Exact earlier owner exports compiled for this evidence; owner classes excluded from production jars.'},
        "owner_public_source": {'repository': 'https://github.com/Antigro09/FRC-World-State', 'revision': '0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab', 'exact_source_matches': ['PlannerBackend.java', 'ObstacleEnvelopeBuilder.java', 'PlannerValidation.java'], 'earlier_support_copies': ['Geometry.java', 'World.java'], 'note': 'See vendor/world-state-api-src/PROVENANCE.md for the two documented validation differences.'},
        "wpilib_runtime_classpath": [], "suite_output": suite, "smoke_output": smoke,
        "owner_integration_output": integration, "owner_fixture_sha256": FIXTURE_PINS,
        "owner_test_fixture_commit": FIXTURE_SOURCE_COMMIT,
        "jdeps_output": dependencies, "commands": commands,
        "jars": {name: {"path": str(path.relative_to(ROOT)),
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                 for name, path in jars.items()},
        "hardware_verified": False, "timed_following_verified": False,
    }
    # Preserve command/evidence detail without publishing workstation paths.
    def portable(value):
        if isinstance(value, str):
            return value.replace(str(home), "$JDK25_HOME").replace(str(ROOT), "$CHECKOUT")
        if isinstance(value, list):
            return [portable(item) for item in value]
        if isinstance(value, dict):
            return {key: portable(item) for key, item in value.items()}
        return value
    record = portable(record)
    record["path_normalization"] = {
        "$JDK25_HOME": "explicit verified JDK25 installation",
        "$CHECKOUT": "repository checkout; private absolute paths removed for publication",
    }
    result = out / "result.json"
    result.write_text(json.dumps(record, indent=2) + "\n")
    print(f"Verified JDK25 release{args.release}; evidence: {result}")


if __name__ == "__main__":
    main()
