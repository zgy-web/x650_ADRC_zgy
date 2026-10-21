#!/usr/bin/env python3
"""Review/apply the pinned SITL DDS estimator-output profile; never fly or download."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]
PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
PROFILE = {"vehicle_attitude": ("VehicleAttitude", 50.),
           "vehicle_angular_velocity": ("VehicleAngularVelocity", 50.)}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def reviewed_text(text):
    """Change only the two named publication blocks, retaining unrelated YAML."""
    text = text.replace("\r\n", "\n")
    for topic, (typ, rate) in PROFILE.items():
        active = re.compile(r"(?m)^  - topic: /fmu/out/"+topic+r"\s*$")
        if not active.search(text):
            pattern = re.compile(r"(?m)^  # - topic: /fmu/out/"+topic+r"[ \t]*\n  #   type: px4_msgs::msg::"+typ+r"[ \t]*$")
            text, count = pattern.subn("  - topic: /fmu/out/"+topic+"\n    type: px4_msgs::msg::"+typ, text)
            if count != 1:
                raise ValueError("Exactly one pinned publication/comment block required: "+topic)
        pattern = re.compile(r"(?m)^  - topic: /fmu/out/"+topic+r"[ \t]*\n((?:    [^\n]*\n)*)")
        matches = list(pattern.finditer(text))
        if len(matches) != 1:
            raise ValueError("Duplicate or malformed publication: "+topic)
        match = matches[0]
        body = match.group(1)
        if "    type: px4_msgs::msg::"+typ+"\n" not in body:
            raise ValueError("Unexpected DDS message type: "+topic)
        if re.search(r"(?m)^    rate_limit:", body):
            body, count = re.subn(r"(?m)^    rate_limit:.*$", "    rate_limit: "+str(rate), body)
            if count != 1:
                raise ValueError("Duplicate rate_limit: "+topic)
        else:
            body += "    rate_limit: "+str(rate)+"\n"
        text = text[:match.start()]+"  - topic: /fmu/out/"+topic+"\n"+body+text[match.end():]
    document = yaml.safe_load(text)
    publications = document["publications"]
    for topic, (typ, rate) in PROFILE.items():
        values = [p for p in publications if p["topic"] == "/fmu/out/"+topic]
        if len(values) != 1 or values[0].get("type") != "px4_msgs::msg::"+typ or values[0].get("rate_limit") != rate:
            raise ValueError("DDS profile verification failed: "+topic)
    for topic in ("vehicle_local_position", "estimator_status_flags"):
        if len([p for p in publications if p["topic"] == "/fmu/out/"+topic]) != 1:
            raise ValueError("Required existing DDS publication missing: "+topic)
    return text


def configure(px4, output, *, apply=False):
    px4, output = Path(px4).resolve(strict=True), Path(output).resolve()
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=px4, check=True, capture_output=True, text=True).stdout.strip()
    if head != PINNED_PX4:
        raise ValueError("This telemetry profile requires the pinned PX4 commit")
    target = px4/"src/modules/uxrce_dds_client/dds_topics.yaml"
    if target.is_symlink() or not target.resolve().is_relative_to(px4):
        raise ValueError("DDS target must be an ordinary file inside PX4")
    original = target.read_bytes()
    changed = reviewed_text(original.decode("utf-8")).encode("utf-8")
    report = {"kind": "amsim_px4_estimator_telemetry_v1", "px4_commit": head,
              "target": str(target), "before_sha256": digest(original), "after_sha256": digest(changed),
              "publications_hz": {k: v[1] for k,v in PROFILE.items()},
              "changes_needed": original != changed, "applied": False,
              "build_required": original != changed, "flight_started": False,
              "note": "Rebuild existing px4_sitl_default, reinstall custom airframe, then verify actual topic streams before use."}
    if apply:
        if (ROOT/".amsim/runtime.json").exists() or (ROOT/".amsim/x650_diagnostic_restore_pending.json").exists():
            raise RuntimeError("An active runtime or pending restoration prevents DDS edits")
        running = subprocess.run(["pgrep", "-x", "px4"], capture_output=True)
        if running.returncode != 1:
            raise RuntimeError("PX4 must be stopped before changing/rebuilding DDS")
        output.mkdir(parents=True, exist_ok=False)
        (output/"dds_topics.before.yaml").write_bytes(original)
        (output/"dds_topics.after.yaml").write_bytes(changed)
        target.write_bytes(changed)
        if target.read_bytes() != changed:
            raise RuntimeError("DDS write verification failed")
        report["applied"] = True
        (output/"telemetry_config.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    print(json.dumps(configure(args.px4, args.output, apply=args.apply), indent=2))


if __name__ == "__main__":
    main()
