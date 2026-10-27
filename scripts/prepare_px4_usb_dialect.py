"""Generate a local MAVLink Python dialect pinned by a PX4 release gitlink.

Downloads only public XML definitions to the requested PC directory. Nothing is
installed or copied to NX. The resulting source can be supplied to the USB probe.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-ref", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    if root.exists():
        parser.error("output already exists; use a fresh directory to preserve previous provenance")
    from pymavlink.generator import mavgen

    def get(url):
        request = urllib.request.Request(url, headers={"User-Agent": "flyarm-readonly-usb-bringup"})
        with urllib.request.urlopen(request, timeout=25) as response:
            return response.read()

    url = "https://api.github.com/repos/PX4/PX4-Autopilot/contents/src/modules/mavlink/mavlink?ref="+urllib.parse.quote(args.px4_ref, safe="")
    metadata = json.loads(get(url))
    sha = metadata["sha"]
    if len(sha) != 40 or metadata.get("submodule_git_url") != "https://github.com/mavlink/mavlink.git":
        raise ValueError("Unexpected MAVLink gitlink metadata")
    root.mkdir(parents=True)
    files = {}

    def download(name):
        if name in files:
            return
        if Path(name).name != name or not name.endswith(".xml"):
            raise ValueError("Unexpected XML include path")
        source_url = f"https://raw.githubusercontent.com/mavlink/mavlink/{sha}/message_definitions/v1.0/{name}"
        data = get(source_url)
        (root/name).write_bytes(data)
        files[name] = dict(url=source_url, sha256=hashlib.sha256(data).hexdigest())
        for included in ET.fromstring(data).findall("include"):
            download(included.text.strip())

    download("development.xml")
    output = root/"px4_development.py"
    # Older generator XSDs reject newer schema metadata; retain the official XML
    # unchanged, parse it, and verify generated decoding against the real stream.
    options = mavgen.Opts(str(output), wire_protocol="2.0", language="Python3", validate=False)
    if not mavgen.mavgen(options, [str(root/"development.xml")]):
        raise RuntimeError("MAVLink generation failed")
    report = dict(px4_ref=args.px4_ref, metadata_url=url, mavlink_commit=sha, files=files,
                  pymavlink_generator_version=importlib.metadata.version("pymavlink"),
                  generated_source=str(output),
                  generated_source_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (root/"source_manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
