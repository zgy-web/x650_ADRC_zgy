"""Download the pinned public ROS hardware dependencies to the PC, never to NX."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest_path = Path(__file__).resolve().parents[1] / "configs/hardware/nx_ros2_state_dependencies_v1.json"
    manifest = json.loads(manifest_path.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    for name, item in manifest.items():
        path = args.output / (name + ".zip")
        if path.exists():
            data = path.read_bytes()
        else:
            url = f"https://codeload.github.com/{item['repository']}/zip/{item['commit']}"
            with urllib.request.urlopen(url, timeout=45) as response:
                data = response.read()
        if hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError("Pinned archive hash mismatch: " + name)
        if not path.exists():
            path.write_bytes(data)
        print(name, item["commit"], item["sha256"])
    (args.output / "dependencies.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
