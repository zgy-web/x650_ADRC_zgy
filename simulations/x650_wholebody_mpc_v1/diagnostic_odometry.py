"""Runtime-only ideal-position observer for explicitly labelled EV uses.

This adds no ROS bridge and changes no model file or physical property. The
existing PX4 GZBridge receives the second observer's covariance topic. The
observer lives only until the owned Gazebo runtime exits; there is no supported
dynamic system-removal operation used here. Parameters must be restored before
ending that runtime. Topic presence, finite input samples and actual EKF
horizontal-vision fusion are checked separately, never inferred from PARAM_SET.
The default remains diagnostic. Research use requires a bound, reviewed
operating condition and never certifies stage completion on its own.
"""
from __future__ import annotations
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time
from xml.etree import ElementTree as ET


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def _blocks(text, field):
    """Extract balanced protobuf-text blocks, excluding braces inside strings."""
    for match in re.finditer(r"\b"+re.escape(field)+r"\s*\{", text):
        start, depth = match.end(), 1
        for token in re.finditer(r'"(?:\\.|[^"\\])*"|[{}]', text[start:]):
            value = token.group()
            if value == "{":
                depth += 1
            elif value == "}":
                depth -= 1
            if depth == 0:
                yield text[start:start+token.start()]
                break


def _scalar(text, name, *, default=None):
    found = re.search(r"(?m)^\s*"+re.escape(name)+r"\s*:\s*([^\n]+)", text)
    return default if found is None else found.group(1).strip()


def model_entity_id(scene_text, instance):
    matches = []
    for block in _blocks(scene_text, "model"):
        name = re.search(r'(?m)^\s*name\s*:\s*("(?:\\.|[^"\\])*")', block)
        identity = re.search(r"(?m)^\s*id\s*:\s*(\d+)", block)
        if name and identity and json.loads(name.group(1)) == instance:
            matches.append(int(identity.group(1)))
    if len(matches) != 1 or matches[0] <= 1:
        raise RuntimeError("Scene did not identify exactly one model entity: "+instance)
    return matches[0]


def protobuf_odometry_samples(text):
    """Finite position plus advancing native stamps from live gz topic echo."""
    starts = [m.start() for m in re.finditer(r"(?m)^header\s*\{", text)]
    samples = []
    for begin, end in zip(starts, [*starts[1:], len(text)]):
        chunk = text[begin:end]
        stamp = next(_blocks(chunk, "stamp"), None)
        position = next(_blocks(chunk, "position"), None)
        if stamp is None or position is None:
            continue
        try:
            seconds = int(_scalar(stamp, "sec", default="0"))
            nanoseconds = int(_scalar(stamp, "nsec", default="0"))
            point = [float(_scalar(position, axis, default="0")) for axis in "xyz"]
        except (ValueError, TypeError):
            raise RuntimeError("Malformed live Gazebo odometry position/stamp")
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000 or not all(map(math.isfinite, point)):
            raise RuntimeError("Nonfinite/invalid live Gazebo odometry position/stamp")
        samples.append({"stamp_ns": seconds*1_000_000_000+nanoseconds, "position_enu_m": point})
    return samples


def listener_samples(text):
    """Parse PX4 listener output without accepting missing or NaN position."""
    starts = [m.start() for m in re.finditer(r"(?m)^\s*timestamp:\s*\d+", text)]
    samples = []
    for begin, end in zip(starts, [*starts[1:], len(text)]):
        chunk = text[begin:end]
        timestamp = re.match(r"\s*timestamp:\s*(\d+)", chunk)
        sample = re.search(r"(?m)^\s*timestamp_sample:\s*(\d+)", chunk)
        point = re.search(r"(?m)^\s*position:\s*\[([^\]]+)\]", chunk)
        quality = re.search(r"(?m)^\s*quality:\s*(-?\d+)\b", chunk)
        frame = re.search(r"(?m)^\s*pose_frame:\s*(\d+)\b", chunk)
        if not all((timestamp, sample, point, quality, frame)):
            continue
        values = [float(v.strip()) for v in point.group(1).split(",")]
        if (len(values) != 3 or not all(map(math.isfinite, values)) or int(sample.group(1)) <= 0
                or int(timestamp.group(1)) <= 0 or int(frame.group(1)) != 1
                or not -128 <= int(quality.group(1)) <= 127):
            raise RuntimeError("PX4 visual odometry is not finite, stamped NED data")
        samples.append({"timestamp_us": int(timestamp.group(1)), "timestamp_sample_us": int(sample.group(1)),
                        "position_ned_m": values, "quality": int(quality.group(1)), "pose_frame": 1})
    return samples


def fusion_status(text, required_flags=None):
    flags = re.findall(r"(?mi)^\s*cs_ev_pos:\s*(true|false|0|1)\b", text)
    stamp = re.search(r"(?m)^\s*timestamp:\s*(\d+)(?:\s*\(([-+\d.eE]+)\s+seconds ago\))?", text)
    if len(flags) != 1 or stamp is None or int(stamp.group(1)) <= 0:
        raise RuntimeError("PX4 estimator status lacks one stamped cs_ev_pos report")
    age = None if stamp.group(2) is None else float(stamp.group(2))
    if age is not None and (not math.isfinite(age) or age < -.05 or age > 2.):
        raise RuntimeError("PX4 estimator status is stale")
    result = {"cs_ev_pos": flags[0].lower() in ("true", "1"), "timestamp_us": int(stamp.group(1)),
              "listener_age_s": age}
    for name in required_flags or {}:
        values = re.findall(r"(?mi)^\s*"+re.escape(name)+r":\s*(true|false|0|1)\b", text)
        if len(values) != 1:
            raise RuntimeError("PX4 estimator status lacks one required fusion flag: "+name)
        result[name] = values[0].lower() in ("true", "1")
    return result


def active_world(root, instance):
    state_path = Path(root)/".amsim/runtime.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    meta_path = Path(state["runtime_meta"])
    if not meta_path.is_absolute():
        meta_path = Path(root)/meta_path
    metadata = json.loads(meta_path.read_text(encoding="utf-8"))
    if metadata.get("instance") != instance:
        raise RuntimeError("Diagnostic model instance differs from actual runtime metadata")
    worlds = set()
    if metadata.get("world"):
        worlds.add(metadata["world"])
    for value in metadata.values():
        if isinstance(value, str):
            match = re.match(r"^/world/([A-Za-z_][A-Za-z0-9_-]*)/model/"+re.escape(instance)+r"/", value)
            if match:
                worlds.add(match.group(1))
    if len(worlds) != 1 or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", next(iter(worlds), "")):
        raise RuntimeError("Actual runtime metadata does not identify one safe Gazebo world")
    return worlds.pop(), {"runtime_meta": str(meta_path), "runtime_meta_sha256": hashlib.sha256(meta_path.read_bytes()).hexdigest()}


class DiagnosticOdometry:
    def __init__(self, root, instance, folder, *, px4_directory=None, purpose="diagnostic", operating_condition=None):
        if purpose not in ("diagnostic", "research"):
            raise ValueError("Observer purpose must be diagnostic or research")
        if purpose == "diagnostic" and operating_condition is not None:
            raise ValueError("Diagnostic observers cannot claim a research operating condition")
        self.purpose = purpose
        self.operating_condition = None
        self.operating_condition_sha256 = None
        if purpose == "research":
            from research_condition import canonical_hash, validate_operating_condition
            self.operating_condition = validate_operating_condition(operating_condition)
            self.operating_condition_sha256 = canonical_hash(self.operating_condition)
        self.root, self.folder = Path(root).resolve(), Path(folder).resolve()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", instance):
            raise ValueError("An explicit safe Gazebo model instance is required")
        if not self.folder.is_dir():
            raise ValueError("Diagnostic run directory must already exist")
        if px4_directory is None:
            from amsim.paths import px4_dir
            px4_directory = px4_dir()
        self.px4 = Path(px4_directory).resolve()
        self.listener = self.px4/"build/px4_sitl_default/bin/px4-listener"
        if not self.listener.is_file():
            raise ValueError("Existing PX4 listener executable missing: "+str(self.listener))
        self.world, runtime_evidence = active_world(self.root, instance)
        self.instance, self.count = instance, 0
        suffix = hashlib.sha256(str(self.folder).encode()).hexdigest()[:12]
        prefix = "/x650_diagnostic/"+suffix
        self.topic = "/model/"+instance+"/odometry_with_covariance"
        self.document = {"kind": ("x650_ideal_external_position_research_observer" if purpose == "research"
                                  else "x650_ideal_external_position_diagnostic_observer"), "schema_version": 1,
            "purpose": purpose, "operating_condition": deepcopy(self.operating_condition),
            "operating_condition_sha256": self.operating_condition_sha256,
            "diagnostic_only": purpose == "diagnostic", "eligible_as_stage_gate": False, "formal_performance_claim": False,
            "ground_truth_aided": True, "formal_gps_performance_claim": False, "hardware_performance_claim": False,
            "ideal_data": True, "runtime_scoped_observer": True, "model_snapshot_unmodified": True,
            "instance": instance, "world": self.world, **runtime_evidence,
            "covariance_topic": self.topic, "odom_topic": prefix+"/odometry",
            "tf_topic": prefix+"/pose", "requested_rate_hz": 50., "gaussian_noise": 0.,
            "measurement_point": "Gazebo model/base_link origin, ENU; pinned GZBridge converts to PX4 NED",
            "covariance_semantics": "Ideal observer raw covariance; diagnostic EKF_EV_NOISE_MD=1 uses explicit parameter noise floor",
            "physical_model_changes": [], "observer_loaded": False, "input_stream_confirmed": False,
            "aiding_confirmed": False, "commands": [], "artifacts": {},
            "cleanup": "No dynamic unload; ends with owned Gazebo runtime after EKF parameters are restored",
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        self.save()

    def save(self):
        _atomic_json(self.folder/"diagnostic_odometry.json", self.document)

    def _run(self, args, pump, *, timeout_s=20., complete_when=None):
        self.count += 1
        path = self.folder/("diagnostic_odometry_%03d.log" % self.count)
        record = {"args": list(map(str, args)), "log": path.name, "timeout_s": timeout_s,
                  "started_wall_time": time.time()}
        self.document["commands"].append(record)
        self.save()
        satisfied = False
        with path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(record["args"], cwd=self.px4, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+timeout_s
                while process.poll() is None:
                    pump()
                    if path.stat().st_size > 2_000_000:
                        raise RuntimeError("Diagnostic command output exceeded bounded capture")
                    if complete_when is not None and complete_when(path.read_text(encoding="utf-8", errors="replace")):
                        satisfied = True
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError("Diagnostic command timed out: "+str(args[0]))
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3.)
                record.update(returncode=process.returncode, satisfied_capture=satisfied)
                self.document["artifacts"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
                self.save()
        text = path.read_text(encoding="utf-8", errors="replace")
        if process.returncode != 0 and not satisfied:
            raise RuntimeError("Diagnostic command failed; inspect "+str(path))
        return text

    def _visual_stream(self, pump, timeout_s=25.):
        # The pinned listener -n>1 uses a poll loop shared with stdin. Its
        # daemon-client/nonterminal route can wait without returning output.
        # -n1 takes a direct snapshot; require two genuinely different source
        # samples at least 0.1 simulation seconds apart, never two cached reads.
        deadline = time.monotonic()+timeout_s
        first = None
        while time.monotonic() < deadline:
            text = self._run([str(self.listener), "vehicle_visual_odometry", "-n", "1"], pump,
                             timeout_s=max(.1, deadline-time.monotonic()))
            samples = listener_samples(text)
            if not samples:
                raise RuntimeError("PX4 visual odometry snapshot lacks finite position/quality/stamp")
            latest = samples[-1]
            if first is None:
                first = latest
            elif latest["timestamp_sample_us"] < first["timestamp_sample_us"]:
                raise RuntimeError("PX4 visual odometry sample clock regressed")
            elif latest["timestamp_sample_us"]-first["timestamp_sample_us"] >= 100000:
                return [first, latest]
            retry = min(deadline, time.monotonic()+.1)
            while time.monotonic() < retry:
                pump()
        raise RuntimeError("PX4 visual odometry does not have two advancing finite samples separated by 0.1 simulation seconds")

    def load(self, pump):
        if self.document["observer_loaded"]:
            raise RuntimeError("Diagnostic observer was already loaded")
        try:
            scene = self._run(["gz", "service", "--timeout", "8000", "-s", "/world/"+self.world+"/scene/info",
                "--reptype", "gz.msgs.Scene", "--reqtype", "gz.msgs.Empty", "--req", ""], pump)
            entity = model_entity_id(scene, self.instance)
            config = ET.Element("diagnostic_observer")
            for key, value in {"dimensions": 3, "odom_publish_frequency": 50,
                    "odom_topic": self.document["odom_topic"], "odom_covariance_topic": self.topic,
                    "tf_topic": self.document["tf_topic"], "odom_frame": "world",
                    "robot_base_frame": self.instance+"/model_frame", "gaussian_noise": 0}.items():
                ET.SubElement(config, key).text = str(value)
            inner = "".join(ET.tostring(element, encoding="unicode") for element in config)
            request = ('entity { id: '+str(entity)+' } plugins { name: "gz::sim::systems::OdometryPublisher" '
                       'filename: "gz-sim-odometry-publisher-system" innerxml: '+json.dumps(inner)+' }')
            config_path = self.folder/"diagnostic_odometry_plugin.xml"
            config_path.write_text(inner+"\n", encoding="utf-8")
            self.document["artifacts"][config_path.name] = hashlib.sha256(config_path.read_bytes()).hexdigest()
            response = self._run(["gz", "service", "--timeout", "8000", "-s", "/world/"+self.world+"/entity/system/add",
                "--reptype", "gz.msgs.Boolean", "--reqtype", "gz.msgs.EntityPlugin_V", "--req", request], pump)
            if not re.search(r"\bdata:\s*true\b", response):
                raise RuntimeError("Gazebo did not confirm diagnostic observer load")
            self.document.update(observer_loaded=True, model_entity_id=entity)
            self.save()
            def enough(text):
                samples = protobuf_odometry_samples(text)
                return len(samples) >= 2 and samples[-1]["stamp_ns"] > samples[0]["stamp_ns"]
            topic_text = self._run(["gz", "topic", "-e", "-t", self.topic], pump, timeout_s=25., complete_when=enough)
            topic_samples = protobuf_odometry_samples(topic_text)
            if not enough(topic_text):
                raise RuntimeError("Diagnostic covariance topic has no advancing finite position stream")
            samples = self._visual_stream(pump)
            self.document.update(input_stream_confirmed=True, initial_gazebo_samples=topic_samples,
                                 initial_visual_samples=samples)
            self.save()
            return self.document
        except BaseException as exc:
            self.document["load_error"] = repr(exc)
            self.save()
            raise

    def confirm_horizontal_fusion(self, pump, timeout_s=90., required_flags=None):
        if not self.document["observer_loaded"] or not self.document["input_stream_confirmed"]:
            raise RuntimeError("Cannot certify fusion without a loaded and observed input stream")
        try:
            expected = required_flags or {"cs_ev_pos": True}
            self.document["required_fusion_flags"] = expected
            deadline = time.monotonic()+timeout_s
            while time.monotonic() < deadline:
                text = self._run([str(self.listener), "estimator_status_flags", "-n", "1"], pump,
                                 timeout_s=min(15., max(.1, deadline-time.monotonic())))
                flags = fusion_status(text, expected)
                self.document["last_estimator_status"] = flags
                self.save()
                if all(flags[name] is wanted for name, wanted in expected.items()):
                    samples = self._visual_stream(pump, timeout_s=min(25., max(.1, deadline-time.monotonic())))
                    self.document.update(aiding_confirmed=True, fusion_confirmed_wall_time=time.time(),
                                         fusion_visual_samples=samples)
                    self.save()
                    return self.document
                retry = min(deadline, time.monotonic()+1.)
                while time.monotonic() < retry:
                    pump()
            raise RuntimeError("PX4 did not confirm horizontal external-vision fusion before takeoff")
        except BaseException as exc:
            self.document.update(aiding_confirmed=False, fusion_error=repr(exc))
            self.save()
            raise
