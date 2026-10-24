"""Bounded, nonblocking, same-host diagnostic handoff.

Only a compact binary copy is made by the worker. JSON conversion, compression,
files and fsync belong to the recorder process. Unix datagrams are private to
an owned 0700 test directory. Pickle input is trusted local worker data only;
this is not a network or hardware protocol.
"""
import json
import pickle
from pathlib import Path
import socket
import struct
import time

MAX_PACKET = 196608
HEADER = struct.Struct("<8sQ")
MAGIC = b"FARMLOG1"


class CaptureAdapter:
    def __init__(self, adapter):
        self.adapter, self.diagnostic = adapter, None

    def __getattr__(self, name):
        return getattr(self.adapter, name)

    def step(self, *args, **kwargs):
        self.diagnostic = None
        command = self.adapter.step(*args, **kwargs)
        self.diagnostic = command.diagnostics
        return command


class DiagnosticSender:
    def __init__(self, path, *, clock_ns=time.monotonic_ns):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * MAX_PACKET)
        self.socket.setblocking(False)
        self.socket.connect(str(path))
        self.attempts = self.sent = self.dropped = self.max_bytes = 0
        self.first_error = None
        self.timings_ns = []
        self.clock_ns = clock_ns
        # Local observation only. Never inserted into the candidate or used
        # to change an admission, expiry, command, or recorder receipt.
        self.last_timing_ns = None

    def offer(self, sequence, value):
        begin = self.clock_ns()
        pickle_end = send_begin = send_end = 0
        self.last_timing_ns = None
        self.attempts += 1
        try:
            # Detach synchronously before the next allocator call can mutate an
            # array. Queueing a mutable Python object to a feeder thread is not
            # an immutable recording. There is no JSON conversion here.
            packet = HEADER.pack(MAGIC, sequence) + pickle.dumps(value, protocol=5)
            pickle_end = self.clock_ns()
            self.max_bytes = max(self.max_bytes, len(packet))
            if len(packet) > MAX_PACKET:
                raise ValueError("Diagnostic packet exceeds frozen channel capacity")
            send_begin = self.clock_ns()
            written = self.socket.send(packet)
            send_end = self.clock_ns()
            if written != len(packet):
                raise RuntimeError("Partial Unix datagram")
            self.sent += 1
            return True
        except (OSError, ValueError, TypeError, RuntimeError, pickle.PickleError) as error:
            self.dropped += 1
            if self.first_error is None:
                self.first_error = {"sequence": sequence, "error": repr(error)}
            return False
        finally:
            end = self.clock_ns()
            self.timings_ns.append(end - begin)
            self.last_timing_ns = (begin, pickle_end, send_begin, send_end, end)

    def close(self, output):
        self.socket.close()
        value = {"kind": "nonblocking_diagnostic_sender_v1", "attempts": self.attempts,
                 "sent": self.sent, "dropped": self.dropped, "first_error": self.first_error,
                 "max_packet_bytes": self.max_bytes, "capacity_bytes": MAX_PACKET,
                 "offer_ns": self.timings_ns, "complete": self.dropped == 0}
        Path(output).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
        return value
