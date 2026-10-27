"""Bounded server transport and exact simulation-clock guards."""
import hashlib
import json
import socket
import struct
import time

MAX_FRAME=131072


def encode(value):
    return json.dumps(value,separators=(',',':'),allow_nan=False).encode('ascii')


def clock_barrier(request_ns,returned_ns,*,ticks=None):
    expected=request_ns if ticks is None else request_ns+ticks*4_000_000
    if type(request_ns) is not int or type(returned_ns) is not int or returned_ns!=expected:
        raise ValueError('Simulation clock crossed its declared synchronous barrier')


class Client:
    def __init__(self,port):
        self.socket=socket.create_connection(('127.0.0.1',port),timeout=3.)
        self.socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.socket.settimeout(3.)
        self.faulted=False

    def request(self,document):
        if self.faulted:raise RuntimeError('PIL session already faulted')
        raw=encode(document)
        if not 0<len(raw)<=MAX_FRAME:raise ValueError('Request frame exceeds bound')
        deadline=time.monotonic()+3.
        def read(n):
            data=bytearray()
            while len(data)<n:
                remaining=deadline-time.monotonic()
                if remaining<=0:raise TimeoutError('Total PIL response deadline')
                self.socket.settimeout(remaining)
                value=self.socket.recv(n-len(data))
                if not value:raise EOFError('NX disconnected')
                data.extend(value)
            return bytes(data)
        before=time.monotonic_ns()
        try:
            self.socket.sendall(struct.pack('!I',len(raw))+raw)
            n=struct.unpack('!I',read(4))[0]
            if not 0<n<=MAX_FRAME:raise ValueError('Response frame exceeds bound')
            payload=read(n);after=time.monotonic_ns()
            response=json.loads(payload)
            if response['request_sha256']!=hashlib.sha256(raw).hexdigest():raise ValueError('Response belongs to another request')
            return response,before,after
        except BaseException:
            self.faulted=True
            raise

    def close(self):
        self.socket.close()
