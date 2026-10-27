import importlib.util
from pathlib import Path
import socket
import struct
import sys
import threading
import unittest
import json
import tempfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'scripts'))
from simulations.x650_nx_sync_pil_v1.protocol import Client,clock_barrier,encode
from simulations.x650_nx_sync_pil_v1.cases import compose,digest,NORMAL
from run_x650_sync_pil import transport_passed


class BarrierTests(unittest.TestCase):
    def test_clean_server_exit_alone_is_not_complete(self):
        self.assertFalse(transport_passed({'server_exit':0}))
        self.assertTrue(transport_passed({'server_exit':0,'nx_exit':0}))
        for change in ({'nx_exit':1},{'server_exit':1},{'relay_thread_alive':True},{'relay_error':'closed'}):
            self.assertFalse(transport_passed(dict({'server_exit':0,'nx_exit':0},**change)))

    def test_exact_frozen_and_fixed_grid(self):
        clock_barrier(40000000,40000000)
        clock_barrier(40000000,60000000,ticks=5)
        for returned in (39999999,40000001,80000000):
            with self.assertRaises(ValueError):clock_barrier(40000000,returned)
        with self.assertRaises(ValueError):clock_barrier(40000000,64000000,ticks=5)

    def exchange(self,reply):
        listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1)
        def server():
            with listener:
                channel,_=listener.accept()
                with channel:
                    header=channel.recv(4);n=struct.unpack('!I',header)[0];data=b''
                    while len(data)<n:data+=channel.recv(n-len(data))
                    channel.sendall(reply)
        worker=threading.Thread(target=server);worker.start()
        client=Client(listener.getsockname()[1])
        try:
            with self.assertRaises((ValueError,EOFError)):client.request({'op':'test'})
            self.assertTrue(client.faulted)
            with self.assertRaises(RuntimeError):client.request({'op':'test'})
        finally:client.close();worker.join(timeout=2)

    def test_wrong_request_latches(self):
        body=encode({'request_sha256':'0'*64})
        self.exchange(struct.pack('!I',len(body))+body)

    def test_truncated_and_oversized_responses_latch(self):
        self.exchange(struct.pack('!I',100)+b'{}')
        self.exchange(struct.pack('!I',131073))


class StageTests(unittest.TestCase):
    def test_normal_requires_new_complete_hold_and_preserves_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def save(name,value):
                p=root/name;p.write_text(json.dumps(value));return {'path':str(p),'sha256':digest(p)}
            raw=save('raw.json',{'original':True})
            plan={'case':'tip_hold','source_task':raw,'startup_manifest':raw,'nx_source_sha256':'a'*64,
                'physics_tick_ns':4000000,'solve_ticks':10,'dispatch_ticks':5}
            launch=save('launch.json',{'complete':True,'runtime_closed':True,'plan':dict(plan)})
            good={'passed':True,'complete':True,'physical_closed_loop':True,'wall_clock_qualified':False,
                'nx_steps':749,'endpoint':{'duration_s':30.},'artifacts':{raw['path']:raw['sha256']}}
            report=save('report.json',good)
            plan.update(case='normal',prior_sync_hold={'launch':launch,'report':report})
            original={'mpc_config':{'weights':[4,5]},'stage':'tip_hold','duration_s':30.}
            cfg,trajectory=compose(original,plan)
            self.assertEqual(cfg['mpc_config'],original['mpc_config']);self.assertEqual(trajectory,NORMAL)
            self.assertEqual(cfg['duration_s'],32.);self.assertEqual(original['duration_s'],30.)
            bad=dict(plan,nx_source_sha256='b'*64)
            with self.assertRaisesRegex(ValueError,'retain'):compose(original,bad)
            for field in ('passed','complete','physical_closed_loop'):
                report=save('report.json',dict(good,**{field:False}))
                plan['prior_sync_hold']['report']=report
                with self.assertRaisesRegex(ValueError,'complete'):compose(original,plan)
            plan['prior_sync_hold']['report']=save('report.json',good)
            Path(raw['path']).write_text('{}')
            with self.assertRaisesRegex(ValueError,'records changed'):compose(original,plan)


if __name__=='__main__':unittest.main()
