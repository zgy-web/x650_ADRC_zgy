import copy
import json
from pathlib import Path
import sys
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from nx_uart_operator_panel import Session, handler, restored


class InertSession:
    def __init__(self):
        self.calls = []
    def snapshot(self):
        return {'phase':'idle'}
    def start(self, action):
        self.calls.append(action)
        return True


@pytest.fixture
def local_panel():
    session = InertSession()
    server = ThreadingHTTPServer(('127.0.0.1',0),handler(session,'test-token'))
    thread = threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    try:
        yield session, 'http://127.0.0.1:'+str(server.server_port)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_get_never_starts_hardware_and_browser_post_is_explicit(local_panel):
    session,url=local_panel
    with urlopen(url+'/',timeout=2) as r:
        assert '开始限时测试' in r.read().decode()
    with urlopen(url+'/status',timeout=2) as r:
        assert json.load(r)['phase']=='idle'
    assert session.calls==[]
    request=Request(url+'/start',method='POST',headers={'Origin':url,'X-Operator-Token':'test-token'})
    with urlopen(request,timeout=2) as r:
        assert r.status==202
    assert session.calls==['start']


@pytest.mark.parametrize('origin,token,path,code',[
    ('https://example.com','test-token','/start',403),
    ('SELF','wrong-token','/start',403),
    ('SELF','test-token','/arm',404),
    ('SELF','test-token','/configure',404),
])
def test_foreign_requests_and_unapproved_actions_cannot_dispatch(local_panel,origin,token,path,code):
    session,url=local_panel
    request=Request(url+path,method='POST',headers={'Origin':url if origin=='SELF' else origin,'X-Operator-Token':token})
    with pytest.raises(HTTPError) as error:
        urlopen(request,timeout=2)
    assert error.value.code==code
    assert session.calls==[]


@pytest.fixture
def final_report():
    return {'complete':True,'port_closed':True,'before':{'kill':True,'outputs':{'0':0,'1':0,'2':0,'3':0}},
            'final_raw':{'vehicle_status':'arming_state: 1\nnav_state: 1\n',
                         'actuator_armed':'kill: True\n',
                         'manual_control_switches':'arm_switch: 3\nmode_slot: 1\n',
                         'failsafe_flags':'offboard_control_signal_lost: True\n'}}


def test_restore_requires_real_kill_output_and_disarmed_evidence(final_report):
    assert restored(final_report)
    mutations=[('actuator_armed','kill: False\n'),
               ('vehicle_status','arming_state: 2\nnav_state: 1\n'),
               ('vehicle_status','arming_state: 1\nnav_state: 14\n'),
               ('manual_control_switches','arm_switch: 2\nmode_slot: 1\n')]
    for topic,raw in mutations:
        case=copy.deepcopy(final_report);case['final_raw'][topic]=raw
        assert not restored(case)
    final_report['before']['outputs']['0']=48
    assert not restored(final_report)


def test_completed_panel_replaces_old_test_mode_with_restored_state(tmp_path, monkeypatch, final_report):
    session=Session(tmp_path,Path('build'),Path('identity'),Path('known_hosts'),'ssh')
    session.set(nav_state=14,actual_kill=False)
    (tmp_path/'restore').mkdir()
    (tmp_path/'restore/report.json').write_text(json.dumps(final_report))
    class Finished:
        def wait(self,timeout):
            return 0
        def join(self,timeout):
            pass
    monkeypatch.setattr(session,'launch',lambda *args:(Finished(),Finished()))
    session.finish()
    assert session.snapshot()['phase']=='restored'
    assert session.snapshot()['nav_state']==1
    assert session.snapshot()['actual_kill'] is True
    assert session.snapshot()['armed'] is False
