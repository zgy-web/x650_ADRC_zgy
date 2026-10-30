"""Exercise actual MAVLink decoding and rejection paths without a serial port."""
import base64
import importlib.util
import json
from pathlib import Path
import struct
import unittest

SPEC = importlib.util.spec_from_file_location('uart_probe', Path(__file__).resolve().parents[2] / 'scripts/nx_uart_mavlink_readonly.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)
mav = probe.common


def report_with_response(source=1, value=101, response_time=6.0):
    encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    chunks=[]
    for t in (1.0,2.0,3.0):
        heartbeat=mav.MAVLink_heartbeat_message(2,12,0,0,3,3).pack(encoder)
        chunks.append({'elapsed_s':t,'data':base64.b64encode(heartbeat).decode()})
    encoder.srcSystem=source
    reply=mav.MAVLink_param_value_message(b'MAV_0_CONFIG',struct.unpack('<f',struct.pack('<i',value))[0],6,100,0).pack(encoder)
    chunks.append({'elapsed_s':response_time,'data':base64.b64encode(reply).decode()})
    capture={'chunks':chunks,'sent':[{'elapsed_s':5.0,'bytes':len(probe.fixed_read_packet())}],
             'settings_restored':True,'fd_closed':True,'error':None,
             'kernel_check_returncode':0,'kernel_check_stderr':'','kernel_faults':[]}
    return {'ssh_returncode':0,'parameter_read_requested':True,'capture':capture,'decoded':probe.decode(capture)}


class ReadonlyProbeTest(unittest.TestCase):
    def test_fixed_packet_is_only_expected_read(self):
        msg=mav.MAVLink(None).parse_char(probe.fixed_read_packet())
        self.assertEqual(msg.get_type(),'PARAM_REQUEST_READ')
        self.assertEqual((msg.target_system,msg.target_component,msg.param_id,msg.param_index),(1,1,'MAV_0_CONFIG',-1))

    def test_bytewise_integer_response_passes(self):
        report=report_with_response()
        self.assertTrue(probe.acceptance(report)['scope_pass'])
        self.assertEqual(report['decoded']['parameter_responses'][0]['value_int32_bytewise'],101)

    def test_wrong_source_value_or_before_request_cannot_pass(self):
        for kwargs in ({'source':2},{'value':0},{'response_time':4.0}):
            with self.subTest(kwargs=kwargs):
                self.assertFalse(probe.acceptance(report_with_response(**kwargs))['parameter_readback_pass'])

    def test_kernel_fault_and_crc_error_fail(self):
        report=report_with_response()
        report['capture']['kernel_faults']=[{'message':'Unhandled context fault'}]
        self.assertFalse(probe.acceptance(report)['scope_pass'])
        report=report_with_response()
        damaged=bytearray(base64.b64decode(report['capture']['chunks'][-1]['data']))
        damaged[-1]^=1
        report['capture']['chunks'].append({'elapsed_s':7.0,'data':base64.b64encode(damaged).decode()})
        report['decoded']=probe.decode(report['capture'])
        self.assertFalse(probe.acceptance(report)['basic_receive_pass'])

    def test_no_response_and_serial_restore_failure_fail(self):
        report=report_with_response()
        report['decoded']['parameter_responses']=[]
        self.assertFalse(probe.acceptance(report)['scope_pass'])
        report=report_with_response()
        report['capture']['settings_restored']=False
        self.assertFalse(probe.acceptance(report)['scope_pass'])

    def test_unknown_nan_values_remain_unknown_in_valid_json(self):
        value=probe.json_safe({'voltage':float('nan'),'other':[float('inf'),1]})
        self.assertIsNone(value['voltage'])
        self.assertEqual(value['other'],[None,1])
        json.dumps(value,allow_nan=False)


if __name__=='__main__':unittest.main()
