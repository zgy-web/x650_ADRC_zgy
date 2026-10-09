import unittest
from amsim.hil.safety_gate import check_hitl_gate

class SafetyGateTest(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(backend="hardware",control_enabled=True,hil_injection_enabled=True,
                      physical_actuators_disconnected=True,hardware_board_verified=True,
                      hitl_firmware_verified=True,operator_present=True)
    def gate(self, **kw):
        opts=dict(hb_hil_enabled=True,heartbeat_age_s=.03,sensor_age_s=.02,actuator_age_s=.02)
        opts.update(kw)
        return check_hitl_gate(self.cfg,**opts)
    def test_nominal_preflight(self):
        self.assertTrue(self.gate().allowed)
    def test_disabled_by_default(self):
        self.assertFalse(check_hitl_gate({"backend":"hardware"},hb_hil_enabled=False,
                       heartbeat_age_s=0,sensor_age_s=0,actuator_age_s=0).allowed)
    def test_stale_input(self):
        self.assertFalse(self.gate(sensor_age_s=.101).allowed)
        self.assertFalse(self.gate(actuator_age_s=.101).allowed)
        self.assertFalse(self.gate(heartbeat_age_s=2.1).allowed)
    def test_no_hil_flag(self):
        self.assertFalse(self.gate(hb_hil_enabled=False).allowed)
    def test_no_physical_isolation(self):
        self.cfg["physical_actuators_disconnected"]=False
        self.assertFalse(self.gate().allowed)
    def test_sim_cannot_arm_hardware(self):
        self.cfg["backend"]="sitl"
        self.assertFalse(self.gate().allowed)
    def test_nonfinite_fails(self):
        self.assertFalse(self.gate(sensor_age_s=float("nan")).allowed)

if __name__=="__main__":
    unittest.main()
