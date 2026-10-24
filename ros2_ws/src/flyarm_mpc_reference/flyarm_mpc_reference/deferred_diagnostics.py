"""Private zero-FF adapter: publish controls before finishing diagnostics.

This policy is selected only by the explicitly configured live worker.
The single-thread worker must finish and synchronously serialize the result
before invoking another step. Public ControllerAdapter behavior is untouched.
"""
import numpy as np
from x650_wholebody import adapter as common
from x650_wholebody.api import ControlCommand


class DeferredLiveDiagnostics:
    def __init__(self, adapter):
        if (adapter.acceleration_feedforward_policy != 'zero'
                or not adapter._owned_diagnostic_snapshot
                or adapter._last_task_t is not None):
            raise ValueError('Deferred diagnostics require pre-step owned zero-FF adapter')
        self.adapter = adapter
        self._pending = None

    def __getattr__(self, name):
        return getattr(self.adapter, name)

    def reset(self, *args, **kwargs):
        if self._pending is not None:
            raise RuntimeError('Finish pending diagnostics before reset')
        return self.adapter.reset(*args, **kwargs)

    def step(self, t, dt, state, target):
        if self._pending is not None:
            raise RuntimeError('Finish pending diagnostics before another step')
        a = self.adapter
        if a.acceleration_feedforward_policy != 'zero' or not a._owned_diagnostic_snapshot:
            raise RuntimeError('Private diagnostic policy changed')
        if not a._initialized:
            raise RuntimeError('reset(p0,R0,q0) is required before task step')
        if (not np.isfinite(t) or t < 0 or not np.isfinite(dt) or dt <= 0
                or (a._last_task_t is not None and t <= a._last_task_t)):
            raise ValueError('Task time must increase and dt must be positive')
        stamp, _ = a._motion_sample(state)
        R = common.rotation_matrix(state.rotation)
        q, dq = common.vector(state.q, 2, 'q'), common.vector(state.dq, 2, 'dq')
        allocation = a.allocator.step(t, dt, state, target)
        if not isinstance(allocation, dict):
            raise ValueError('Allocator must return the common allocation dictionary')
        p_ref = common.vector(allocation['p_ref'], 3, 'allocation.p_ref')
        base = common.vector(allocation['base_velocity_cmd'], 3, 'allocation.base_velocity_cmd')
        joint = common.vector(allocation['qd_cmd'], 2, 'allocation.qd_cmd')
        nominal = common.vector(allocation['a_ref'], 3, 'allocation.a_ref')
        qdd = common.vector(allocation['qdd_ref'], 2, 'allocation.qdd_ref')
        alpha = a.observe(state)
        raw = allocation.get('diagnostics', {})
        if not isinstance(raw, dict):
            raise TypeError('Allocator diagnostics must be a dictionary')
        # Authoritative optimizer quality remains available before publication.
        # Borrowed descendants cannot cross the next step: finish() detaches
        # them before the worker's synchronous immutable binary handoff.
        command = ControlCommand.from_executed_velocity(
            state, base, joint, nominal_position=p_ref,
            acceleration_feedforward=np.zeros(3), position_gains=a.position_gains,
            diagnostics=dict(raw))
        self._pending = (allocation, q, dq, qdd, R, alpha, state.omega_body.copy(), nominal, stamp)
        a._last_task_t = float(t)
        return command

    def finish_diagnostics(self):
        if self._pending is None:
            raise RuntimeError('No pending diagnostic result')
        allocation, q, dq, qdd, R, alpha, omega, nominal, stamp = self._pending
        a = self.adapter
        ff = common.arm_reaction_feedforward(
            q, dq, qdd, model=a.model, R_world_body=R, omega_body=omega,
            alpha_body=alpha, base_acceleration_world=nominal,
            acceleration_limit=a.arm_acceleration_limit)
        combined = nominal + ff['acceleration_world']
        candidate, candidate_saturated = common.clip_norm(combined, a.total_acceleration_limit)
        diagnostic, owned = a._snapshot_diagnostics(allocation)
        diagnostic.update(
            allocation=owned, filtered_alpha_body=alpha.copy(),
            state_stamp_s=stamp, arm_feedforward=ff,
            nominal_acceleration_world=nominal.copy(),
            arm_acceleration_world=ff['acceleration_world'].copy(),
            combined_acceleration_before_limit_world=combined,
            candidate_acceleration_before_policy_world=candidate.copy(),
            candidate_acceleration_saturated=candidate_saturated,
            command_acceleration_world=np.zeros(3),
            acceleration_feedforward_policy='zero', acceleration_feedforward_applied=False,
            acceleration_saturated=False,
            feedforward_joint_acceleration_source='allocation.qdd_ref: bounded difference of executed joint velocity commands',
            feedforward_angular_acceleration_source='same measured body omega at distinct state.stamp_s; first-order .12 s default filter',
            feedforward_scope='Relative arm inertia plus fixed-body COM angular transport; acceleration force feedforward only',
            reaction_moment_applied=False)
        self._pending = None
        return diagnostic
