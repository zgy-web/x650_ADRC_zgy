"""Conservative, signed jerk-aware reference braking (NumPy only).

State update: a_next=(v_next-v)/h, q_next=q+h*v_next. No physical limit is
relaxed. A maximum step is part of the reference contract, not an assumed dt.
"""
from __future__ import annotations

import math
import numpy as np


def outward_peak_speed(velocity, acceleration, jerk):
    """Peak outward speed under immediately applying maximum braking jerk.

    Signed v<0 is already moving away, but positive acceleration can turn it
    back toward the limit. This is why max(v,0) cannot replace v in the sum.
    """
    return max(0., velocity + max(acceleration, 0.)**2/(2.*jerk))


def outward_stop_distance(velocity, acceleration, acceleration_limit, jerk):
    """Maximum *net outward displacement* under earliest continuous braking.

    Apply -jerk until a=-acceleration_limit, then hold that acceleration until
    velocity reaches zero. For v<0,a>0, a positive-speed excursion is possible;
    the initial inward displacement must be included. The maximum position
    occurs either now or at the later zero of velocity, so clamp distance at 0.
    This is a braking envelope, not a simulated physical plant.
    """
    v, a, limit, j = map(float, (velocity, acceleration, acceleration_limit, jerk))
    if not all(math.isfinite(x) for x in (v,a,limit,j)) or limit <= 0 or j <= 0:
        raise ValueError("Finite state and positive acceleration/jerk limits required")
    if outward_peak_speed(v,a,j) <= 0:
        return 0.
    # This function's contract requires the original acceleration envelope.
    if abs(a) > limit + 32*np.finfo(float).eps*max(1.,limit):
        raise ValueError("Acceleration outside stopping-envelope contract")
    ramp_time = max(0., (a+limit)/j)
    discriminant = max(0., a*a+2*j*v)
    zero_time = (a+math.sqrt(discriminant))/j
    if zero_time <= ramp_time:
        distance = v*zero_time + .5*a*zero_time**2 - j*zero_time**3/6
    else:
        ramp_velocity = v+a*ramp_time-.5*j*ramp_time**2
        distance = (v*ramp_time+.5*a*ramp_time**2-j*ramp_time**3/6
                    + max(0.,ramp_velocity)**2/(2*limit))
    return max(0.,distance)


def directional_violation(position, velocity, acceleration, velocity_limit,
                          acceleration_limit, jerk, position_bound,
                          max_step_s):
    """Nonpositive iff a state satisfies one outward-direction envelope.

    Transform lower-bound checks by negating p/v/a and lower position. The
    hmax*peak margin upper-bounds positive right-end quadrature error while
    braking with variable discrete steps no larger than hmax.
    """
    peak = outward_peak_speed(velocity,acceleration,jerk)
    violation = peak-velocity_limit
    if math.isfinite(position_bound):
        distance = outward_stop_distance(velocity,acceleration,acceleration_limit,jerk)
        # Compare small stopping distance to remaining clearance directly.
        # Adding both to a large absolute coordinate first can erase a tiny
        # outward violation at an exactly occupied position boundary.
        violation = max(violation, distance+max_step_s*peak-(position_bound-position))
    return violation


def _last_feasible_increasing(function, lower, upper):
    """Upper end of a monotone nondecreasing <=0 set, never widening bounds."""
    if function(lower) > 0:
        raise RuntimeError("Coordinator jerk-aware braking domain has no feasible velocity")
    if function(upper) <= 0:
        return upper
    lo,hi = lower,upper
    for _ in range(52):
        middle = (lo+hi)/2
        if middle == lo or middle == hi:
            break
        if function(middle) <= 0:
            lo = middle
        else:
            hi = middle
    return lo


def tighten_velocity_bounds(position, velocity, acceleration, dt,
                            velocity_limit, acceleration_limit, jerk_limit,
                            *, position_lower, position_upper, max_step_s=.05):
    """Bound next velocity with unchanged v/a/jerk and signed braking domains.

    The next-state upper envelope is monotone in next velocity, because both
    next velocity and next acceleration grow with that variable. The mirrored
    lower envelope is monotone in its negation. Scalar bisection thus retains
    all feasible velocities inside the original box, including a singleton.
    Empty domains and late/oversized updates remain explicit failures.
    """
    arrays = [np.asarray(x,float) for x in (position,velocity,acceleration,
        velocity_limit,acceleration_limit,jerk_limit,position_lower,position_upper)]
    p,v,a,vmax,amax,jmax,pmin,pmax = arrays
    if (p.ndim != 1 or any(x.shape != p.shape for x in arrays)
            or any(not np.isfinite(x).all() for x in arrays[:6])
            or np.isnan(pmin).any() or np.isnan(pmax).any()
            or np.any(vmax<=0) or np.any(amax<=0) or np.any(jmax<=0)
            or np.any(pmin>=pmax) or not math.isfinite(dt)
            or not 0 < dt <= max_step_s or not math.isfinite(max_step_s)):
        raise ValueError("Invalid braking state/limits or dt exceeds maximum reference step")
    alo = np.maximum(-amax,a-jmax*dt)
    ahi = np.minimum(amax,a+jmax*dt)
    lower = np.maximum(-vmax,v+alo*dt)
    upper = np.minimum(vmax,v+ahi*dt)
    for i in range(len(p)):
        if lower[i] > upper[i]:
            raise RuntimeError("Coordinator original speed/acceleration/jerk bounds became infeasible")
        def upper_violation(x):
            return directional_violation(p[i]+dt*x,x,(x-v[i])/dt,
                vmax[i],amax[i],jmax[i],pmax[i],max_step_s)
        def mirrored_lower_violation(x):
            return directional_violation(-p[i]+dt*x,x,(x+v[i])/dt,
                vmax[i],amax[i],jmax[i],-pmin[i],max_step_s)
        upper[i] = _last_feasible_increasing(upper_violation,lower[i],upper[i])
        lower[i] = -_last_feasible_increasing(mirrored_lower_violation,-upper[i],-lower[i])
        if lower[i] > upper[i]:
            raise RuntimeError("Coordinator speed/acceleration/jerk/position bounds became infeasible")
    return lower,upper
