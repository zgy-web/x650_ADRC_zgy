# Original-row feasibility regression

`hold16_round1416_qp.npz` is the unmodified quadratic program captured at the
rejected round 1416 of the isolated full endpoint-hold experiment. It contains
the Hessian, gradient, all two-sided constraint rows, limits, warm plan, control
scale, old returned solution, and the original iteration bound. It contains no
simulator communication or credentials. SHA-256:

`b0588307e0901093ed25b9115b9ca4818a7f9009737bc7ffa10f733ae1a40946`

The original library (`773241c1…`) reproduces the archived solution exactly.
There are 75 decision variables and 230 two-sided rows, which become 610
inequalities after including the normalized control bounds. Its last violated
inequality, index 453 in `[C, -C, I, -I]` order, has a whitened row norm of
`4.981719910852118`. A remaining original-row violation of
`3.270557920181938e-7` becomes `6.56511802893092e-8` after row normalization.
The upstream termination test accepted this because its aggregate threshold
`610 * epsilon * 75^2 * 100` is `7.618905506490137e-8`. The independent original-row
check correctly rejected it against the existing `2e-7` limit. This was an
active-set convergence precision defect, not an infeasible MPC problem or an
unconstrained-center solution.

The correction retains the aggregate test and adds a per-row numerical target
of `1e-10` in the original inequality units, transformed by the same row norm.
The solver therefore performs the remaining active-set correction. It does not
delete constraints, alter objective weights or physical bounds, relax the
external gate, or use a previous-plan fallback. A search that cannot satisfy
both convergence tests fails closed. The returned plan still undergoes the
independent full-row quality and candidate checks.

`tests/test_native_optimization.py` checks this captured case and independently
checks analytic QPs at two dimensions, three positive row scales and both
constraint directions. The native source receipt includes this directory and
fixture so the numerical acceptance is bound to the tested input.

This numerical regression does not itself qualify a new flight or a hardware
deployment. The corrected backend requires the complete software regression,
recorded-call replay, fresh endpoint hold, and subsequent trajectory stages.
