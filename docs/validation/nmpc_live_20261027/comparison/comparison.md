# Upper-controller paired simulation observations

One fresh run per backend and case (n=1). All six independent audits passed. These observations do not establish superiority or statistical significance.

Errors use the original physical tip and same-time world-frame reference. Tracking includes the original smooth phase ramps; the final four seconds are the tail hold.

| Case | Window [s] | MPC RMSE [mm] | NMPC RMSE [mm] | Difference [%] | MPC peak [mm] | NMPC peak [mm] |
|---|---:|---:|---:|---:|---:|---:|
| hold / endpoint | 0–30 | 2.988 | 2.533 | -15.251 | 4.730 | 4.361 |
| normal / endpoint | 0–32 | 4.099 | 4.152 | 1.297 | 8.173 | 7.772 |
| normal / tracking_endpoint | 2–28 | 4.332 | 4.423 | 2.095 | 8.173 | 7.772 |
| normal / tail_endpoint | 28–32 | 1.909 | 1.690 | -11.468 | 3.469 | 3.288 |
| fast / endpoint | 0–24 | 6.865 | 7.091 | 3.294 | 12.108 | 12.274 |
| fast / tracking_endpoint | 2–20 | 7.741 | 7.986 | 3.164 | 12.108 | 12.274 |
| fast / tail_endpoint | 20–24 | 1.890 | 2.208 | 16.817 | 4.323 | 5.003 |

Difference = 100 × (NMPC − MPC) / MPC. Negative means lower in this single observed pair.

| Case | MPC core p95 / max [ms] | NMPC core p95 / max [ms] | MPC gate max [ms] | NMPC gate max [ms] |
|---|---:|---:|---:|---:|
| hold | 10.122 / 13.500 | 25.571 / 30.451 | 13.944 | 31.124 |
| normal | 13.331 / 16.981 | 29.365 / 38.581 | 17.404 | 39.260 |
| fast | 16.642 / 19.596 | 32.609 / 41.497 | 22.205 | 41.918 |

Task hashes differ because backend and stage qualification identities differ. The physical model, model parameters, PID, effective PX4 parameters, numerical libraries, worker, timing profile, and declared paired trajectories were checked separately.

Recorded target centres are retained in comparison.json. Each run uses its original centre; measured traces are never translated or phase shifted to reduce the error.
