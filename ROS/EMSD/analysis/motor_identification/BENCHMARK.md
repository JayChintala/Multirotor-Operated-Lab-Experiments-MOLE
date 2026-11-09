# First- and second-order benchmark

The recorded-current plots use the supplied workbook current, linearly interpolated between measurements. Both models start in equilibrium at the first measured current and include their fitted output offsets. Thrust is positive N; signed torque is N m.

| Output | Model | RMSE | MAE | Max absolute error | R² |
|---|---|---:|---:|---:|---:|
| thrust (N) | First order | 1.21096702 | 1.08026936 | 2.35943755 | 0.961808212 |
| thrust (N) | Second order | 1.21085686 | 1.080194 | 2.35932748 | 0.96181516 |
| torque (N m) | First order | 0.0181183833 | 0.0159933872 | 0.0347588356 | 0.960962597 |
| torque (N m) | Second order | 0.0181165868 | 0.0159922036 | 0.0347572767 | 0.960970338 |

## Interpretation

- Thrust: second order reduces training RMSE by 0.00910%. Maximum difference at measured timestamps: 0.0053577595 N.
- Torque: second order reduces training RMSE by 0.00992%. Maximum difference at measured timestamps: 7.9474414e-05 N m.

The second-order models are numerically slightly better on the training sweep. The improvement is too small to support choosing their extra dynamics from this dataset. For a compact approximation, prefer the first-order model over the second-order fit, while recognizing that its 1 ms time constant is an imposed fit bound, not a measured response time. The static gains remain the most directly supported result.

The separate +1 A step plots expose the very different predicted transients: the second-order fit predicts strong ringing, whereas the first-order fit rises quickly without overshoot. These are synthetic predictions around equilibrium, not measured step responses. Faster independent step data are needed to decide which transient shape describes the motor.

Files: `benchmark_recorded_current.png/.pdf`, `benchmark_step_response.png/.pdf`, `benchmark_metrics.csv`, `benchmark_predictions.csv`, and `benchmark_step_response.csv`.

Reproduce with `python3 tools/plot_motor_benchmarks.py`.
