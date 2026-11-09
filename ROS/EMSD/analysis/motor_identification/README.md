# Current-to-thrust and current-to-torque identification

Source: `motor_data.xlsx`, columns A (time, s), L (current, A), J (signed thrust, kgf), and I (signed torque, N m).

82 samples spanning 107.874344 s; median sample interval 1.349129 s. The first two timestamps are reversed in workbook order; fitting uses sorted timestamps and elapsed time starts at zero.

`extracted_measurements.csv` preserves the four requested columns in workbook order and original units. `identified_samples.csv` contains sorted data, elapsed time, original time, current, signed torque, positive thrust in N, and voltage. Positive thrust is defined as -9.80665 times the recorded kgf; torque sign is preserved.

## Recommended empirical models

These are global affine fits over the measured range, with current as input. Offsets belong to the output equations, not to transfer-function numerators. The transfer functions below describe deviations from a point on the fitted line.

| Output | Output equation | Delta-output / delta-current | RMSE | R² |
|---|---|---|---|---|
| Thrust [N] | T = 1.9522553 + 0.58184915 I | G_T(s) = 0.58184915 | 1.21093 N | 0.961811 |
| Torque [N m] | Q = 0.026380064 + 0.0086069898 I | G_Q(s) = 0.0086069898 | 0.0181176 N m | 0.960966 |

Measured current range: 0.0732529–34.5837 A. Do not interpret the fitted intercept as a measured motor-off output or extrapolate below the measured range.

## Exploratory dynamic fits

Stable first- and second-order continuous-time models were fitted using exact integration of piecewise-linear current at the original timestamps. Gain and offset were fitted by least squares, with an equilibrium initial state and no time delay. Multiple parameter starts were used. All fit scores are in-sample; no independent validation data were supplied.

- Thrust, first order: G_T(s) = 0.58185205 / (0.001 s + 1); offset = 1.9522079; RMSE = 1.210967.
- Thrust, second order: G_T(s) = 22156.788 / (s² + 20.189264 s + 38080.122); offset = 1.9523346; RMSE = 1.2108569.
- Torque, first order: G_Q(s) = 0.0086070284 / (0.001 s + 1); offset = 0.02637941; RMSE = 0.018118383.
- Torque, second order: G_Q(s) = 329.66804 / (s² + 20.106224 s + 38302.537); offset = 0.026381253; RMSE = 0.018116587.

Both first-order time constants hit the imposed 0.001 s lower bound, far below the typical sampling interval. Second-order fits improve error negligibly and their fast poles are unresolved by this recording. These dynamic equations are optimizer results, not identified physical motor dynamics; use the static gains as the supported approximation.

The sweep also changes voltage and has nonlinear steady-state behavior. Measured current co-varies with outputs under ESC commands, so this dataset does not establish an independently excited, causal current-to-output model. To identify useful time constants and delay, record repeated steps around a fixed operating point with faster synchronized current/thrust/torque measurements (at least 10–20 samples across the expected response), and reserve separate steps for validation.

## Reproduce

Run `python3 tools/fit_motor_models.py` from the project root with NumPy, SciPy, and Matplotlib installed. The script reads the workbook directly using Python’s standard library and writes all results to `analysis/motor_identification/`. See `fit_results.json` for full precision coefficients and `fit_comparison.png` for measured/fitted curves.
