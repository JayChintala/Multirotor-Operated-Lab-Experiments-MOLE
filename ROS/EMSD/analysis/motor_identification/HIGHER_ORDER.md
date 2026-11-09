# Higher-order and nonlinear model comparison

Orders 3-6 use parallel first-order sections with independently fitted negative real poles, signed gains and output offset. Exact piecewise-linear input integration; equilibrium start. Not an exhaustive search of all higher-order LTI structures.

Fit first 41 rising-sweep samples; evaluate remaining 41 falling-sweep samples. No held-out output used for estimation. Current remains an observed input throughout.

| Output | Model | All-data RMSE | Return-sweep RMSE |
|---|---|---:|---:|
| thrust | LTI order 3 | 1.0175125 | 68.886569 |
| thrust | LTI order 4 | 0.86433899 | 41.130155 |
| thrust | LTI order 5 | 0.86347187 | 35.181062 |
| thrust | LTI order 6 | 0.8609846 | 32.06581 |
| thrust | Static degree 1 | 1.2109302 | 1.1773392 |
| thrust | Static degree 2 | 0.40473507 | 0.46496939 |
| thrust | Static degree 3 | 0.19237191 | 0.27013597 |
| thrust | Static degree 4 | 0.14140561 | 0.23690609 |
| torque | LTI order 3 | 0.015485255 | 1.2547729 |
| torque | LTI order 4 | 0.012936777 | 0.73322945 |
| torque | LTI order 5 | 0.01291948 | 0.72423571 |
| torque | LTI order 6 | 0.012900512 | 0.69259432 |
| torque | Static degree 1 | 0.01811762 | 0.017484176 |
| torque | Static degree 2 | 0.0068021177 | 0.0078144543 |
| torque | Static degree 3 | 0.0037065282 | 0.0052491008 |
| torque | Static degree 4 | 0.0028177501 | 0.0046670526 |

Thrust errors are in N and torque errors in N m. Return-sweep RMSE is from separately fitted models trained only on the rising sweep. All-data RMSE uses a different fit trained on all samples.

## Nonlinear equations (all-data fits)

- thrust, degree 2: y = (0.7591985557) I^0 + (0.9477813225) I^1 + (-0.01224249431) I^2
- thrust, degree 3: y = (0.3763459478) I^0 + (1.187445056) I^1 + (-0.03292482139) I^2 + (0.0004322942724) I^3
- torque, degree 2: y = (0.008826150595) I^0 + (0.01399109401) I^1 + (-0.0001801286392) I^2
- torque, degree 3: y = (0.002694025629) I^0 + (0.01782977222) I^1 + (-0.0005113961077) I^2 + (6.924028846e-06) I^3

Nonlinear static maps are not transfer functions. Their local small-signal gain at a chosen current I0 is the derivative of the fitted polynomial at I0.

Single experiment and one directional holdout. Final all-data fits are exploratory; dynamic parameters may be ill-conditioned or unresolved.

Reproduce: `python3 tools/compare_higher_order_models.py`. Full coefficients and pole diagnostics are in `higher_order_results.json`.
