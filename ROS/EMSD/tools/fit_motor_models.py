"""Exploratory continuous-time current-to-output fits of the measured step sweep.
Uses piecewise-linear current, equilibrium initial state, affine output offset,
and unweighted least squares at original timestamps. Not validated motor dynamics.
"""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/emsd_matplotlib')
from pathlib import Path
import csv
import json
import numpy as np
from scipy.optimize import least_squares
from import_motor_data import read_workbook
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/motor_identification'
OUT.mkdir(parents=True, exist_ok=True)
workbook_rows = read_workbook()
raw = np.array([tuple(row) for row in workbook_rows], dtype=[(name, float) for name in
    ('time_s', 'esc_signal', 'torque_nm_signed', 'thrust_kgf_signed', 'voltage_v', 'current_a', 'electrical_rpm')])
with (OUT/'extracted_measurements.csv').open('w', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(['time_s', 'current_a', 'thrust_kgf_signed', 'torque_nm_signed'])
    writer.writerows((r['time_s'], r['current_a'], r['thrust_kgf_signed'], r['torque_nm_signed']) for r in raw)
order = np.argsort(raw['time_s'], kind='stable')
t = raw['time_s'][order]; t = t - t[0]
u = raw['current_a'][order]
# Preserve torque sign so the measured low-load offset is not rectified.
outputs = {'torque': raw['torque_nm_signed'][order],
           'thrust': -raw['thrust_kgf_signed'][order] * 9.80665}
if not np.all(np.isfinite(raw.view(float))) or not np.all(np.diff(t) > 0):
    raise ValueError('Measurements must be finite with distinct timestamps')


def pole_response(pole):
    # Exact response of 1/(s-pole) to piecewise-linear current.
    dt = np.diff(t)
    step = np.expm1(pole*dt)
    e = step + 1
    ramp = (step - pole*dt) / (pole*pole)
    forcing = u[:-1]*step/pole + np.diff(u)/dt*ramp
    response = np.empty(len(t), dtype=complex)
    response[0] = -u[0]/pole  # equilibrium at first measured input
    for i in range(len(dt)):
        response[i+1] = e[i]*response[i]+forcing[i]
    return response


def filtered(logp, model):
    if model == 1:
        tau = np.exp(logp[0])
        return (pole_response(-1/tau)/tau).real
    wn, zeta = np.exp(logp)
    root = np.sqrt(complex(zeta*zeta-1))
    p1, p2 = wn*(-zeta+root), wn*(-zeta-root)
    # Evaluate repeated poles by a small symmetric separation.
    if abs(p1-p2) < wn*1e-6:
        p1, p2 = -wn*(1-1e-6), -wn*(1+1e-6)
    return (wn*wn*(pole_response(p1)-pole_response(p2))/(p1-p2)).real


def regress(v, y):
    gain, offset = np.linalg.lstsq(np.column_stack([v, np.ones_like(v)]), y, rcond=None)[0]
    return gain, offset, gain*v+offset


results = {}
predictions = {}
for name, y in outputs.items():
    gain, offset, pred = regress(u, y)
    results[name] = {'static': {'gain': gain, 'offset': offset,
                               'rmse': float(np.sqrt(np.mean((pred-y)**2))),
                               'r_squared': float(1-np.sum((pred-y)**2)/np.sum((y-y.mean())**2)),
                               'numerator': [float(gain)], 'denominator': [1.0]}}
    predictions[name] = {'static': pred}
    for model in (1, 2):
        starts = [[v] for v in (.003,.1,1,10,100)] if model == 1 else [[w,z] for w,z in [(0.1,.7),(1,.7),(10,.7),(100,.7),(1,3),(100,3)]]
        lower, upper = ([.001], [300]) if model == 1 else ([.003,.05], [1000,20])
        def residual(p):
            v = filtered(p, model)
            return (regress(v, y)[2]-y) / np.std(y)
        fits = [least_squares(residual, np.log(start), bounds=(np.log(lower), np.log(upper)),
                              max_nfev=1000, diff_step=1e-6) for start in starts]
        best = min(fits, key=lambda f: np.sum(f.fun**2))
        params = np.exp(best.x)
        gain, offset, prediction = regress(filtered(best.x, model), y)
        error = prediction-y
        rmse = float(np.sqrt(np.mean(error**2)))
        item = {'gain': float(gain), 'offset': float(offset), 'rmse': rmse,
                'r_squared': float(1-np.sum(error**2)/np.sum((y-y.mean())**2)),
                'parameters': params.tolist(), 'converged': bool(best.success),
                'at_parameter_bound': bool(np.any(np.isclose(best.x, np.log(lower), atol=.02)) or np.any(np.isclose(best.x, np.log(upper), atol=.02)))}
        if model == 1:
            tau = params[0]
            item.update(numerator=[float(gain)], denominator=[float(tau),1.0], tau_s=float(tau))
        else:
            wn, zeta = params
            item.update(numerator=[float(gain*wn**2)], denominator=[1.0,float(2*zeta*wn),float(wn**2)],
                        natural_frequency_rad_s=float(wn), damping_ratio=float(zeta))
        results[name][f'order_{model}'] = item
        predictions[name][f'order_{model}'] = prediction
        print(name,model,json.dumps(item),flush=True)
report = {'source': 'motor_data.xlsx', 'input': 'measured current [A]', 'sample_count':len(t),
          'duration_s':float(t[-1]), 'median_sample_interval_s':float(np.median(np.diff(t))),
          'minimum_sample_interval_s':float(np.min(np.diff(t))),
          'maximum_sample_interval_s':float(np.max(np.diff(t))),
          'timestamp_note':'First two workbook timestamps decrease; samples sorted by timestamp before fitting.',
          'method':'Exact piecewise-linear input integration at measured timestamps. Affine stable continuous-time models; linearly interpolated current; equilibrium initial state; unweighted least squares; no delay; all samples used for training.',
          'caveat':'Exploratory empirical fits, not validated causal motor dynamics. Sparse step-sweep samples, voltage sag, input/output co-variation and nonlinear steady-state behavior limit identification.',
          'recommended_model': 'Static affine regression. Dynamic poles are unresolved: first-order time constants hit the 0.001 s lower bound; second-order fits provide negligible improvement.',
          'results':results}
(OUT/'fit_results.json').write_text(json.dumps(report,indent=2)+'\n')
lines = [
    '# Current-to-thrust and current-to-torque identification', '',
    'Source: `motor_data.xlsx`, columns A (time, s), L (current, A), '
    'J (signed thrust, kgf), and I (signed torque, N m).', '',
    f'{len(t)} samples spanning {t[-1]:.6f} s; median sample interval '
    f'{np.median(np.diff(t)):.6f} s. The first two timestamps are reversed in '
    'workbook order; fitting uses sorted timestamps and elapsed time starts at zero.', '',
    '`extracted_measurements.csv` preserves the four requested columns in workbook '
    'order and original units. `identified_samples.csv` contains sorted data, elapsed '
    'time, original time, current, signed torque, positive thrust in N, and voltage. '
    'Positive thrust is defined as -9.80665 times the recorded kgf; torque sign is preserved.', '',
    '## Recommended empirical models', '',
    'These are global affine fits over the measured range, with current as input. '
    'Offsets belong to the output equations, not to transfer-function numerators. '
    'The transfer functions below describe deviations from a point on the fitted line.', '',
    '| Output | Output equation | Delta-output / delta-current | RMSE | R² |',
    '|---|---|---|---|---|',
]
for name, symbol, unit in [('thrust', 'T', 'N'), ('torque', 'Q', 'N m')]:
    r = results[name]['static']
    lines.append(f"| {name.title()} [{unit}] | {symbol} = {r['offset']:.8g} + {r['gain']:.8g} I | G_{symbol}(s) = {r['gain']:.8g} | {r['rmse']:.6g} {unit} | {r['r_squared']:.6f} |")
lines += ['', f"Measured current range: {u.min():.6g}–{u.max():.6g} A. "
          'Do not interpret the fitted intercept as a measured motor-off output or extrapolate below the measured range.', '',
          '## Exploratory dynamic fits', '',
          'Stable first- and second-order continuous-time models were fitted using '
          'exact integration of piecewise-linear current at the original timestamps. '
          'Gain and offset were fitted by least squares, with an equilibrium initial '
          'state and no time delay. Multiple parameter starts were used. All fit '
          'scores are in-sample; no independent validation data were supplied.', '']
for name, symbol in [('thrust', 'T'), ('torque', 'Q')]:
    r1, r2 = results[name]['order_1'], results[name]['order_2']
    d = r2['denominator']
    lines += [f"- {name.title()}, first order: G_{symbol}(s) = {r1['gain']:.8g} / ({r1['tau_s']:.8g} s + 1); offset = {r1['offset']:.8g}; RMSE = {r1['rmse']:.8g}.",
              f"- {name.title()}, second order: G_{symbol}(s) = {r2['numerator'][0]:.8g} / (s² + {d[1]:.8g} s + {d[2]:.8g}); offset = {r2['offset']:.8g}; RMSE = {r2['rmse']:.8g}."]
lines += ['', 'Both first-order time constants hit the imposed 0.001 s lower bound, '
          'far below the typical sampling interval. Second-order fits improve error '
          'negligibly and their fast poles are unresolved by this recording. '
          'These dynamic equations are optimizer results, not identified physical '
          'motor dynamics; use the static gains as the supported approximation.', '',
          'The sweep also changes voltage and has nonlinear steady-state behavior. '
          'Measured current co-varies with outputs under ESC commands, so this dataset '
          'does not establish an independently excited, causal current-to-output model. '
          'To identify useful time constants and delay, record repeated steps around '
          'a fixed operating point with faster synchronized current/thrust/torque '
          'measurements (at least 10–20 samples across the expected response), and '
          'reserve separate steps for validation.', '',
          '## Reproduce', '', 'Run `python3 tools/fit_motor_models.py` from the project '
          'root with NumPy, SciPy, and Matplotlib installed. The script reads the '
          'workbook directly using Python’s standard library and writes all results '
          'to `analysis/motor_identification/`. See `fit_results.json` for full '
          'precision coefficients and `fit_comparison.png` for measured/fitted curves.', '']
(OUT/'README.md').write_text('\n'.join(lines))
with (OUT/'identified_samples.csv').open('w',newline='') as stream:
    writer=csv.writer(stream)
    writer.writerow(['elapsed_time_s','workbook_time_s','current_a','torque_nm_signed','thrust_n_positive','voltage_v'])
    writer.writerows(zip(t,raw['time_s'][order],u,outputs['torque'],outputs['thrust'],raw['voltage_v'][order]))
fig, axes=plt.subplots(3,2,figsize=(13,10))
axes[0,0].plot(t,u,'.-'); axes[0,0].set(xlabel='Time [s]',ylabel='Current [A]',title='Measured input (sparse step sweep)')
axes[0,1].plot(t,raw['voltage_v'][order],'.-');axes[0,1].set(xlabel='Time [s]',ylabel='Voltage [V]',title='Supply voltage varies during sweep')
for row,(name,y) in enumerate(outputs.items(),1):
    unit='N m' if name=='torque' else 'N'
    axes[row,0].plot(t,y,'k.',label='Measured')
    axes[row,1].scatter(u,y,c=t,cmap='viridis',s=18,label='Measured')
    for key, style in [('static', ':'),('order_1','-'),('order_2','--')]:
        axes[row,0].plot(t,predictions[name][key],style,label=key)
    g=results[name]['static']; xs=np.linspace(min(u),max(u),100)
    axes[row,1].plot(xs,g['offset']+g['gain']*xs,'r:',label='Static linear fit')
    axes[row,0].set(xlabel='Time [s]',ylabel=f'{name.title()} [{unit}]',title=f'{name.title()}: in-sample fit')
    axes[row,1].set(xlabel='Current [A]',ylabel=f'{name.title()} [{unit}]',title='Steady-state nonlinearity / sweep hysteresis')
    axes[row,0].legend();axes[row,1].legend()
for ax in axes.flat: ax.grid(alpha=.25)
fig.suptitle('Exploratory fits — sparse sweep does not validate dynamic poles')
fig.tight_layout();fig.savefig(OUT/'fit_comparison.png',dpi=160)
