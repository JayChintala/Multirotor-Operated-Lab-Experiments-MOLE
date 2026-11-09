"""Compare saved first/second order fits with the recorded current and unit steps.
Run after tools/fit_motor_models.py. Requires NumPy and Matplotlib.
"""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/emsd_matplotlib')
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'analysis/motor_identification'
models = json.loads((OUT / 'fit_results.json').read_text())['results']
data = np.genfromtxt(OUT / 'identified_samples.csv', delimiter=',', names=True)
t, u = data['elapsed_time_s'], data['current_a']
outputs = {'thrust': data['thrust_n_positive'], 'torque': data['torque_nm_signed']}
colors = {'order_1': '#0072B2', 'order_2': '#D55E00'}
labels = {'order_1': 'First order', 'order_2': 'Second order'}
units = {'thrust': 'N', 'torque': 'N m'}
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .2})


def simulate(ts, current, m, order):
    """Exact integration for linear interpolation of input; equilibrium start."""
    dt = np.diff(ts)
    def pole_response(p):
        a = np.expm1(p * dt)
        forcing = current[:-1] * a/p + np.diff(current)/dt * (a-p*dt)/(p*p)
        x = np.empty(len(ts), dtype=complex)
        x[0] = -current[0]/p
        for i in range(len(dt)):
            x[i+1] = (1+a[i])*x[i]+forcing[i]
        return x
    if order == 'order_1':
        tau = m['tau_s']
        v = (pole_response(-1/tau)/tau).real
    else:
        p1, p2 = np.roots(m['denominator'])
        v = (m['denominator'][-1]*(pole_response(p1)-pole_response(p2))/(p1-p2)).real
    return m['offset']+m['gain']*v


# Include every original timestamp to preserve all corners of the interpolated current.
dense_t = np.unique(np.r_[np.arange(0, t[-1], .002), t])
dense_u = np.interp(dense_t, t, u)
index = np.searchsorted(dense_t, t)
pred = {}
metrics = []
fig, axes = plt.subplots(4, 2, figsize=(15, 13), sharex=True)
for col, (name, y) in enumerate(outputs.items()):
    unit = units[name]
    axes[0,col].plot(t, u, 'o-', color='#444444', ms=3)
    axes[0,col].set(title=f'Current → {name}', ylabel='Measured current [A]')
    axes[1,col].plot(t, y, 'ko', ms=3.5, label='Measurements', zorder=5)
    for order in ['order_1', 'order_2']:
        m = models[name][order]
        curve = simulate(dense_t, dense_u, m, order)
        yp = curve[index]
        # Verify against the saved fit, independently reconstructed here.
        rmse = float(np.sqrt(np.mean((yp-y)**2)))
        assert np.isclose(rmse, m['rmse'], rtol=1e-6, atol=1e-10)
        pred[name,order] = yp
        metrics.append({'output':name, 'model':order, 'unit':unit, 'rmse':rmse,
                        'mae':float(np.mean(abs(yp-y))), 'max_abs_error':float(np.max(abs(yp-y))),
                        'r_squared':float(1-np.sum((yp-y)**2)/np.sum((y-y.mean())**2))})
        axes[1,col].plot(dense_t, curve, color=colors[order], ls='-' if order=='order_1' else '--',
                         lw=1.8, label=labels[order])
        axes[2,col].plot(t, yp-y, color=colors[order], ls='-' if order=='order_1' else '--',
                         marker='.', label=f'{labels[order]}: RMSE {rmse:.6g} {unit}')
    difference = pred[name,'order_2']-pred[name,'order_1']
    axes[3,col].plot(t, difference, color='#009E73', marker='.', ms=4)
    axes[3,col].set(ylabel=f'Second − first [{unit}]', xlabel='Elapsed time [s]',
                    title=f'Maximum sampled model difference: {np.max(abs(difference)):.6g} {unit}')
    axes[1,col].set(ylabel=f'{name.title()} [{unit}]', title='Recorded-current response (curves nearly overlap)')
    axes[2,col].set(ylabel=f'Prediction − measurement [{unit}]', title='Error at original measurement timestamps')
    axes[1,col].legend(); axes[2,col].legend(fontsize=9)
    for row in [2,3]: axes[row,col].axhline(0, color='k', lw=.7, alpha=.5)
fig.suptitle('First vs second order — benchmark against the recorded current sweep\n'
             'Same training data for both models; this is an in-sample comparison', fontsize=15)
fig.tight_layout(rect=(0,0,1,.95))
for ext in ['png','pdf']: fig.savefig(OUT/f'benchmark_recorded_current.{ext}', dpi=180)
plt.close(fig)

# An incremental step about a fitted equilibrium: offsets cancel. No absolute motor-off assumption.
step_t = np.linspace(0, .6, 12001)
step_curves = {}
fig, axes = plt.subplots(2,2,figsize=(13,8))
for row,name in enumerate(outputs):
    for order in ['order_1','order_2']:
        m = models[name][order]
        if order == 'order_1':
            response = m['gain'] * (-np.expm1(-step_t/m['tau_s']))
        else:
            wn,zeta = m['natural_frequency_rad_s'],m['damping_ratio']
            wd=wn*np.sqrt(1-zeta*zeta)
            response=m['gain']*(1-np.exp(-zeta*wn*step_t)*(np.cos(wd*step_t)+zeta*wn/wd*np.sin(wd*step_t)))
        step_curves[name,order]=response
        for col in [0,1]:
            axes[row,col].plot(step_t,response,color=colors[order],ls='-' if order=='order_1' else '--',label=labels[order])
        axes[row,0].axhline(m['gain'],color=colors[order],lw=.6,alpha=.6)
    for col in [0,1]:
        axes[row,col].set(xlabel='Time after +1 A step [s]',ylabel=f'Δ{name} [{units[name]}]',
                          title=f'{name.title()}: '+('full response' if col==0 else 'first 50 ms'))
        axes[row,col].legend()
    axes[row,1].set_xlim(0,.05)
fig.suptitle('Predicted response to an ideal +1 A current step about equilibrium\n'
             'Model extrapolation: these fast transients were not resolved in the measured sweep',fontsize=13)
fig.tight_layout(rect=(0,0,1,.92))
for ext in ['png','pdf']: fig.savefig(OUT/f'benchmark_step_response.{ext}',dpi=180)
plt.close(fig)
with (OUT/'benchmark_metrics.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(metrics[0]));writer.writeheader();writer.writerows(metrics)
with (OUT/'benchmark_predictions.csv').open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['elapsed_time_s','current_a','thrust_measured_n','thrust_first_order_n','thrust_second_order_n',
                     'torque_measured_nm','torque_first_order_nm','torque_second_order_nm'])
    writer.writerows(zip(t,u,outputs['thrust'],pred['thrust','order_1'],pred['thrust','order_2'],
                        outputs['torque'],pred['torque','order_1'],pred['torque','order_2']))
with (OUT/'benchmark_step_response.csv').open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['step_time_s','delta_thrust_first_order_n','delta_thrust_second_order_n',
                     'delta_torque_first_order_nm','delta_torque_second_order_nm'])
    writer.writerows(zip(step_t,step_curves['thrust','order_1'],step_curves['thrust','order_2'],
                        step_curves['torque','order_1'],step_curves['torque','order_2']))
lines=['# First- and second-order benchmark','',
       'The recorded-current plots use the supplied workbook current, linearly interpolated between measurements. '
       'Both models start in equilibrium at the first measured current and include their fitted output offsets. '
       'Thrust is positive N; signed torque is N m.','',
       '| Output | Model | RMSE | MAE | Max absolute error | R² |',
       '|---|---|---:|---:|---:|---:|']
for m in metrics:
    lines.append(f"| {m['output']} ({m['unit']}) | {labels[m['model']]} | {m['rmse']:.9g} | {m['mae']:.9g} | {m['max_abs_error']:.9g} | {m['r_squared']:.9g} |")
lines += ['','## Interpretation','']
for name in outputs:
    a,b=[m for m in metrics if m['output']==name]
    improvement=100*(a['rmse']-b['rmse'])/a['rmse']
    lines.append(f"- {name.title()}: second order reduces training RMSE by {improvement:.5f}%. Maximum difference at measured timestamps: {np.max(abs(pred[name,'order_2']-pred[name,'order_1'])):.8g} {units[name]}.")
lines += ['','The second-order models are numerically slightly better on the training sweep. '
          'The improvement is too small to support choosing their extra dynamics from this dataset. '
          'For a compact approximation, prefer the first-order model over the second-order fit, '
          'while recognizing that its 1 ms time constant is an imposed fit bound, not a measured response time. '
          'The static gains remain the most directly supported result.','',
          'The separate +1 A step plots expose the very different predicted transients: '
          'the second-order fit predicts strong ringing, whereas the first-order fit rises quickly without overshoot. '
          'These are synthetic predictions around equilibrium, not measured step responses. '
          'Faster independent step data are needed to decide which transient shape describes the motor.','',
          'Files: `benchmark_recorded_current.png/.pdf`, `benchmark_step_response.png/.pdf`, '
          '`benchmark_metrics.csv`, `benchmark_predictions.csv`, and `benchmark_step_response.csv`.','',
          'Reproduce with `python3 tools/plot_motor_benchmarks.py`.']
(OUT/'BENCHMARK.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
