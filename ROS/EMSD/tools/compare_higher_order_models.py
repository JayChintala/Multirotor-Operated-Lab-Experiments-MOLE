"""Explore stable real-pole LTI orders 3-6 versus nonlinear static current maps.
Validation: fit the rising sweep only; predict the complete falling sweep.
"""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/emsd_matplotlib')
from pathlib import Path
import json
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/motor_identification'
x=np.genfromtxt(OUT/'identified_samples.csv',delimiter=',',names=True)
t=x['elapsed_time_s']; u=x['current_a']; dt=np.diff(t)
peak=int(np.argmax(u)); train=np.arange(peak+1); test=np.arange(peak+1,len(t)); allrows=np.arange(len(t))
outputs={'thrust':x['thrust_n_positive'],'torque':x['torque_nm_signed']}


def basis(logtau):
    tau=np.exp(logtau)
    a=-np.expm1(-dt[:,None]/tau)
    forcing=u[:-1,None]*a + (np.diff(u)/dt)[:,None]*(dt[:,None]-tau*a)
    v=np.empty((len(t),len(tau)));v[0]=u[0]
    for k in range(len(dt)):v[k+1]=(1-a[k])*v[k]+forcing[k]
    return np.column_stack([np.ones(len(t)),v])


def dynamic(y,rows,n):
    def residual(p):
        b=basis(p)[rows]
        c=np.linalg.lstsq(b,y[rows],rcond=1e-10)[0]
        return (b@c-y[rows])/np.std(y[rows])
    starts=[np.geomspace(.005,120,n),np.geomspace(.2,250,n)]
    fits=[least_squares(residual,np.log(z),bounds=(np.log(.001),np.log(500)),
                        max_nfev=220,diff_step=1e-5) for z in starts]
    fit=min(fits,key=lambda z:np.sum(z.fun**2)); b=basis(fit.x)
    c=np.linalg.lstsq(b[rows],y[rows],rcond=1e-10)[0]
    taus=np.exp(fit.x); poles=-1/taus
    den=np.poly(poles)
    num=np.zeros(n)
    for j,gain in enumerate(c[1:]): num+=gain/taus[j]*np.poly(np.delete(poles,j))
    info={'type':'stable strictly proper LTI with real poles','order':n,'time_constants_s':taus.tolist(),
          'parallel_gains':c[1:].tolist(),'offset':float(c[0]),'numerator':num.tolist(),
          'denominator':den.tolist(),'converged':bool(fit.success),
          'basis_condition_number':float(np.linalg.cond(b[rows])),
          'at_tau_bound':bool(np.any(taus<.00102) or np.any(taus>490))}
    return b@c,info


def polynomial(y,rows,degree):
    # Scaling current keeps polynomial regression well conditioned.
    z=u/u.max(); b=np.vander(z,degree+1,increasing=True)
    c=np.linalg.lstsq(b[rows],y[rows],rcond=None)[0]
    return b@c,{'type':'static polynomial','degree':degree,
               'coefficients_ascending_current_powers':(c/u.max()**np.arange(degree+1)).tolist()}


report={'method':'Orders 3-6 use parallel first-order sections with independently fitted negative real poles, signed gains and output offset. Exact piecewise-linear input integration; equilibrium start. Not an exhaustive search of all higher-order LTI structures.',
        'validation':f'Fit first {len(train)} rising-sweep samples; evaluate remaining {len(test)} falling-sweep samples. No held-out output used for estimation. Current remains an observed input throughout.',
        'caveat':'Single experiment and one directional holdout. Final all-data fits are exploratory; dynamic parameters may be ill-conditioned or unresolved.',
        'results':{}}
predictions={}
for name,y in outputs.items():
    report['results'][name]={}
    for label,kind,order in [('LTI order '+str(n),'dynamic',n) for n in range(3,7)]+[('Static degree '+str(n),'static',n) for n in range(1,5)]:
        method=dynamic if kind=='dynamic' else polynomial
        full,info=method(y,allrows,order)
        holdout,holdinfo=method(y,train,order)
        err=full-y; he=holdout[test]-y[test]
        item={'all_data_rmse':float(np.sqrt(np.mean(err**2))),
              'all_data_r_squared':float(1-np.sum(err**2)/np.sum((y-y.mean())**2)),
              'return_sweep_rmse':float(np.sqrt(np.mean(he**2))),
              'all_data_model':info,'rising_sweep_model':holdinfo}
        report['results'][name][label]=item
        predictions[name,label]=(full,holdout)
        print(name,label,'fit RMSE',item['all_data_rmse'],'holdout RMSE',item['return_sweep_rmse'],flush=True)
(OUT/'higher_order_results.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axes=plt.subplots(2,2,figsize=(15,10),sharex=True)
for col,(name,y) in enumerate(outputs.items()):
    unit='N' if name=='thrust' else 'N m'
    axes[0,col].plot(t,y,'ko',ms=3,label='Measured',zorder=5)
    for row in [0,1]:
        axes[row,col].set(ylabel=f'{name.title()} [{unit}]',xlabel='Elapsed time [s]')
        axes[row,col].grid(alpha=.2)
    bestlti=min((key for key in report['results'][name] if key.startswith('LTI')),key=lambda key:report['results'][name][key]['return_sweep_rmse'])
    for label in [bestlti,'Static degree 1','Static degree 2','Static degree 3','Static degree 4']:
        full,hold=predictions[name,label]
        axes[0,col].plot(t,full,label=label,lw=1.5)
        axes[1,col].semilogy(t[test],np.maximum(abs(hold[test]-y[test]),1e-8),label=label,lw=1.5)
    axes[0,col].set_title(f'{name.title()}: fitted on all 82 samples')
    axes[1,col].set_title(f'{name.title()}: return-sweep prediction error (lower is better)')
    axes[1,col].set_ylabel(f'Absolute error [{unit}], log scale')
    axes[1,col].axvline(t[peak],color='gray',ls=':')
    axes[1,col].axvspan(t[peak],t[-1],color='green',alpha=.045)
    for row in [0,1]:axes[row,col].legend(fontsize=9)
fig.suptitle('Does more model complexity help?\nTop: training fit. Bottom: independent return-sweep prediction.',fontsize=14)
fig.tight_layout(rect=(0,0,1,.93))
for ext in ['png','pdf']:fig.savefig(OUT/f'higher_order_comparison.{ext}',dpi=170)
lines=['# Higher-order and nonlinear model comparison','',report['method'],'',report['validation'],'',
       '| Output | Model | All-data RMSE | Return-sweep RMSE |','|---|---|---:|---:|']
for name,result in report['results'].items():
    for label,r in result.items():lines.append(f"| {name} | {label} | {r['all_data_rmse']:.8g} | {r['return_sweep_rmse']:.8g} |")
lines+=['','Thrust errors are in N and torque errors in N m. Return-sweep RMSE is from separately fitted models trained only on the rising sweep. All-data RMSE uses a different fit trained on all samples.','',
        '## Nonlinear equations (all-data fits)','']
for name in outputs:
    for degree in [2,3]:
        m=report['results'][name][f'Static degree {degree}']['all_data_model']
        c=m['coefficients_ascending_current_powers']
        lines.append(f"- {name}, degree {degree}: y = "+' + '.join(f'({v:.10g}) I^{j}' for j,v in enumerate(c)))
lines+=['','Nonlinear static maps are not transfer functions. Their local small-signal gain at a chosen current I0 is the derivative of the fitted polynomial at I0.','',report['caveat'],'',
        'Reproduce: `python3 tools/compare_higher_order_models.py`. Full coefficients and pole diagnostics are in `higher_order_results.json`.']
(OUT/'HIGHER_ORDER.md').write_text('\n'.join(lines)+'\n')
