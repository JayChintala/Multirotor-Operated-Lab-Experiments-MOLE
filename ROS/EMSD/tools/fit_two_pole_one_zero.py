"""Fit G(s)=(b1*s+b0)/(s^2+a1*s+a0), with stable poles and a free zero.
Exact continuous-time response to linearly interpolated measured current,
equilibrium initial state, independent affine output offset, multistart fits.
"""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/emsd_matplotlib')
from pathlib import Path
import json,csv,io,re
import numpy as np
from scipy.optimize import least_squares
from scipy.linalg import expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/motor_identification/two_pole_one_zero';OUT.mkdir(exist_ok=True)
x=np.genfromtxt(OUT.parent/'identified_samples.csv',delimiter=',',names=True)
t=x['elapsed_time_s'];u=x['current_a'];dt=np.diff(t)
outputs={'torque':x['torque_nm_signed'],'thrust':x['thrust_n_positive']}
prior=json.loads((OUT.parent/'fit_results.json').read_text())['results']
peak=int(np.argmax(u));train=np.arange(peak+1);test=np.arange(peak+1,len(t));allrows=np.arange(len(t))
low=np.log([.003,.05]);high=np.log([1000,20])


def pole_response(p):
    a=np.expm1(p*dt)
    forcing=u[:-1]*a/p+np.diff(u)/dt*(a-p*dt)/(p*p)
    v=np.empty(len(t),dtype=complex);v[0]=-u[0]/p
    for k in range(len(dt)):v[k+1]=(1+a[k])*v[k]+forcing[k]
    return v


def basis(p,zero=True):
    wn,zeta=np.exp(p)
    root=np.sqrt(complex(zeta*zeta-1))
    p1,p2=wn*(-zeta+root),wn*(-zeta-root)
    if abs(p1-p2)<wn*1e-6:p1,p2=-wn*(1-1e-6),-wn*(1+1e-6)
    v1,v2=pole_response(p1),pole_response(p2)
    pos=(wn*wn*(v1-v2)/(p1-p2)).real
    vel=(wn*(p1*v1-p2*v2)/(p1-p2)).real
    return np.column_stack([np.ones(len(t)),pos,vel]) if zero else np.column_stack([np.ones(len(t)),pos])


def fit(y,rows,zero=True):
    def solve(p):
        b=basis(p,zero)
        c=np.linalg.lstsq(b[rows],y[rows],rcond=1e-12)[0]
        return c,b@c
    def residual(p):return (solve(p)[1][rows]-y[rows])/np.std(y[rows])
    starts=[(wn,zeta) for wn in [.006,.02,.07,.2,1,10,100,400] for zeta in [.1,.7,3,12]]
    fits=[least_squares(residual,np.log(s),bounds=(low,high),max_nfev=600,diff_step=1e-5,
                        ftol=1e-10,xtol=1e-10,gtol=1e-9) for s in starts]
    best=min(fits,key=lambda f:np.sum(f.fun**2))
    wn,zeta=np.exp(best.x);c,pred=solve(best.x)
    numerator=[float(c[2]*wn),float(c[1]*wn*wn)] if zero else [float(c[1]*wn*wn)]
    den=[1.,float(2*zeta*wn),float(wn*wn)]
    poles=np.roots(den);zeros=np.roots(numerator)
    err=pred[rows]-y[rows]
    metrics={'rmse':float(np.sqrt(np.mean(err**2))),
             'r_squared':float(1-np.sum(err**2)/np.sum((y[rows]-y[rows].mean())**2)),
             'nrmse_fit_percent':float(100*(1-np.linalg.norm(err)/np.linalg.norm(y[rows]-y[rows].mean()))),
             'mae':float(np.mean(abs(err)))}
    info={'numerator':numerator,'denominator':den,'offset':float(c[0]),'dc_gain':float(c[1]),
          'wn_rad_s':float(wn),'zeta':float(zeta),
          'poles_real_imag':[[float(z.real),float(z.imag)] for z in poles],
          'zeros_real_imag':[[float(z.real),float(z.imag)] for z in zeros],
          'converged':bool(best.success),'at_parameter_bound':bool(np.any(abs(best.x-low)<.02) or np.any(abs(best.x-high)<.02)),
          'basis_condition_number':float(np.linalg.cond(basis(best.x,zero)[rows])),
          'multistart_count':len(starts),**metrics}
    return pred,info


report={'model':'G(s)=(b1 s+b0)/(s^2+a1 s+a0), plus separately fitted output offset',
        'source':'motor_data.xlsx via identified_samples.csv','sample_count':len(t),
        'method':'Stable continuous-time poles; zero unrestricted (LHP or RHP). Exact piecewise-linear input integration at measured timestamps. Equilibrium initial state. Affine least squares for numerator and offset; multistart nonlinear least squares for wn and zeta. Bounds: wn 0.003–1000 rad/s, zeta 0.05–20.',
        'fit_definition':'NRMSE fit (%) = 100*(1-||y-yhat||_2/||y-mean(y)||_2); R_squared = 1-SSE/SST.',
        'validation':'Train first 41 rising-sweep samples; predict remaining 41 falling-sweep samples using observed current throughout, with no held-out output used in estimation.',
        'results':{}}
predictions={}
for name,y in outputs.items():
    full,m=fit(y,allrows,True)
    held,hm=fit(y,train,True)
    baseline,bm=fit(y,allrows,False)
    bh,bhm=fit(y,train,False)
    m['rmse_improvement_vs_prior_2p0z_percent']=100*(1-m['rmse']/prior[name]['order_2']['rmse'])
    m['rmse_improvement_vs_refitted_2p0z_percent']=100*(1-m['rmse']/bm['rmse'])
    item={'two_pole_one_zero':m,'rising_sweep_model':hm,'refitted_two_pole_no_zero':bm,
          'return_sweep_rmse':float(np.sqrt(np.mean((held[test]-y[test])**2))),
          'refitted_two_pole_no_zero_return_sweep_rmse':float(np.sqrt(np.mean((bh[test]-y[test])**2)))}
    report['results'][name]=item;predictions[name]=(full,held,baseline,bh)
    print(name,json.dumps(item),flush=True)
(OUT/'fit_results.json').write_text(json.dumps(report,indent=2)+'\n')

# Verify the complete response through an independent augmented state-space exponential.
# x1 is the normalized second-order filter output and x2 its time derivative.
for name,item in report['results'].items():
    m=item['two_pole_one_zero'];a1,a0=m['denominator'][1:];b1,b0=m['numerator']
    state=np.array([u[0],0.,u[0],0.]);actual=[]
    A=np.array([[0,1,0,0],[-a0,-a1,a0,0],[0,0,0,1],[0,0,0,0]],dtype=float)
    actual.append(m['offset']+(b0*state[0]+b1*state[1])/a0)
    for k,d in enumerate(dt):
        state[2]=u[k];state[3]=(u[k+1]-u[k])/d
        state=expm(A*d)@state
        actual.append(m['offset']+(b0*state[0]+b1*state[1])/a0)
    delta=float(np.max(abs(np.asarray(actual)-predictions[name][0])))
    assert np.allclose(actual,predictions[name][0],rtol=1e-5,atol=1e-7),(name,delta)
    assert np.all(np.real(np.roots(m['denominator']))<0)
    item['independent_state_space_check_max_error']=delta
(OUT/'fit_results.json').write_text(json.dumps(report,indent=2)+'\n')

plt.rcParams.update({'svg.fonttype':'none','font.size':10})
fig,axes=plt.subplots(3,2,figsize=(14,11))
svgs={}
for col,(name,y) in enumerate(outputs.items()):
    unit='N m' if name=='torque' else 'N';full,held,base,bh=predictions[name]
    axes[0,col].plot(t,y,'ko',ms=3,label='Measured')
    axes[0,col].plot(t,base,':',color='#d97706',label='2 poles, 0 zeros (refit)')
    axes[0,col].plot(t,full,'-',color='#2563eb',label='2 poles, 1 zero')
    axes[0,col].set(title=name.title()+': fitted on all samples',ylabel=f'Output [{unit}]')
    axes[1,col].plot(t,base-y,':',color='#d97706',label='2p0z residual')
    axes[1,col].plot(t,full-y,'-',color='#2563eb',label='2p1z residual')
    axes[1,col].axhline(0,color='gray',lw=.7)
    axes[1,col].set(title='Prediction − measurement',ylabel=f'Error [{unit}]')
    axes[2,col].plot(t[test],y[test],'ko',ms=3,label='Held-out measurements')
    axes[2,col].plot(t[test],bh[test],':',color='#d97706',label='2p0z return prediction')
    axes[2,col].plot(t[test],held[test],'-',color='#2563eb',label='2p1z return prediction')
    axes[2,col].set(title='Fit rising sweep; predict falling sweep',ylabel=f'Output [{unit}]')
    for ax in axes[:,col]:ax.set_xlabel('Elapsed time [s]');ax.grid(alpha=.2);ax.legend(fontsize=8)
    single,ax=plt.subplots(figsize=(6,3.6))
    ax.plot(t,y,'ko',ms=3,label='Measured')
    ax.plot(t,base,':',color='#d97706',label='2 poles, no zero (refit)')
    ax.plot(t,full,color='#2563eb',label='2 poles, 1 zero')
    ax.set(xlabel='Elapsed time [s]',ylabel=f'{name.title()} [{unit}]',title='Recorded-current fit')
    ax.grid(alpha=.2);ax.legend(fontsize=8);single.tight_layout()
    for ext in ['png','pdf']:single.savefig(OUT/f'{name}_fit.{ext}',dpi=180)
    b=io.StringIO();single.savefig(b,format='svg');plt.close(single)
    svg=b.getvalue();svg=svg[svg.index('<svg'):];svg=re.sub(r'<metadata>.*?</metadata>|<!--.*?-->','',svg,flags=re.S)
    for ident in sorted(set(re.findall(r'\bid="([^"]+)"',svg)),key=len,reverse=True):
        svg=svg.replace(f'id="{ident}"',f'id="{name}_{ident}"').replace(f'#{ident}',f'#{name}_{ident}')
    svgs[name]=re.sub(r'>\s+<','><',svg)
    (OUT/f'{name}_fit.svg').write_text(svgs[name])
fig.suptitle('Two poles + one zero: training fit and return-sweep check',fontsize=14)
fig.tight_layout(rect=(0,0,1,.96))
for ext in ['png','pdf']:fig.savefig(OUT/f'comparison.{ext}',dpi=180)
plt.close(fig)
with (OUT/'predictions.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['elapsed_time_s','current_a','torque_measured_nm','torque_2p1z_nm','torque_2p0z_nm','thrust_measured_n','thrust_2p1z_n','thrust_2p0z_n'])
    w.writerows(zip(t,u,outputs['torque'],predictions['torque'][0],predictions['torque'][2],outputs['thrust'],predictions['thrust'][0],predictions['thrust'][2]))

lines=['# Second order: two poles and one zero','',report['method'],'',report['fit_definition'],'',
       '| Output | 2p1z RMSE | 2p1z R² | 2p1z fit (%) | Refit 2p0z RMSE | Return-sweep 2p1z RMSE |','|---|---:|---:|---:|---:|---:|']
for name,item in report['results'].items():
    m=item['two_pole_one_zero'];base=item['refitted_two_pole_no_zero']
    lines.append(f"| {name} | {m['rmse']:.9g} | {m['r_squared']:.9g} | {m['nrmse_fit_percent']:.6g} | {base['rmse']:.9g} | {item['return_sweep_rmse']:.9g} |")
for name,item in report['results'].items():
    m=item['two_pole_one_zero'];b1,b0=m['numerator'];a1,a0=m['denominator'][1:]
    lines+=['',f"## {name.title()}",'',f'G(s) = ({b1:.12g} s + {b0:.12g}) / (s² + {a1:.12g} s + {a0:.12g})',
            f"Output offset = {m['offset']:.12g}. DC gain = {m['dc_gain']:.12g}.",
            f"Poles (real, imaginary) = {m['poles_real_imag']}. Zero = {m['zeros_real_imag']}.",
            f"Converged: {m['converged']}. At parameter bound: {m['at_parameter_bound']}.",
            f"RMSE reduction vs the previous 2p0z fit: {m['rmse_improvement_vs_prior_2p0z_percent']:.6g}%. "
            f"Reduction vs a 2p0z refit with the same broader multistart search: {m['rmse_improvement_vs_refitted_2p0z_percent']:.6g}%."]
lines+=['','## Interpretation','',
        'Both poles are constrained stable; the zero may be in either half-plane. The affine offset is outside the transfer function. '
        'The original no-zero fit has been refitted with the same 32-start search to make the comparison fair. '
        'Any improvement in the refitted no-zero result is due to finding a different optimizer solution, not adding a zero. '
        'All full-record scores are in-sample.', '',report['validation'],'',
        'The recording is sparse and nonlinear, so these poles and zeros remain empirical. '
        'Poor return-sweep performance indicates that fitting the training trajectory does not establish reliable motor dynamics. '
        'NRMSE fit percent is distinct from R² percent.','',
        'Reproduce: `python3 tools/fit_two_pole_one_zero.py`. All plots, coefficients and predictions are saved alongside this report.']
(OUT/'README.md').write_text('\n'.join(lines)+'\n')
mat=['% Stable-pole, unrestricted-zero exploratory current-to-output fits.', '% Transfer functions act on deviations; offsets are separate.']
for name,item in report['results'].items():
    m=item['two_pole_one_zero']
    mat += [f"G_{name}_2p1z = tf([{' '.join(format(v,'.17g') for v in m['numerator'])}], [{' '.join(format(v,'.17g') for v in m['denominator'])}]);",f"{name}_offset_2p1z = {m['offset']:.17g};"]
(OUT/'models.m').write_text('\n'.join(mat)+'\n')

def eq(name):
    m=report['results'][name]['two_pole_one_zero'];b1,b0=m['numerator'];a1,a0=m['denominator'][1:]
    return f'<div class="fraction"><span>{b1:.8g}s {b0:+.8g}</span><span>s² + {a1:.8g}s + {a0:.8g}</span></div><p>Offset: {m["offset"]:.8g}<br>RMSE: {m["rmse"]:.8g}<br>R²: {m["r_squared"]:.8g}<br>NRMSE fit: {m["nrmse_fit_percent"]:.5g}%</p>'
html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Two poles and one zero</title><style>body{font:14px system-ui;color:#172033;padding:18px}p{line-height:1.6}.scroll{overflow:auto}table{border-collapse:collapse;min-width:1500px;width:100%}th,td{border:1px solid #dce2ec;padding:14px;vertical-align:middle;text-align:left}th{background:#142b4a;color:white}svg{width:430px;height:auto}.fraction{display:inline-flex;flex-direction:column;text-align:center;font:16px Georgia;white-space:nowrap}.fraction span:first-child{border-bottom:1px solid;padding-bottom:6px}.fraction span:last-child{padding-top:6px}</style><body><h1>Second order · two poles and one zero</h1><p>Recorded-current responses with fitted offsets; black dots are measurements. Blue: 2p1z model. Orange: refitted 2p0z baseline. All plotted scores use the complete 82-sample training recording. Torque in N·m, thrust in N, current in A.</p><div class="scroll"><table><thead><tr><th>Order</th><th>Tf_torque</th><th>Tf_thrust</th><th>Plot for Torque</th><th>Plot for thrust</th></tr></thead><tbody><tr><td>2 poles, 1 zero</td><td>'''+eq('torque')+'</td><td>'+eq('thrust')+'</td><td>'+svgs['torque']+'</td><td>'+svgs['thrust']+'</td></tr></tbody></table></div><p>See the accompanying notes for return-sweep validation, pole/zero locations, bounds and the fair baseline comparison. Dynamic parameters remain exploratory.</p></body></html>'
(OUT/'notion_table.html').write_text(html)
assert len(html.encode())<=200*1024
print('Verified stable poles and independent state-space responses; saved report and plots.',flush=True)
