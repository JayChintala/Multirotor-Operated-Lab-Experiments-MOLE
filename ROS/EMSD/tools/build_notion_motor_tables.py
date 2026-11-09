"""Build self-contained HTML tables and individual plots for Notion publication.
All curves reconstruct saved models; no refitting or external assets.
"""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/emsd_matplotlib')
from pathlib import Path
import json, io, re, html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'analysis/motor_identification'
OUT=BASE/'notion_publication';OUT.mkdir(exist_ok=True)
f=json.loads((BASE/'fit_results.json').read_text())['results']
h=json.loads((BASE/'higher_order_results.json').read_text())['results']
x=np.genfromtxt(BASE/'identified_samples.csv',delimiter=',',names=True)
t=x['elapsed_time_s'];u=x['current_a'];dt=np.diff(t)
y={'torque':x['torque_nm_signed'],'thrust':x['thrust_n_positive']}
plt.rcParams.update({'svg.fonttype':'none','font.size':9,'axes.spines.top':False,'axes.spines.right':False})


def section(tau):
    a=-np.expm1(-dt/tau)
    forcing=u[:-1]*a+np.diff(u)/dt*(dt-tau*a)
    v=np.empty(len(t));v[0]=u[0]
    for k in range(len(dt)):v[k+1]=(1-a[k])*v[k]+forcing[k]
    return v


def complex_section(p):
    a=np.expm1(p*dt)
    force=u[:-1]*a/p+np.diff(u)/dt*(a-p*dt)/(p*p)
    v=np.empty(len(t),dtype=complex);v[0]=-u[0]/p
    for k in range(len(dt)):v[k+1]=(1+a[k])*v[k]+force[k]
    return v


def prediction(m,order):
    if order==0:return m['offset']+m['gain']*u
    if order==1:return m['offset']+m['gain']*section(m['tau_s'])
    if order==2:
        p1,p2=np.roots(m['denominator'])
        return m['offset']+(m['numerator'][0]*(complex_section(p1)-complex_section(p2))/(p1-p2)).real
    return m['offset']+np.column_stack([section(tau) for tau in m['time_constants_s']])@np.array(m['parallel_gains'])


def poly_text(c,var):
    degree=len(c)-1
    terms=[]
    for j,v in enumerate(c):
        power=degree-j
        suffix='' if power==0 else var if power==1 else f'{var}<sup>{power}</sup>'
        value=f'{abs(v):.7g}'
        terms.append(('− ' if v<0 else '+ ' if terms else '')+value+suffix)
    return ' '.join(terms)


def formula(m,order):
    n=poly_text(m['numerator'],'s');d=poly_text(m['denominator'],'s')
    return f'<span class="fraction"><span>{n}</span><span>{d}</span></span><small>Output offset: {m["offset"]:.8g}</small>'


def chart(name,pred,label,slug):
    fig,ax=plt.subplots(figsize=(5,2.8))
    ax.plot(t,y[name],'o',color='#111827',ms=2.7,label='Measured',zorder=3)
    ax.plot(t,pred,color='#2563eb',lw=1.5,label='Model')
    unit='N·m' if name=='torque' else 'N'
    ax.set(xlabel='Elapsed time (s)',ylabel=f'{name.title()} ({unit})')
    ax.grid(alpha=.18);ax.legend(loc='upper right',fontsize=7)
    fig.tight_layout(pad=.6)
    p=OUT/f'{slug}_{name}'
    fig.savefig(str(p)+'.png',dpi=160)
    fig.savefig(str(p)+'.pdf')
    buffer=io.StringIO();fig.savefig(buffer,format='svg');plt.close(fig)
    svg=buffer.getvalue();svg=svg[svg.index('<svg'):]
    svg=re.sub(r'<metadata>.*?</metadata>','',svg,flags=re.S)
    svg=re.sub(r'<!--.*?-->','',svg,flags=re.S)
    # Each inline plot needs unique SVG IDs within a table document.
    ids=re.findall(r'\bid="([^"]+)"',svg)
    for ident in sorted(set(ids),key=len,reverse=True):
        svg=svg.replace(f'id="{ident}"',f'id="{slug}_{name}_{ident}"').replace(f'#{ident}',f'#{slug}_{name}_{ident}')
    svg=re.sub(r'>\s+<','><',svg)
    p.with_suffix('.svg').write_text(svg)
    rmse=np.sqrt(np.mean((pred-y[name])**2))
    return f'<figure>{svg}<figcaption>RMSE: {rmse:.8g} {unit}</figcaption></figure>'


STYLE='''<style>
*{box-sizing:border-box}body{margin:0;padding:18px;font:14px system-ui,sans-serif;color:#172033;background:#fff}h1{font-size:24px;margin:0 0 8px}p{line-height:1.6;color:#526078}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;min-width:1540px}th,td{border:1px solid #dce2ec;padding:14px;text-align:left;vertical-align:middle}th{background:#142b4a;color:white;position:sticky;top:0;font-size:14px}tbody tr:nth-child(even){background:#f7f9fc}td:first-child{font-weight:700;min-width:90px}td.eq{min-width:240px;max-width:330px;overflow-wrap:anywhere;font-family:Cambria,Georgia,serif;font-size:15px}td.plot{min-width:390px}figure{margin:0}svg{width:100%;height:auto;min-width:350px;max-width:540px}small{display:block;font:12px system-ui,sans-serif;color:#526078;line-height:1.5;margin-top:10px}figcaption{font-size:12px;color:#526078;text-align:center}.fraction{display:inline-flex;flex-direction:column;text-align:center;max-width:100%}.fraction>span:first-child{border-bottom:1px solid #526078;padding-bottom:6px}.fraction>span:last-child{padding-top:6px}.note{padding:12px 16px;background:#fff8e7;border-left:4px solid #ca8a04;margin:12px 0 20px}.legend{font-size:13px}sup{font-size:.75em}
@media print{body{padding:0}table{min-width:0}td.plot{min-width:0}svg{min-width:0}th{position:static}td.eq{min-width:0;font-size:10px}th,td{padding:6px}h1{font-size:20px}}
</style>'''


def document(title,rows,nonlinear=False):
    headers=['Degree','Torque Q(I)','Thrust T(I)','Plot for Torque','Plot for thrust'] if nonlinear else ['Order','Tf_torque = ΔQ(s)/ΔI(s)','Tf_thrust = ΔT(s)/ΔI(s)','Plot for Torque','Plot for thrust']
    desc='Static polynomial current maps; polynomial degree is not dynamic system order.' if nonlinear else 'Stable continuous-time models. Transfer functions describe deviations; the output offsets shown separately apply to absolute-current simulations.'
    note='All plots use the same recorded current sweep (82 samples, ~108 s), equilibrium initial conditions, and fitted output offsets. Black dots: measured output; blue curve: model at measured timestamps. I in A, T in N, Q in N·m. Thrust sign reversed from the workbook and converted from kgf using 9.80665.'
    warning='These are in-sample plots, not validation of dynamic poles. First-order time constants reached the 1 ms lower bound. Second-order fast poles were unresolved. Orders 3–6 use signed parallel real-pole sections, can be ill-conditioned, and predicted the held-out return sweep poorly. The cubic/quartic static maps predict the return sweep substantially better.'
    return '<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title>'+STYLE+'<body><h1>'+title+'</h1><p>'+desc+'</p><p class="legend">'+note+'</p><div class="note">'+warning+'</div><div class="scroll"><table><thead><tr>'+''.join('<th>'+a+'</th>' for a in headers)+'</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div><p>Scroll horizontally to see all five columns. Each model also has individual PNG, PDF, and SVG plots in the downloadable bundle.</p></body></html>'


manifest=[]
for group,orders in [('transfer_functions_orders_0_2',range(3)),('transfer_functions_orders_3_6',range(3,7))]:
    rows=[]
    for order in orders:
        cells=[];plots=[]
        for name in ['torque','thrust']:
            m=f[name]['static' if order==0 else f'order_{order}'] if order<=2 else h[name][f'LTI order {order}']['all_data_model']
            pred=prediction(m,order)
            assert np.isfinite(pred).all()
            saved=f[name]['static' if order==0 else f'order_{order}']['rmse'] if order<=2 else h[name][f'LTI order {order}']['all_data_rmse']
            assert np.isclose(np.sqrt(np.mean((pred-y[name])**2)),saved,rtol=1e-7)
            cells.append(formula(m,order))
            plots.append(chart(name,pred,f'Order {order}',f'order_{order}'))
        rows.append('<tr><td>'+('0 (static gain)' if order==0 else str(order))+'</td>'+''.join('<td class="eq">'+a+'</td>' for a in cells)+''.join('<td class="plot">'+a+'</td>' for a in plots)+'</tr>')
    path=OUT/f'{group}.html';path.write_text(document('Transfer functions · orders '+('0–2' if group.endswith('0_2') else '3–6'),rows));manifest.append(path.name)
rows=[]
for degree in range(1,5):
    cells=[];plots=[]
    for name in ['torque','thrust']:
        m=h[name][f'Static degree {degree}']['all_data_model'];c=m['coefficients_ascending_current_powers']
        pred=np.polynomial.polynomial.polyval(u,c)
        cells.append(poly_text(c[::-1],'I'))
        plots.append(chart(name,pred,f'Degree {degree}',f'polynomial_degree_{degree}'))
    rows.append('<tr><td>'+str(degree)+'</td>'+''.join('<td class="eq">'+a+'</td>' for a in cells)+''.join('<td class="plot">'+a+'</td>' for a in plots)+'</tr>')
path=OUT/'nonlinear_current_maps.html';path.write_text(document('Nonlinear current maps · degrees 1–4',rows,True));manifest.append(path.name)
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
for filename in manifest:
    size=(OUT/filename).stat().st_size
    assert size<=200*1024,(filename,size)
    print(filename,size,'bytes')
print('All dynamic predictions match saved fit RMSE. Generated 22 individual plots in PNG/PDF/SVG.')
