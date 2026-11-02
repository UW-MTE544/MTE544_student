"""Plot saved controller runs; pose stamps are ns, error stamps are seconds."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parent / 'Plots'
OUT = ROOT / 'report' / 'point'
OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
runs = {}
for label, folder, prefix in [('P','p','p_'),('PID','pid','pid_')]:
    tables = {}
    for kind in ('linear','angular','robot_pose'):
        path = ROOT / 'point' / folder / (('' if kind == 'robot_pose' else prefix) + kind + '.csv')
        with path.open() as f:
            rows = list(csv.reader(f))[1:]
        a = np.array([[float(v.strip()) for v in row if v.strip()] for row in rows if any(v.strip() for v in row)])
        assert np.isfinite(a).all(), path
        a[:,-1] /= 1e9 if kind == 'robot_pose' else 1
        assert (np.diff(a[:,-1]) >= 0).all(), path
        tables[kind] = a
    start = tables['robot_pose'][0,-1]
    for a in tables.values():
        a[:,-1] -= start
    runs[label] = tables
colors = ['#2563eb','#dc2626','#059669','#9333ea']
styles = ['-','--','-.',':']
def finish(fig, name):
    fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(OUT / (name+'.png'), dpi=180)
    fig.savefig(OUT / (name+'.pdf'))
    plt.close(fig)
fig, axes = plt.subplots(2,2,figsize=(13,8))
fig.suptitle('Recorded controller comparison — errors and error rates',fontsize=17)
for label,c,ls in zip(runs,colors,styles):
    for row,kind in enumerate(('linear','angular')):
        a=runs[label][kind]
        for col in range(2):
            axes[row,col].plot(a[:,-1],a[:,col],label=label,color=c,ls=ls)
for ax,title,unit in zip(axes.flat,['Distance error','Distance error rate','Heading error','Heading error rate'],['m','m/s','rad','rad/s']):
    ax.set(title=title,xlabel='Time since first pose (s)',ylabel=unit)
    ax.grid(alpha=.25); ax.legend()
finish(fig,'errors')
fig,axes=plt.subplots(2,2,figsize=(13,8))
fig.suptitle('Recorded controller comparison — robot motion',fontsize=17)
for label,c,ls in zip(runs,colors,styles):
    a=runs[label]['robot_pose']
    axes[0,0].plot(a[:,0],a[:,1],label=label,color=c,ls=ls)
    axes[0,0].scatter(a[-1,0],a[-1,1],color=c,s=25)
    for ax,i in zip([axes[0,1],axes[1,0],axes[1,1]],range(3)):
        ax.plot(a[:,-1],a[:,i],label=label,color=c,ls=ls)
axes[0,0].set(title='XY paths (dots mark final samples)',xlabel='x (m)',ylabel='y (m)'); axes[0,0].axis('equal')
for ax,title,unit in zip([axes[0,1],axes[1,0],axes[1,1]],['x position','y position','Heading'],['m','m','rad']):
    ax.set(title=title,xlabel='Time since first pose (s)',ylabel=unit)
for ax in axes.flat: ax.grid(alpha=.25); ax.legend()
finish(fig,'motion')
fig,axes=plt.subplots(1,2,figsize=(13,5))
fig.suptitle('Recorded controller comparison — error phase portraits',fontsize=17)
for label,c,ls in zip(runs,colors,styles):
    for ax,kind,unit in zip(axes,('linear','angular'),('m','rad')):
        a=runs[label][kind]; ax.plot(a[:,0],a[:,1],label=label,color=c,ls=ls)
        ax.set(title=kind.capitalize(),xlabel=f'Error ({unit})',ylabel=f'Error rate ({unit}/s)')
for ax in axes: ax.grid(alpha=.25); ax.legend()
finish(fig,'phase')
for label,t in runs.items():
    print(f"{label}: recorded duration {t['robot_pose'][-1,-1]:.2f} s, final logged distance error {t['linear'][-1,0]:.4f} m")

# Point-run metrics: goal from decisions.py, consistent with recorded errors.
goal = np.array([-1., -1.])
metrics = []
for label,tables in runs.items():
    pose = tables['robot_pose']; xy = pose[:,:2]; time = pose[:,-1]
    direction = goal-xy[0]; distance = np.linalg.norm(direction)
    progress = (xy-xy[0]) @ (direction/distance) / distance
    def crossing(level):
        indices = np.flatnonzero(progress >= level)
        if not len(indices): return np.nan
        i = indices[0]
        if i == 0: return time[0]
        return float(np.interp(level, progress[i-1:i+1], time[i-1:i+1]))
    error = np.linalg.norm(xy-goal,axis=1)
    within = np.flatnonzero(error <= .05)
    metrics.append([label, crossing(.9)-crossing(.1),
                    float(time[within[0]]) if len(within) else np.nan,
                    error[-1]*100, max(0., float(progress.max()-1))*100])
with (OUT/'metrics.csv').open('w') as f:
    w=csv.writer(f); w.writerow(['Controller','10–90% rise time (s)','First entry within 5 cm (s)','Final position error (cm)','Observed along-track overshoot (%)']); w.writerows(metrics)
fig,axes=plt.subplots(1,3,figsize=(14,4.8))
fig.suptitle('Point-goal performance — target (-1, -1) m',fontsize=17)
for ax,column,title,unit in zip(axes,[1,3,4],['Agility: 10–90% rise time','Accuracy: final position error','Observed target overshoot'],['s','cm','% of initial distance']):
    values=[r[column] for r in metrics]
    bars=ax.bar(list(runs),values,color=colors,width=.65)
    ax.bar_label(bars,labels=[f'{v:.2f}' for v in values],padding=5)
    ax.set(title=title,ylabel=unit,ylim=(0,max(max(values)*1.25,1)))
    ax.grid(axis='y',alpha=.25); ax.set_axisbelow(True)
fig.text(.5,.015,'Lower is better. Overshoot is along the start-to-goal axis; recordings end near the 5 cm stop threshold.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.06,1,.92))
fig.savefig(OUT/'metrics.png',dpi=180); fig.savefig(OUT/'metrics.pdf'); plt.close(fig)
for row in metrics: print(row)
