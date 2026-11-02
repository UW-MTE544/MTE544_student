"""Plot saved trajectory runs against current planner references, without ROS."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from planner import planner, TRAJECTORY_PLANNER
ROOT=Path(__file__).resolve().parent/'Plots'
OUT=ROOT/'report'/'trajectories'; OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
def read(path, ns=False):
    with path.open() as f:
        rows=list(csv.reader(f))[1:]
    a=np.array([[float(v) for v in row if v.strip()] for row in rows if any(v.strip() for v in row)])
    assert np.isfinite(a).all(), path
    if ns: a[:,-1]/=1e9
    assert np.all(np.diff(a[:,-1])>=0), path
    return a
runs={}; metrics=[]
for shape in ['parabola','sigmoid']:
    folder=ROOT/shape/'pid_1'
    pose=read(folder/'robot_pose.csv',True)
    linear=read(next(folder.glob('*linear.csv'))); angular=read(next(folder.glob('*angular.csv')))
    start=pose[0,-1]
    for a in [pose,linear,angular]: a[:,-1]-=start
    ref=np.array(planner(TRAJECTORY_PLANNER,trajectory=shape).plan())
    # Exact nearest distance to each segment of the planned waypoint polyline.
    seg=np.diff(ref,axis=0); offset=pose[:,None,:2]-ref[None,:-1,:]
    u=np.clip(np.sum(offset*seg,axis=2)/np.sum(seg*seg,axis=1),0,1)
    dist=np.linalg.norm(offset-u[:,:,None]*seg,axis=2).min(axis=1)
    duration=pose[-1,-1]
    rms=np.sqrt(np.trapz(dist**2,pose[:,-1])/duration)
    metrics.append([shape,duration,100*rms,100*dist.max(),100*np.linalg.norm(pose[-1,:2]-ref[-1])])
    runs[shape]=(pose,linear,angular,ref,dist)
def save(fig,name):
    fig.tight_layout(rect=(0,0,1,.94)); fig.savefig(OUT/(name+'.png'),dpi=170);fig.savefig(OUT/(name+'.pdf'));plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,5))
fig.suptitle('PID trajectory tracking — recorded paths vs current planner',fontsize=16)
for ax,(shape,(pose,lin,ang,ref,dist)) in zip(axes,runs.items()):
    ax.plot(ref[:,0],ref[:,1],'k--',label='Planned path')
    ax.plot(pose[:,0],pose[:,1],color='#2563eb',label='Recorded PID path')
    ax.scatter(*pose[0,:2],color='#059669',label='Start',zorder=4)
    ax.scatter(*pose[-1,:2],color='#dc2626',label='Final pose',zorder=4)
    ax.scatter(*ref[-1],marker='x',color='black',s=70,label='Goal')
    ax.set(title=shape.capitalize() + (' (planned x: 0–1 m)' if shape == 'parabola' else ' (planned x: 0–2.5 m)'),xlabel='x (m)',ylabel='y (m)');ax.axis('equal');ax.grid(alpha=.25);ax.legend()
save(fig,'paths')
fig,axes=plt.subplots(3,2,figsize=(12,10));fig.suptitle('PID trajectory errors — distinct references',fontsize=16)
for col,(shape,(pose,lin,ang,ref,dist)) in enumerate(runs.items()):
    for row,t,v,title,unit in [(0,pose[:,-1],100*dist,'Distance to planned path','cm'),(1,lin[:,-1],lin[:,0],'Logged distance to final goal','m'),(2,ang[:,-1],ang[:,0],'Logged heading error to lookahead','rad')]:
        ax=axes[row,col]
        color, style, marker = [('#059669', ':', '^'), ('#2563eb', '-', 'o'), ('#dc2626', '--', 's')][row]
        ax.plot(t, v, color=color, linestyle=style, marker=marker,
                markevery=max(1, len(t)//12), markersize=3, label=title)
        ax.set(title=f'{shape.capitalize()}: {title}', xlabel='Elapsed time (s)',
               ylabel=f'Error ({unit})')
        ax.grid(alpha=.25)
        ax.legend(fontsize=9)
save(fig,'errors')
fig,axes=plt.subplots(3,2,figsize=(12,10));fig.suptitle('PID trajectory robot states',fontsize=16)
for col,(shape,(pose,*_)) in enumerate(runs.items()):
    for row,(title,unit) in enumerate([('x','m'),('y','m'),('Heading','rad')]):
        ax=axes[row,col];ax.plot(pose[:,-1],pose[:,row]);ax.set(title=f'{shape.capitalize()}: {title}',xlabel='Elapsed time (s)',ylabel=unit);ax.grid(alpha=.25)
save(fig,'states')
fig,axes=plt.subplots(2,2,figsize=(12,8));fig.suptitle('PID trajectory error rates',fontsize=16)
fig2,axes2=plt.subplots(2,2,figsize=(12,8));fig2.suptitle('PID trajectory error phase portraits',fontsize=16)
for col,(shape,(pose,lin,ang,ref,dist)) in enumerate(runs.items()):
    for row,(a,title,unit) in enumerate([(lin,'Distance to goal','m'),(ang,'Heading to lookahead','rad')]):
        ax=axes[row,col];ax.plot(a[:,-1],a[:,1]);ax.set(title=f'{shape.capitalize()}: {title}',xlabel='Elapsed time (s)',ylabel=f'Error rate ({unit}/s)');ax.grid(alpha=.25)
        ax=axes2[row,col];ax.plot(a[:,0],a[:,1]);ax.set(title=f'{shape.capitalize()}: {title}',xlabel=f'Error ({unit})',ylabel=f'Error rate ({unit}/s)');ax.grid(alpha=.25)
save(fig,'error_rates');save(fig2,'phase')
with (OUT/'metrics.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['Trajectory','Recorded duration (s)','Time-weighted RMS path error (cm)','Max path error (cm)','Final goal error (cm)']);w.writerows(metrics)
for row in metrics: print(row)
