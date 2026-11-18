from pathlib import Path
import sys, json, yaml, math, sqlite3
import numpy as np
from scipy import ndimage as ndi
from scipy.stats import theilslopes
from PIL import Image
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
sys.path.insert(0,str(Path(__file__).resolve().parent/'source'))
import go2_accumulated_scan as mod
root=Path('/media/yaorz/Elements/unitree_go2')
rp,=root.glob('slam_validation/2026-11-18/two_boxes_dynamic_86cm/two_boxes_dynamic_gap86_190437_58794_*/metrics.json')
run,=root.glob('experiments/2026-11-18_two_boxes*/two_boxes_dynamic_gap86_190437_58794')
stat_run,=root.glob('experiments/2026-11-18_two_boxes*/two_boxes_static_gap86_184933_57204')
static_q=json.loads((stat_run/'gap_analysis_v1/gap_metrics.json').read_text())
d=json.loads(rp.read_text()); summary=json.loads((run/'motion_validation.json').read_text())
bag,=(run/'bag').glob('*.db3');wal=Path(str(bag)+'-wal'); assert (run/'bag/metadata.yaml').is_file() and (not wal.exists() or wal.stat().st_size==0)
db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
t0=db.execute('select min(timestamp) from messages').fetchone()[0]/1e9
ids={n:i for n,i in db.execute('select name,id from topics')}
def entries(topic):
 return db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids[topic],))
poses=[]
for nt,b in entries('/utlidar/robot_odom'):
 m=deserialize_message(b,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
 poses.append([mod.stamp_seconds(m.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
poses=np.array(poses);assert (np.diff(poses[:,0])>=0).all()
tr=np.genfromtxt(rp.parent/'estimated_trajectory.csv',delimiter=',',names=True)
tt=tr['aligned_odom_stamp_ns']*1e-9
ang=tr['map_yaw_rad']-tr['odom_yaw_rad']
tfx=tr['map_x_m']-np.cos(ang)*tr['odom_x_m']+np.sin(ang)*tr['odom_y_m']
tfy=tr['map_y_m']-np.sin(ang)*tr['odom_x_m']-np.cos(ang)*tr['odom_y_m']
px,py,yaw=d['slam_pose_change']['first_xy_yaw'];ca,sa=np.cos(yaw),np.sin(yaw)
def start_frame(x,y):
 dx,dy=x-px,y-py
 return np.column_stack((ca*dx+sa*dy,-sa*dx+ca*dy))
offset=float(d['time_diagnostics']['go2/slam/time_alignment']['offset_seconds'])
phases=[(2,21),(24,30),(35,62)]
cloud=[[] for _ in phases]; num=[0,0,0]
for j,(nt,b) in enumerate(entries('/utlidar/cloud_base')):
 tm=nt/1e9-t0
 phase=next((k for k,(s,e) in enumerate(phases) if s<=tm<e),None)
 if phase is None:continue
 m=deserialize_message(b,PointCloud2); xyz,dt=mod.read_points(m);st=mod.stamp_seconds(m.header.stamp)+dt
 rng=np.hypot(xyz[:,0],xyz[:,1])
 k=(rng>=.2)&(rng<=5)&(xyz[:,2]>=-.2)&(xyz[:,2]<=1.)&(st>=poses[0,0])&(st<=poses[-1,0])
 if not np.any(k):continue
 xyz=xyz[k];st=st[k]
 pos,q=mod.interpolate_poses(poses,st);world=mod.rotate_points(xyz,q)+pos
 ix=np.clip(np.searchsorted(tt,st+offset,side='right')-1,0,len(tt)-1)
 c,s=np.cos(ang[ix]),np.sin(ang[ix])
 xy=start_frame(tfx[ix]+c*world[:,0]-s*world[:,1],tfy[ix]+s*world[:,0]+c*world[:,1])
 cloud[phase].append(np.c_[xy,world[:,2],xyz[:,2],np.full(len(xy),tm)])
 num[phase]+=1
db.close();cloud=[np.concatenate(v) for v in cloud]
g=yaml.safe_load((rp.parent/'maps/map.yaml').read_text());im=np.flipud(np.array(Image.open(rp.parent/'maps'/g['image'])))
r=g['resolution']; ox,oy,a=g['origin']; assert abs(a)<1e-6
ri,ci=np.where(im==0);grid=start_frame(ox+(ci+.5)*r,oy+(ri+.5)*r)
def clusters(p):
 edges=np.arange(-3.005,3.006,.02)
 h,_,_=np.histogram2d(p[:,0],p[:,1],bins=(edges,edges));mask=h>=3
 la,n=ndi.label(ndi.binary_dilation(mask,iterations=1),structure=np.ones((3,3)))
 v=[]
 for i in range(1,n+1):
  a,b=np.where((la==i)&mask)
  if len(a)<5:continue
  v.append((int(h[a,b].sum()),(np.round([edges[a.min()],edges[a.max()]+.02,edges[b.min()],edges[b.max()]+.02],2)).tolist()))
 return sorted(v,reverse=True)[:15]
def face_stats(p):
 series={}
 for name,yr in [('left',(.62,.90)),('right',(-.90,-.66))]:
  sub=p[(p[:,0]>1.30)&(p[:,0]<1.68)&(p[:,1]>=yr[0])&(p[:,1]<yr[1])]
  edges=np.arange(yr[0],yr[1]+.012,.025); ys=[];xs=[]
  for a,b in zip(edges,edges[1:]):
   v=sub[(sub[:,1]>=a)&(sub[:,1]<b)]
   if len(v):ys.append(np.median(v[:,1]));xs.append(np.median(v[:,0]))
  series[name]=dict(median_forward_m=float(np.median(xs)),lateral_bins=len(xs))
 return series
def sides(p,x0,x1):
 sides=[]
 for yr in [(.30,.62),(-.66,-.38)]:
  sub=p[(p[:,0]>=x0)&(p[:,0]<x1)&(p[:,1]>=yr[0])&(p[:,1]<=yr[1]),1]
  sides.append(dict(n=len(sub),median_y=float(np.median(sub)) if len(sub) else None))
 return dict(sides=sides,lateral_gap_m=sides[0]['median_y']-sides[1]['median_y'] if all(s['n'] for s in sides) else None)
samples=cloud+[grid];names=['before','moving','after','final_map']
stats={name:{'inner_sides':sides(p,1.59,1.71),'window_sensitivity':{f'{x:.2f}':sides(p,x,x+.12) for x in [1.56,1.59,1.62]},'fronts':face_stats(p)} for name,p in zip(names,samples)}
# Inspect all three classes in the middle of the gap, without counting the walls.
rows,cols=np.indices(im.shape)
centres=start_frame(ox+(cols.ravel()+.5)*r,oy+(rows.ravel()+.5)*r)
central=(centres[:,0]>=1.38)&(centres[:,0]<1.70)&(centres[:,1]>=-.35)&(centres[:,1]<.28)
stats['central_patch']={str(int(c)):int(n) for c,n in zip(*np.unique(im.ravel()[central],return_counts=True))}
ob_tr=np.genfromtxt(run/'odometry_trajectory.csv',delimiter=',',names=True)
raw_mot=summary['odometry_motion']
mtr=d['slam_pose_change'];ntr=d['trajectory_timing']
stats['run']={'duration':summary['duration_s'],'cloud_hz':summary['streams']['/utlidar/cloud_base']['received_hz'],'cloud_frames':summary['streams']['/utlidar/cloud_base']['messages'],'net_odom_xy_m':raw_mot['net_xy_displacement_m'],'net_odom_yaw_deg':raw_mot['net_yaw_change_deg'],'net_slam_xy_m':mtr['net_xy_m'],'net_slam_yaw_deg':mtr['net_yaw_deg'],'scan_count':d['received_messages']['/scan'],'vertices':d['pose_graph_vertices'],'trajectory_timing':ntr,'source_max_gap_s':summary['streams']['/utlidar/cloud_base']['source_interval_s']['maximum']}
stats['phase_windows']={k:{'seconds':list(interval),'cloud_frames':n} for k,interval,n in zip(names,phases,num)}
stats['static_map_gap_m']=static_q['map']['gap']
stats['coordinate_method']='Per-point body data transformed by interpolated full odometry, then latest sampled replay map-to-odom; all layers expressed in the start body XY axes. Registered positions can contain pose/timing and view effects.'
stats['distance_method']='Median separation of opposite side-face returns/cell centres in x=[1.59,1.71] m. Nearby 12cm strips tested with left edges 1.56,1.59,1.62 m; static result used a strip relative to its own starting pose. All heights from the configured -0.2 to +1.0 m body slice. Front depths are balanced over lateral 2.5cm bins.'

candidate_path=Path(__file__).resolve().parent/'source/candidate.py'
import importlib.util, time
from collections import deque
from types import SimpleNamespace
from rclpy.time import Time
sp=importlib.util.spec_from_file_location('candidate',candidate_path);candidate=importlib.util.module_from_spec(sp);sp.loader.exec_module(candidate)
replay_scans=d['received_messages']['/scan']
scan_db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
otype=ids['/utlidar/robot_odom'];ctype=ids['/utlidar/cloud_base']
ev=[(tid,nt/1e9,deserialize_message(blob,Odometry if tid==otype else PointCloud2)) for tid,nt,blob in scan_db.execute('SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp',(otype,ctype))]
scan_db.close()
def stream(module, span):
 class Offline:
  on_odom=module.AccumulatedScan.on_odom
  on_cloud=module.AccumulatedScan.on_cloud
  process_pending=module.AccumulatedScan.process_pending
  def get_clock(self):return SimpleNamespace(now=lambda:Time(nanoseconds=round((self.rt-offset)*1e9)))
 s=Offline();s.history=deque();s.pending=deque(maxlen=10);s.clouds=deque();s.last_cloud_time=s.last_frame=s.last_published_pose=s.last_publish_time=None
 s.bootstrap_scans=s.dropped=s.processed=s.resets=0;s.min_span=span;s.cell_size=.10;s.persistence_range=2.
 conf=yaml.safe_load((run/'config/go2_slam_bringup/accumulated_scan.yaml').read_text())['go2_accumulated_scan']['ros__parameters']
 for a,b in dict(window='window_seconds',low='minimum_height',high='maximum_height',minimum_range='range_min',maximum_range='range_max',increment='angle_increment',gap_reset='gap_reset_seconds',publish_translation='minimum_publish_translation',publish_rotation='minimum_publish_rotation',publish_interval='minimum_publish_interval',stationary_refresh='stationary_refresh_seconds').items():setattr(s,a,conf[b])
 saved=[];s.publisher=SimpleNamespace(publish=lambda m:saved.append((s.rt,mod.stamp_seconds(m.header.stamp),np.asarray(m.ranges).copy(),m.angle_min,m.angle_increment)))
 start=time.perf_counter()
 for tid,rt,msg in ev:
  s.rt=rt;(s.on_odom if tid==otype else s.on_cloud)(msg)

 return saved

old=stream(mod,0); new=stream(candidate,.20)
assert len(old)==len(new) and all(o[1]==n[1] for o,n in zip(old,new))
def scan_counts(scans):
 S={'scans':len(scans),'finite':0,'near_40cm':0,'inner_gap_patch':0,'box_region':0}
 for rt,t,rng,amin,inc in scans:
  k=np.isfinite(rng); av=amin+np.flatnonzero(k)*inc; p,q=mod.interpolate_poses(poses,np.array([t])); pts=np.c_[rng[k]*np.cos(av), rng[k]*np.sin(av),np.zeros(k.sum())]
  world=mod.rotate_points(pts,q)+p;j=np.clip(np.searchsorted(tt,t+offset,side='right')-1,0,len(tt)-1)
  c,s=math.cos(ang[j]),math.sin(ang[j]);xy=start_frame(tfx[j]+c*world[:,0]-s*world[:,1],tfy[j]+s*world[:,0]+c*world[:,1])
  S['finite']+=int(k.sum());S['near_40cm']+=int((rng[k]<.4).sum());S['inner_gap_patch']+=int(((xy[:,0]>=1.38)&(xy[:,0]<1.70)&(xy[:,1]>=-.35)&(xy[:,1]<.28)).sum());S['box_region']+=int(((xy[:,0]>=1.30)&(xy[:,0]<=1.85)&(((xy[:,1]>=.30)&(xy[:,1]<=1.12))|((xy[:,1]>=-1.15)&(xy[:,1]<=-.38)))).sum())
 return S
stats['offline_scan_comparison']={'default':scan_counts(old),'span_0.2s':scan_counts(new),'scope':'in-memory scan endpoints; no ground-truth leg labels'}
from matplotlib import use
use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import hashlib
out=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,1,figsize=(9,5.8),dpi=150,sharex=True,sharey=True)
for i,c in [(0,'#007375'),(2,'#cb5328')]:
 p=cloud[i];ok=(p[:,0]>1.17)&(p[:,0]<1.92)&(np.abs(p[:,1])<1.23);xy=p[ok,:2]
 key,ct=np.unique(np.floor(xy/.007).astype(int),axis=0,return_counts=True)
 xy=(key[ct>=3]+.5)*.007
 axes[0].scatter(-xy[:,1],xy[:,0],s=6,c=c,alpha=.6,linewidth=0,label=['Before motion (2-21 s)','Moving','After motion (35-62 s)'][i])
axes[0].legend(loc='upper center',ncol=2,fontsize=8,frameon=False);axes[0].set_title('Body-cloud returns aligned using the replay poses',loc='left',fontsize=11)
ax=axes[1];ok=(grid[:,0]>1.17)&(grid[:,0]<1.92)&(abs(grid[:,1])<1.23)
ax.scatter(-grid[ok,1],grid[ok,0],s=17,marker='s',c='#213647',linewidth=0)
ax.axhspan(1.59,1.71,color='#ffedb5',alpha=.65,zorder=0)
R=stats['final_map']['inner_sides'];xleft,xright=[-s['median_y'] for s in R['sides']]
ax.annotate('',xy=(xleft,1.651),xytext=(xright,1.651),arrowprops=dict(arrowstyle='<->',color='#95391d',lw=1.4))
ax.text((xleft+xright)/2,1.67,f"{100*R['lateral_gap_m']:.1f} cm",ha='center',fontsize=10,color='#873720')
ax.set_title('Occupied cell centres on the final 3 cm grid',loc='left',fontsize=11)
for a in axes:
 a.set_xlim(-1.19,1.22);a.set_ylim(1.205,1.90);a.set_aspect('equal');a.grid(alpha=.15);a.set_ylabel('forward (m)')
axes[1].set_xlabel('lateral position, positive to the right (m)')
fig.suptitle('Two-box dynamic run  |  18 Nov 2026, 19:04',fontsize=14)
fig.tight_layout();fig.savefig(out/'dynamic_box_comparison.png',bbox_inches='tight')
stats['paths']={k:str(v) for k,v in {'bag':bag,'map':rp.parent/'maps/map.pgm','metrics':rp,'static':stat_run/'gap_analysis_v1/gap_metrics.json'}.items()}
stats['sha256']={k:hashlib.file_digest(open(v,'rb'),'sha256').hexdigest() for k,v in stats['paths'].items()} if hasattr(hashlib,'file_digest') else {k:hashlib.sha256(Path(v).read_bytes()).hexdigest() for k,v in stats['paths'].items() if k!='bag'}
(out/'metrics.json').write_text(json.dumps(stats,indent=2))

