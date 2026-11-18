from pathlib import Path
import json,sqlite3,sys,hashlib
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy import ndimage as ndi
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parent/'source'))
from go2_accumulated_scan import read_points
run=Path('/media/yaorz/Elements/unitree_go2/experiments/2026-11-18_two_boxes_go2_replacement_01/two_boxes_static_gap86_184933_57204')
assert (run/'bag/metadata.yaml').exists()
bag,=(run/'bag').glob('*.db3')
wal=Path(str(bag)+'-wal');assert not wal.exists() or wal.stat().st_size==0
db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
clouds=[]; framecounts=[]
for i,(t,b) in enumerate(db.execute("select timestamp,data from messages where topic_id=(select id from topics where name='/utlidar/cloud_base') order by timestamp")):
 if i%2:continue
 m=deserialize_message(b,PointCloud2)
 assert m.header.frame_id=='base_link'
 xyz,_=read_points(m);dist=np.linalg.norm(xyz[:,:2],axis=1)
 mask=(dist>=.2)&(dist<=4.)&(xyz[:,2]>=-.2)&(xyz[:,2]<=1.)
 clouds.append(xyz[mask]);framecounts.append(len(clouds[-1]))
db.close(); xyz=np.concatenate(clouds)

import importlib.util,math,yaml,time
from types import SimpleNamespace
from collections import deque
from nav_msgs.msg import Odometry
from rclpy.time import Time
import go2_accumulated_scan as mod
def quat(q):return [q.x,q.y,q.z,q.w]
db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
ids={n:i for i,n in db.execute('select id,name from topics')}
otype=ids['/utlidar/robot_odom'];ctype=ids['/utlidar/cloud_base']
ev=[];poses=[]
for tid,nt,b in db.execute('SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp',(otype,ctype)):
 msg=deserialize_message(b,Odometry if tid==otype else PointCloud2)
 ev.append((tid,nt*1e-9,msg))
 if tid==otype:
  p=msg.pose.pose.position;q=msg.pose.pose.orientation;poses.append([mod.stamp_seconds(msg.header.stamp),p.x,p.y,p.z,*quat(q)])
poses=np.asarray(poses);db.close()
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
 offset=yaml.safe_load((run/'run_metadata.yaml').read_text())['record_time_minus_odom_source_time_median_seconds']
 for tid,rt,msg in ev:
  s.rt=rt;(s.on_odom if tid==otype else s.on_cloud)(msg)
 return saved

rp,=Path('/media/yaorz/Elements/unitree_go2/slam_validation/2026-11-18/two_boxes_86cm').glob('two_boxes_static_gap86_184933_57204_185138_918606/metrics.json')
metric=json.loads(rp.read_text()); spec=yaml.safe_load((rp.parent/'maps/map.yaml').read_text())
from PIL import Image
grid=np.flipud(np.array(Image.open(rp.parent/'maps'/spec['image'])))
row,col=np.nonzero(grid==0);res=spec['resolution'];ox,oy,ang=spec['origin'];assert abs(ang)<1e-6
px,py,yaw=metric['slam_pose_change']['first_xy_yaw'];co,si=np.cos(yaw),np.sin(yaw)
dx=ox+(col+.5)*res-px;dy=oy+(row+.5)*res-py;mp=np.c_[dx*co+dy*si,-dx*si+dy*co]
def endpoints(samples):
 ps=[]
 for _,_,d,a0,da in samples:
  m=np.isfinite(d);a=a0+np.flatnonzero(m)*da
  ps.append(np.c_[d[m]*np.cos(a),d[m]*np.sin(a)])
 return np.concatenate(ps)
old=stream(mod,0)
sc=endpoints(old)
sp=importlib.util.spec_from_file_location('candidate',str(Path(__file__).resolve().parent/'source/candidate_scan.py'))
new=importlib.util.module_from_spec(sp);sp.loader.exec_module(new); sc2=endpoints(stream(new,.2))
def section(p):
 side=[]
 for ya,yb in [(0.36,.69),(-.58,-.27)]:
  z=p[(p[:,0]>=1.39)&(p[:,0]<=1.51)&(p[:,1]>ya)&(p[:,1]<yb)]
  side.append(dict(n=len(z),median=float(np.median(z[:,1])) if len(z) else None))
 return dict(side=side,gap=side[0]['median']-side[1]['median'] if all(s['n'] for s in side) else None)
stats={label:section(p) for label,p in [('xyz',xyz),('scan',sc),('scan_filtered',sc2),('map',mp)]}
stats['splits']=[section(np.concatenate(c))['gap'] for c in [clouds[j*len(clouds)//4:(j+1)*len(clouds)//4] for j in range(4)]]
stats['reconstructed_scans']=len(old)

from matplotlib import use
use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
out=Path(__file__).resolve().parent
fig,axes=plt.subplots(3,1,figsize=(8,7.5),dpi=155,sharex=True,sharey=True)
for ax,p,key,title in zip(axes,[xyz,sc,mp],['xyz','scan','map'],['Body point cloud (every second frame)','Reconstructed default scan endpoints','Saved SLAM map: occupied cell centres']):
 p=p[(abs(p[:,1])<1.18)&(p[:,0]>.99)&(p[:,0]<1.78)]
 ax.axhspan(1.39,1.51,color='#fff0c9',zorder=0)
 ax.scatter(-p[:,1],p[:,0],s=2 if key=='xyz' else 12,c='#196b67',marker='s' if key=='map' else '.',alpha=.20 if key=='xyz' else .9,linewidths=0)
 L,R=stats[key]['side'];a,b=-L['median'],-R['median']
 ax.annotate('',(a,1.45),(b,1.45),arrowprops=dict(arrowstyle='<->',color='#922e16',lw=1.4))
 ax.text((a+b)/2,1.49,f"{stats[key]['gap']*100:.1f} cm",ha='center',color='#922e16',fontsize=10)
 ax.set_title(title,fontsize=10,loc='left');ax.set_aspect('equal');ax.grid(lw=.4,alpha=.3);ax.set_ylabel('forward (m)');ax.yaxis.set_major_locator(MultipleLocator(.2))
axes[0].set_ylim(1.00,1.75);axes[0].set_xlim(-1.15,1.15)
axes[-1].set_xlabel('lateral distance (m), positive to the right')
fig.suptitle('Two stationary boxes | 18 Nov 2026, 18:49 | approximate reference: 86 cm')
fig.tight_layout()
fig.savefig(out/'two_boxes_gap.png',bbox_inches='tight')
stats.update(run=str(run),replay_dir=str(rp.parent),reported_gap_m_approximate=.86,method='Lateral separation of median y at body x=[1.39,1.51] m. Left ROI y=[0.36,0.69], right y=[-0.58,-0.27]. Raw cloud: every other frame, unchanged height and range slice. Rebuilt scans use per-scan body frame; map occupied cell centres in first reported map pose. Not independent ground truth.',sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'source').glob('*.py')})
(out/'gap_metrics.json').write_text(json.dumps(stats,indent=2))
print(out,'saved')

