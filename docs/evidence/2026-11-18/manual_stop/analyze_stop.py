from pathlib import Path
import json, sqlite3, sys, struct,math
import numpy as np, yaml
from scipy.ndimage import median_filter
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState,SportModeState
from nav_msgs.msg import Odometry
def read_run(run):
 p,=(run/'bag').glob('*.db3'); wal=Path(str(p)+'-wal')
 assert (run/'bag/metadata.yaml').is_file() and (not wal.exists() or wal.stat().st_size==0)
 cn=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
 assert cn.execute('pragma quick_check').fetchone()==('ok',)
 t0=cn.execute('select min(timestamp) from messages').fetchone()[0]
 def rows(topic,typ):
  for ns,b in cn.execute('select timestamp,data from messages where topic_id=(select id from topics where name=?) order by timestamp',(topic,)):
   yield (ns-t0)/1e9,deserialize_message(b,typ)
 rc=[]
 for t,m in rows('/lf/lowstate',LowState):
  h,key,lx,rx,ry,trig,ly=struct.unpack_from('<2sH5f',bytes(m.wireless_remote))
  rc.append([t,h==bytes.fromhex('5551'),key,lx,ly,rx,ry,trig])
 od=[]
 for t,m in rows('/utlidar/robot_odom',Odometry):
  p=m.pose.pose.position;q=m.pose.pose.orientation
  od.append([t,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
 sport=[]
 for t,m in rows('/sportmodestate',SportModeState):sport.append([t,m.velocity[0],m.velocity[1]])
 cn.close();return t0,np.array(rc),np.array(od),np.array(sport)
def summarize(run):
 t0,rc,od,sp=read_run(run);t=rc[:,0];mag=np.max(np.abs(rc[:,3:]),axis=1)
 moving=(mag>=.12)&(rc[:,1]==1)
 if not moving.any():raise ValueError('No active stick input')
 first,last=np.flatnonzero(moving)[[0,-1]]
 def release(th):
  for j in np.where((np.arange(len(t))>last)&(mag<th)&(rc[:,1]==1)&(rc[:,2]==0))[0]:
   k=t.searchsorted(t[j]+.3)
   if k<len(t) and np.all(mag[j:k+1]<th) and np.all(rc[j:k+1,1]==1) and np.all(rc[j:k+1,2]==0):return j
  raise ValueError('No sustained return to neutral')
 j=release(.03);rel=t[j]
 ts=np.arange(od[0,0]+.1,od[-1,0]-.1,.01)
 if len(ts)<100:raise ValueError('Insufficient odometry')
 def pos(z):return np.array([np.interp(z,od[:,0],od[:,i]) for i in [1,2]]).T
 def speed(half):return np.linalg.norm(pos(ts+half)-pos(ts-half),axis=1)/(half*2)
 v=speed(.05);pos_rel=pos(rel)
 def stop(vv,thr):
  a=ts.searchsorted(rel)
  for i in range(a,len(ts)-51):
   if np.all(vv[i:i+51]<thr):return i
  raise ValueError('No sustained stop')
 i=stop(v,.03);pos_stop=pos(ts[i]); drift=pos_stop-pos_rel
 tmax=ts[-1]; tail=od[od[:,0]>=tmax-2,1:3]; pos_end=np.median(tail,axis=0)
 n0=od[(od[:,0]>=max(0,t[first]-2))&(od[:,0]<t[first]),1:3];pos0=np.median(n0,axis=0)
 net=pos_end-pos0; d=np.linalg.norm(net); unit=net/d if d>.01 else np.array([np.nan,np.nan])
 spd=np.linalg.norm(sp[:,1:],axis=1);v2=median_filter(np.interp(ts,sp[:,0],spd),size=11)
 try:isp=stop(v2,.03)
 except ValueError:isp=None
 alt=[(thr,round(ts[ii]-rel,3),round(100*np.linalg.norm(pos(ts[ii])-pos_rel),2)) for thr in [.02,.03,.04,.05] for ii in [stop(v,thr)]]
 result=dict(run=run.name,start_unix_s=t0/1e9,duration_s=round(tmax+.1,3),lowstate_n=len(rc),odom_n=len(od),sport_n=len(sp),invalid_rc_packets=int(np.sum(rc[:,1]==0)),first_active_s=t[first],last_active_s=t[last],last_sample_before_neutral_s=t[j-1],t_release=rel,t_release_deadband_10=t[release(.1)],t_still=ts[i],stop_time_s=ts[i]-rel,net_stop_distance_m=float(np.linalg.norm(drift)),along_path_stop_m=float(drift@unit),net_run_m=d,release_xy=pos_rel.tolist(),stop_xy=pos_stop.tolist(),final_xy=pos_end.tolist(),distance_to_final_from_release_m=float(np.linalg.norm(pos_end-pos_rel)),speed_at_release_m_s=np.interp(rel,ts,v),peak_speed_m_s=float(v.max()),pre_release_speed_m_s=float(np.median(v[(ts>=rel-.5)&(ts<rel)])),poststop_max_speed_m_s=float(v[i+51:].max()),threshold_sensitivity=alt,sport_stop_time_s=None if isp is None else ts[isp]-rel,sport_peak_speed_m_s=float(v2.max()),max_axis=np.max(np.abs(rc[:,3:]),axis=0).tolist(),max_odom_receive_gap_s=float(np.diff(od[:,0]).max()),max_rc_receive_gap_s=float(np.diff(t).max()),tail_xy_range_m=np.ptp(tail,axis=0).tolist(),max_yaw_change_deg=float(np.ptp(np.unwrap(od[:,3]))*180/np.pi))
 z=pos(ts)-pos_rel
 after=(ts>=rel)&(ts<=ts[i]+2.)
 proj=z@unit
 k=np.where(after)[0][np.argmax(proj[after])]
 result.update(max_forward_excursion_m=float(proj[k]),peak_excursion_time_s=ts[k]-rel,forward_to_final_m=float((pos_end-pos_rel)@unit),joystick_sample_bracket_s=rel-t[j-1],method='Timebase: common host bag receive time. Neutral starts at first valid keys=0 axes <0.03 sustained 0.30s after last >=0.12. XY speed is a centered 100ms position difference sampled at 100Hz. Translational standstill: <0.03m/s for 0.50 s. Projected distances use the run direction.',turn_or_multidirection=bool(result['max_yaw_change_deg']>30 or max(result['max_axis'][0],result['max_axis'][2])>.25))
 return result,(t,mag,ts,v,v2,proj)

if __name__=='__main__':
 from matplotlib import use
 use('Agg')
 import matplotlib.pyplot as plt
 out=Path(__file__).resolve().parent
 for trial,r in enumerate(sys.argv[1:],1):
  result,(t,m,ts,v,vs,s)=summarize(Path(r));result['operator_note']=(Path(r)/'operator_note.txt').read_text().strip()
  (out/f'trial_{trial}.json').write_text(json.dumps(result,indent=2))
  print(trial,{k:result[k] for k in ['t_release','net_stop_distance_m','max_forward_excursion_m','peak_excursion_time_s','forward_to_final_m','joystick_sample_bracket_s','stop_time_s']})
  if trial!=2:continue
  release=result['t_release'];stop=result['stop_time_s']
  f,axs=plt.subplots(3,1,sharex=True,figsize=(8.0,6.4),dpi=140)
  axs[0].step(t-release,m,where='post',color='#176d69');axs[0].set_ylabel('stick magnitude')
  axs[1].plot(ts-release,v,label='odometry (100 ms difference)',color='#126963');axs[1].plot(ts-release,vs,label='SportModeState',alpha=.7,color='#d87826');axs[1].axhline(.03,ls=':',c='gray');axs[1].legend(fontsize=8);axs[1].set_ylabel('XY speed (m/s)')
  axs[2].plot(ts-release,100*s,c='#793654');axs[2].axhline(0,c='gray',lw=.6);axs[2].set_ylabel('forward travel (cm)');axs[2].set_xlabel('time relative to stick release (s)')
  axs[2].annotate(f"max {result['max_forward_excursion_m']*100:.1f} cm",xy=(result['peak_excursion_time_s'],result['max_forward_excursion_m']*100),xytext=(1.9,2),arrowprops=dict(arrowstyle='->'))
  for a in axs:
   a.axvline(0,c='#17272c',ls='--',lw=1);a.axvline(stop,c='#ca5f33',ls=':',lw=1);a.grid(alpha=.2);a.set_xlim(-2.2,3.5)
  axs[2].set_ylim(-40,24);f.suptitle('Go2 manual stopping test | repeat | 18 Nov 2026');f.tight_layout();f.savefig(out/'stopping_trace.png')

