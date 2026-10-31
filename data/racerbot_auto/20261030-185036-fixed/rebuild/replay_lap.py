import re,sys,math,numpy as np,yaml
from pure_pursuit.auto_map_race_node import LapRecorder
P=yaml.safe_load(open('/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/config/auto_map_race.yaml'))
p=list(P.values())[0]['ros__parameters']
def rec(**kw):
    a=dict(spacing=p['waypoint_spacing'],min_distance=p['minimum_lap_distance'],departure_distance=p['departure_distance'],closure_distance=p['closure_distance'],closure_heading_rad=math.radians(p['closure_heading_deg']),min_duration_sec=p['minimum_lap_duration_sec'],min_turn_rad=math.radians(p['minimum_lap_turn_deg']),max_pose_jump=p['max_pose_jump_m'],closure_widen_after_revolutions=p['closure_widen_after_revolutions'],max_closure_distance=p['max_closure_distance'],close_at_closest_approach=p['close_at_closest_approach'],closest_approach_margin=p['closest_approach_margin_m'],min_duration_fraction=p['minimum_lap_duration_fraction'])
    a.update(kw); return LapRecorder(**a)
run,log=sys.argv[1],sys.argv[2]
z=np.load(f'x/{run}.npz'); s=z['slam_pose']; o=z['odom']
# logged progress
L=[]
for l in open(f'/home/racerbotcar-2/racerbot-ws/data/racerbot_runs/{log}'):
    m=re.search(r'\[(\d+\.\d+)\] \[auto_map_race_node\].*lap (\d)/\d.*samples=(\d+), distance=([\d.]+)/.*turn=(-?\d+)/.*elapsed=([\d.]+)/.*start distance=([\d.]+)/([\d.]+)m, heading error=([\d.]+)/.*absorbed=(\d+)',l)
    if m: L.append(tuple(float(x) for x in m.groups()))
L=np.array(L); print('log lines',len(L),'first log t',L[0,0],'bag first pose',s[0,0],'log start offset elapsed',L[0,0]-L[0,5])
t_start=L[0,0]-L[0,5]
oyaw=np.unwrap(o[:,3])
def run_rec(R,tick_only=True,stop_at_close=True):
    out=[];closes=[]
    for t,x,y,yaw in s:
        if t<t_start: continue
        i=min(np.searchsorted(o[:,0],t),len(o)-1)
        if R.update(x,y,yaw,t,float(math.atan2(math.sin(oyaw[i]),math.cos(oyaw[i])))):
            closes.append((t-t_start,R.distance,R.distance_from_start,math.degrees(R.heading_error),R.reanchor_count)); R.reset()
        out.append((t,R.distance,math.degrees(R.turn),R.distance_from_start,math.degrees(R.heading_error),R.reanchor_count,R.closest_approach, R.start[0] if R.start is not None else np.nan, R.start[1] if R.start is not None else np.nan))
    return np.array(out),closes
out,closes=run_rec(rec())
print('closes as shipped:',[tuple(round(v,1) for v in c) for c in closes])
print(' t_el  | logged dist turn start_d head absorbed | replay dist turn start_d head absorbed | replay stored start')
for l in L[::max(1,len(L)//12)]:
    i=np.searchsorted(out[:,0],l[0]); i=min(i,len(out)-1); r=out[i]
    print(f'{l[5]:6.1f} | {l[3]:6.1f} {l[4]:5.0f} {l[6]:6.2f} {l[8]:5.0f} {l[9]:4.0f} | {r[1]:6.1f} {r[2]:5.0f} {r[3]:6.2f} {r[4]:5.0f} {r[5]:4.0f} | ({r[7]:+.2f},{r[8]:+.2f})')
out2,c2=run_rec(rec(max_pose_jump=0.0))
print('closes with reanchor disabled:',[tuple(round(v,1) for v in c) for c in c2])
for j in (0.25,0.5):
    o3,c3=run_rec(rec(max_pose_jump=j)); print('closes with jump',j,':',[tuple(round(v,1) for v in c) for c in c3])
# step-size stats
ds=np.hypot(*np.diff(s[:,1:3],axis=0).T); dt=np.diff(s[:,0]); print('step p50/p90/p99 m',np.percentile(ds,[50,90,99]).round(3),'dt p50/p99',np.percentile(dt,[50,99]).round(3),'frac>0.12',(ds>.12).mean().round(3))
ods=np.hypot(*np.diff(o[:,1:3],axis=0).T); v=ods/np.maximum(np.diff(o[:,0]),1e-3); vs=np.interp(s[1:,0],o[1:,0],v)
print('jump events',(ds>.12).sum(),'mean speed during them',vs[ds>.12].mean().round(2),'expected motion in step at those',(vs[ds>.12]*dt[ds>.12]).mean().round(3),'actual step',ds[ds>.12].mean().round(3),'dt',dt[ds>.12].mean().round(3))
