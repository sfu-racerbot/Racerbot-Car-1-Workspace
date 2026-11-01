import math,numpy as np,yaml,sys,types
from pure_pursuit.auto_map_race_node import LapRecorder, angle_difference
from pure_pursuit import occupancy_map
from scipy.ndimage import distance_transform_edt as edt
p=list(yaml.safe_load(open('/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/config/auto_map_race.yaml')).values())[0]['ros__parameters']
root='/home/racerbotcar-2/racerbot-ws/data/racerbot_runs/'
class Fixed(LapRecorder):
    """A correction is map-frame motion the odom frame did not see."""
    def feed(s,x,y,yaw,t,oy,ox,oyy):
        if s.last_pose is not None and s._lastodom is not None:
            odom_step=math.hypot(ox-s._lastodom[0],oyy-s._lastodom[1]); map_step=math.hypot(x-s.last_pose[0],y-s.last_pose[1])
            s.max_pose_jump=map_step+1 if abs(map_step-odom_step)<=.12 else 0.0001
        s._lastodom=(ox,oyy)
        return s.update(x,y,yaw,t,oy)
    def reset(s): s._lastodom=None; super().reset()
kw=dict(spacing=p['waypoint_spacing'],min_distance=p['minimum_lap_distance'],departure_distance=p['departure_distance'],closure_distance=p['closure_distance'],closure_heading_rad=math.radians(p['closure_heading_deg']),min_duration_sec=p['minimum_lap_duration_sec'],min_turn_rad=math.radians(p['minimum_lap_turn_deg']),max_pose_jump=.12,closure_widen_after_revolutions=p['closure_widen_after_revolutions'],max_closure_distance=p['max_closure_distance'],close_at_closest_approach=p['close_at_closest_approach'],closest_approach_margin=p['closest_approach_margin_m'],min_duration_fraction=p['minimum_lap_duration_fraction'])
for run,lb in (('20261030-185036',1793411451.6742187),('20261030-184556',1793411168.6793122),('20261030-184826',None)):
    z=np.load(f'x/{run}.npz'); s=z['slam_pose']; o=z['odom']; A=np.load(f'x/{run}_track.npy')
    g=z['grid'];res=float(z['res']);ox=float(z['ox']);oy=float(z['oy']); wall=edt(g<50)*res
    if lb is None:
        sp=np.hypot(*np.diff(s[:,1:3],axis=0).T); lb=s[np.where(sp>.01)[0][0],0]-.1
    oth=np.unwrap(o[:,3]); R=Fixed(**kw); laps=[]
    for t,x,y,yaw in s:
        if t<lb: continue
        i=min(np.searchsorted(o[:,0],t),len(o)-1)
        if R.feed(x,y,yaw,t,float(math.atan2(math.sin(oth[i]),math.cos(oth[i]))),o[i,1],o[i,2]):
            pts=np.array(R.lap_points()); laps.append((t-lb,R.distance,R.distance_from_start,math.degrees(R.heading_error),R.reanchor_count,R.revolutions,pts)); R.reset(); R._lastodom=None
    print(run,'with motion-compensated check:',len(laps),'laps; in progress',round(R.distance,1),'m')
    for l in laps:
        pts=l[-1]; c=((pts[:,0]-ox)/res).astype(int); r=((pts[:,1]-oy)/res).astype(int); ok=(c>=0)&(c<g.shape[1])&(r>=0)&(r<g.shape[0]); cl=np.full(len(pts),-1.); cl[ok]=wall[r[ok],c[ok]]
        from scipy.spatial import cKDTree
        d,_=cKDTree(A[:,1:3]).query(pts)
        print(f'   t={l[0]:5.1f} len={l[1]:5.1f}m miss={l[2]:.2f}m head={l[3]:4.1f}deg corrections={l[4]:3d} rev={l[5]:.2f} | recorded line vs final map: min wall {cl.min():.2f}m, frac<0.20m {(cl<.2).mean():.2f}; off driven path median {np.median(d):.2f} p95 {np.percentile(d,95):.2f} max {d.max():.2f}')
    # odom lag vs truth, by yaw and translation
    T=A[:,0]; atheta=np.unwrap(A[:,3]); ayr=np.gradient(atheta,T); asp=np.gradient(A[:,1],T)**2+np.gradient(A[:,2],T)**2; asp=np.sqrt(asp)
    for key in ('odom','odom_imu'):
        if key not in z.files or not len(z[key]): continue
        q=z[key]; qt=np.unwrap(q[:,3]); yr=np.gradient(qt,q[:,0]); sp=np.hypot(np.gradient(q[:,1],q[:,0]),np.gradient(q[:,2],q[:,0]))
        tg=np.arange(T[0]+1,T[-1]-1,.02); ay=np.convolve(np.interp(tg,T,ayr),np.ones(5)/5,'same'); asg=np.convolve(np.interp(tg,T,asp),np.ones(5)/5,'same')
        best=[]; bests=[]
        for lag in np.arange(-.4,.4,.005):
            by=np.convolve(np.interp(tg+lag,q[:,0],yr),np.ones(5)/5,'same'); bs=np.convolve(np.interp(tg+lag,q[:,0],sp),np.ones(5)/5,'same')
            best.append((np.corrcoef(ay,by)[0,1],lag)); bests.append((np.corrcoef(asg,bs)[0,1],lag))
        b=max(best);c=max(bests); print(f'   {key}: leads scans by {-b[1]*1000:+.0f} ms on yaw rate (r={b[0]:.2f}), {-c[1]*1000:+.0f} ms on speed (r={c[0]:.2f})')
