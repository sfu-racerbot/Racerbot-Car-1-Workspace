import numpy as np,yaml,math,sys
from pathlib import Path
from pure_pursuit import recorded_path,racing_math,occupancy_map,run_commands
run='20261030-185036'
OUT=Path('/home/racerbotcar-2/racerbot-ws/data/racerbot_auto/20261030-185036-fixed'); OUT.mkdir(parents=True,exist_ok=True)
p=list(yaml.safe_load(open('/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/config/auto_map_race.yaml')).values())[0]['ros__parameters']
z=np.load(f'x/{run}.npz'); g=z['grid'];res=float(z['res']);ox=float(z['ox']);oy=float(z['oy']);s=z['slam_pose']
# map files, same trinary encoding slam_toolbox's map saver writes
img=np.full(g.shape,205,np.uint8); img[g==0]=254; img[g>=50]=0
with open(OUT/'map.pgm','wb') as f:
    f.write(f'P5\n# CREATOR: final /map of run {run}\n{g.shape[1]} {g.shape[0]}\n255\n'.encode()); f.write(img[::-1].tobytes())
(OUT/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print('despeckle:',occupancy_map.despeckle_map_file(str(OUT/'map.yaml'),max_cells=p['map_despeckle_max_cells']))
print('files',[x.name for x in OUT.iterdir()])
grid=occupancy_map.OccupancyMap.from_yaml(str(OUT/'map.yaml')) if hasattr(occupancy_map.OccupancyMap,'from_yaml') else None
if grid is None:
    print([m for m in dir(occupancy_map.OccupancyMap) if not m.startswith('_')]); sys.exit()
field=grid.clearance_field(); cf=lambda xs,ys:grid.clearance_at(xs,ys,field)
A=np.load(f'x/{run}_track.npy'); T=A[:,0]-s[0,0]; seg=np.r_[0,np.hypot(*np.diff(A[:,1:3],axis=0).T)]; L=np.cumsum(seg)
# one lap = from a point back to its nearest return in the same direction
start=np.array([s[0,1],s[0,2]])
near=np.hypot(A[:,1]-start[0],A[:,2]-start[1])
laps=[]; i=0
passes=[]; 
for k in range(1,len(A)-1):
    if near[k]<2.5 and near[k]<=near[k-1] and near[k]<=near[k+1] and abs(((A[k,3]-s[0,3])+np.pi)%(2*np.pi)-np.pi)<.6 and (not passes or L[k]-L[passes[-1]]>20): passes.append(k)
print('passes t,len:',[(round(T[k],1),round(L[k],1),round(near[k],2)) for k in passes])
best=None
for a,b in zip(passes[:-1],passes[1:]):
    xy=A[a:b,1:3]
    pr=recorded_path.prepare(xy,spacing=p['waypoint_spacing'],max_steering_angle=p['profile_max_steering_angle'],wheelbase=p['profile_wheelbase'],min_feature_wavelength=p['profile_min_feature_wavelength'],curvature_margin=p['profile_curvature_margin'],max_deviation=p['profile_max_deviation'],reject_ratio=p['profile_reject_ratio'],reject_fraction=p['profile_reject_fraction'],clearance_fn=cf,required_clearance=p['profile_wall_clearance'])
    cl=float(np.min(cf(pr.xy[:,0],pr.xy[:,1]))); print(f'lap t={T[a]:.1f}-{T[b]:.1f} len={L[b]-L[a]:.1f} ok={pr.acceptable} feasible={pr.feasible} clear={cl:.2f} | {pr.describe()}')
    if pr.acceptable and (best is None or cl>best[0]): best=(cl,pr,a,b,xy)
cl,pr,a,b,xy=best
racing_math.save_xy_csv(str(OUT/'raceline_raw.csv'),xy)
line=pr.xy
# seam at the point closest to where the car started that run, so racing starts near its usual spot
i0=int(np.argmin(np.hypot(line[:,0]-start[0],line[:,1]-start[1]))); line=np.roll(line,-i0,axis=0)
seg_len=racing_math.compute_segment_lengths(line,closed=True); kappa=racing_math.estimate_path_curvature(line,closed=True)
def prof(vm): return racing_math.compute_velocity_profile(seg_len,kappa,v_max=vm,v_min=p['profile_min_speed'],a_lat_max=p['profile_max_lateral_accel'],a_accel_max=p['profile_max_accel'],a_brake_max=p['profile_max_brake'],closed=True,smoothing_passes=p['profile_smoothing_passes'])
for vm,name in ((1.5,'raceline_profiled.csv'),(1.5,'raceline_1p5.csv'),(2.5,'raceline_2p5.csv'),(4.0,'raceline_4p0.csv')):
    v=prof(vm); racing_math.save_profiled_csv(str(OUT/name),line,v); print(name,f'{v.min():.2f}-{v.max():.2f} m/s, lap {racing_math.estimate_lap_time(seg_len,v):.1f}s')
d=line[1]-line[0]; yaw=math.atan2(d[1],d[0])
run_commands.write_start_pose(OUT,(line[0,0],line[0,1],yaw))
print('start',line[0],math.degrees(yaw),'original car start',start,math.degrees(s[0,3]),'off by',np.hypot(*(line[0]-start)))
print('len',seg_len.sum(),'pts',len(line),'closest wall',float(np.min(cf(line[:,0],line[:,1]))),'kmax',np.abs(kappa).max(),'limit',recorded_path.curvature_limit(p['profile_max_steering_angle'],p['profile_wheelbase']))
np.save('x/line.npy',line)
