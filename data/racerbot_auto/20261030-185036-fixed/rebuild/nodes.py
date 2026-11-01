import numpy as np,math,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root='/home/racerbotcar-2/racerbot-ws/data/racerbot_runs/'
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
W=lambda a:(a+np.pi)%(2*np.pi)-np.pi
for run in ['20261030-185036','20261030-184826','20261030-184556']:
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=root+run+'/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    ty={t.name:t.type for t in r.get_all_topics_and_types()}; r.set_filter(rosbag2_py.StorageFilter(topics=['/pose']))
    Q=[]
    while r.has_next():
        tp,d,tt=r.read_next(); m=deserialize_message(d,get_message(ty[tp])); p=m.pose.pose
        Q.append((tt*1e-9,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.position.x,p.position.y,yaw(p.orientation)))
    Q=np.array(Q); print(run,'/pose frame',m.header.frame_id,'count',len(Q),'recv-stamp latency median %.3f p95 %.3f max %.3f s'%(np.median(Q[:,0]-Q[:,1]),np.percentile(Q[:,0]-Q[:,1],95),(Q[:,0]-Q[:,1]).max()))
    A=np.load(f'x/{run}_track.npy'); th=np.unwrap(A[:,3]); z=np.load(f'x/{run}.npz'); s=z['slam_pose']
    q=np.load(root+run+'/analysis/slam.npz',allow_pickle=True); mo=q['map_odom']; ob=q['odom_base']
    m_=(Q[:,1]>A[0,0]+1)&(Q[:,1]<A[-1,0]-1)
    t=Q[m_,1]; tx,ty_,tth=np.interp(t,A[:,0],A[:,1]),np.interp(t,A[:,0],A[:,2]),np.interp(t,A[:,0],th)
    for name,off in (('base_link',0.),('laser',.26)):
        px=tx+off*np.cos(tth); py=ty_+off*np.sin(tth); e=np.hypot(Q[m_,2]-px,Q[m_,3]-py); print(f'  /pose vs tracked {name:9s}: pos err median {np.median(e):.3f} p95 {np.percentile(e,95):.3f} max {e.max():.2f}')
    eth=np.degrees(W(Q[m_,4]-tth)); print('  /pose heading err median %.2f p95 %.2f max %.2f (signed mean %.2f)'%(np.median(abs(eth)),np.percentile(abs(eth),95),abs(eth).max(),eth.mean()))
    # map->odom applied at scan stamp: where the TF chain says the car is at the node stamp
    i=np.searchsorted(mo[:,0],t+0.0); # TF rows are (recv,x,y,yaw)
    moth=np.unwrap(mo[:,3]); obth=np.unwrap(ob[:,3])
    def chain(tq,tmo):
        mx=np.interp(tmo,mo[:,0],mo[:,1]);my=np.interp(tmo,mo[:,0],mo[:,2]);mt=np.interp(tmo,mo[:,0],moth)
        ox=np.interp(tq,ob[:,0],ob[:,1]);oy=np.interp(tq,ob[:,0],ob[:,2]);ot=np.interp(tq,ob[:,0],obth)
        return mx+np.cos(mt)*ox-np.sin(mt)*oy,my+np.sin(mt)*ox+np.cos(mt)*oy,mt+ot
    cx,cy,ct=chain(t,t+.0); e=np.hypot(cx-tx,cy-ty_); print('  TF chain at stamp with map->odom then: vs tracked base: median %.3f p95 %.3f'%(np.median(e),np.percentile(e,95)))
    cx,cy,ct=chain(t,Q[m_,0]+.03); e=np.hypot(cx-tx,cy-ty_); print('  TF chain at stamp with map->odom just after /pose: vs tracked base: median %.3f p95 %.3f; heading err median %.2f'%(np.median(e),np.percentile(e,95),np.median(abs(np.degrees(W(ct-tth))))))
    # map->odom change between nodes vs odom-frame distance
    # compare map->odom (from /pose) odometry frame consistency: yaw of map_odom variance
    # odom used by SLAM: odom_imu? 
    mo_y=np.degrees(moth); print('  map->odom yaw range %.1f deg, stepwise p95 %.2f deg, max %.2f'%(mo_y.max()-mo_y.min(),np.percentile(abs(np.diff(mo_y)[abs(np.diff(mo_y))>0]),95),abs(np.diff(mo_y)).max()))
    print('  map->odom xy at corrections, origin lever: odom xy range x %.1f y %.1f'%(np.ptp(ob[:,1]),np.ptp(ob[:,2])))
