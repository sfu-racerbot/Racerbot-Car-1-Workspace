import sys, math, pickle
from pathlib import Path
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
root=Path('/home/racerbotcar-2/racerbot-ws/data/racerbot_runs'); out=Path(sys.argv[1])
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for run in sys.argv[2:]:
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=str(root/run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[x for x in ['/map','/slam_pose','/pose','/odom','/odom/imu'] if x in types]))
    P={k:[] for k in ['/slam_pose','/pose','/odom','/odom/imu']}; M=None; nmap=0
    while r.has_next():
        tp,data,t=r.read_next(); m=deserialize_message(data,get_message(types[tp]))
        if tp=='/map': M=m; nmap+=1
        else:
            p=m.pose.pose if hasattr(m.pose,'pose') else m.pose
            P[tp].append((t*1e-9,p.position.x,p.position.y,yaw(p.orientation)))
    P={k:np.array(v) for k,v in P.items()}
    print(run,{k:len(v) for k,v in P.items()},'maps',nmap, (M.info.width,M.info.height) if M else None)
    if M is None: continue
    g=np.array(M.data,dtype=np.int16).reshape(M.info.height,M.info.width)
    img=np.full(g.shape,0.75); img[g==0]=1; img[g>=50]=0
    res=M.info.resolution;ox,oy=M.info.origin.position.x,M.info.origin.position.y
    np.savez_compressed(out/f'{run}.npz',grid=g,res=res,ox=ox,oy=oy,**{k[1:].replace('/','_'):v for k,v in P.items()})
    fig,a=plt.subplots(figsize=(11,9)); a.imshow(img,cmap='gray',origin='lower',extent=[ox,ox+res*g.shape[1],oy,oy+res*g.shape[0]],vmin=0,vmax=1)
    s=P['/slam_pose']
    if len(s): 
        sc=a.scatter(s[:,1],s[:,2],c=s[:,0]-s[0,0],s=1,cmap='turbo'); plt.colorbar(sc,label='s')
        a.plot(s[0,1],s[0,2],'g*',ms=14)
    a.set_title(run+f' final live /map ({g.shape[1]}x{g.shape[0]})'); a.set_aspect('equal'); fig.tight_layout(); fig.savefig(out/f'{run}.png',dpi=80); plt.close(fig)
