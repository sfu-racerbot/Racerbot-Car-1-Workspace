"""Re-localise a run's scans against a fixed occupancy grid (distance-field Gauss-Newton)."""
import sys,math,numpy as np
from scipy.ndimage import distance_transform_edt as edt, gaussian_filter, map_coordinates
root='/home/racerbotcar-2/racerbot-ws/data/racerbot_runs/'
LX=.26; MAXR=12.0; CAP=.5
def wrap(a): return (a+np.pi)%(2*np.pi)-np.pi
class Grid:
    def __init__(s,g,res,ox,oy):
        s.res,s.ox,s.oy=res,ox,oy
        d=edt(g<50)*res; s.d=gaussian_filter(np.minimum(d,CAP+.2),.7)
        s.gy,s.gx=np.gradient(s.d,res)
    def samp(s,F,x,y):
        return map_coordinates(F,[(y-s.oy)/s.res-.5,(x-s.ox)/s.res-.5],order=1,mode='nearest')
def match(G,bx,by,pose,iters=12,trim=.30):
    x,y,th=pose
    for it in range(iters):
        c,sn=math.cos(th),math.sin(th)
        wx=x+c*bx-sn*by; wy=y+sn*bx+c*by
        d=G.samp(G.d,wx,wy); gx=G.samp(G.gx,wx,wy); gy=G.samp(G.gy,wx,wy)
        trim_d=max(trim*(.7**it),.12); w=(d<trim_d).astype(float)
        if w.sum()<30: break
        J=np.stack([gx,gy,gx*(-sn*bx-c*by)+gy*(c*bx-sn*by)],1)
        H=(J*w[:,None]).T@J+np.diag([1e-3,1e-3,1e-2]); b=(J*w[:,None]).T@d
        dx=-np.linalg.solve(H,b); x+=dx[0];y+=dx[1];th+=dx[2]
        if np.hypot(dx[0],dx[1])<1e-3 and abs(dx[2])<3e-4: break
    c,sn=math.cos(th),math.sin(th); d=G.samp(G.d,x+c*bx-sn*by,y+sn*bx+c*by)
    return (x,y,th),(d<.10).mean(),(d<.20).mean()
def track(run,t0_rel=10.,step=2,odomkey='odom',grid_override=None):
    z=np.load(f'x/{run}.npz'); g=z['grid'] if grid_override is None else grid_override
    res,ox,oy=float(z['res']),float(z['ox']),float(z['oy']); s=z['slam_pose']; o=z[odomkey] if len(z[odomkey]) else z['odom']
    r=np.load(root+run+'/analysis/run.npz',allow_pickle=True); st=r['scan_t']; sc=r['scan']; m=r['scan_meta']; a=float(m[0])+float(m[1])*np.arange(sc.shape[1])
    G=Grid(g,res,ox,oy); ca,sa=np.cos(a),np.sin(a)
    oth=np.unwrap(o[:,3]); ox_=lambda t:np.interp(t,o[:,0],o[:,1]); oy_=lambda t:np.interp(t,o[:,0],o[:,2]); oh=lambda t:np.interp(t,o[:,0],oth)
    T0=s[0,0]+t0_rel; idx=np.where((st>=T0)&(st<=s[-1,0]))[0][::step]
    pth=np.interp(st[idx[0]],s[:,0],np.unwrap(s[:,3])); pose=(np.interp(st[idx[0]],s[:,0],s[:,1]),np.interp(st[idx[0]],s[:,0],s[:,2]),pth)
    out=[];prev=None
    for i in idx:
        rng=sc[i]; ok=np.isfinite(rng)&(rng>.1)&(rng<MAXR); ok[::1]&=True
        k=np.where(ok)[0][::3]
        bx=LX+rng[k]*ca[k]; by=rng[k]*sa[k]
        t=st[i]
        if prev is not None:
            dth=oh(t)-oh(prev); dxo=ox_(t)-ox_(prev); dyo=oy_(t)-oy_(prev); c0=math.cos(-oh(prev)); s0=math.sin(-oh(prev))
            lx=c0*dxo-s0*dyo; ly=s0*dxo+c0*dyo
            c,sn=math.cos(pose[2]),math.sin(pose[2]); pose=(pose[0]+c*lx-sn*ly,pose[1]+sn*lx+c*ly,pose[2]+dth)
        pose,h10,h20=match(G,bx,by,pose); prev=t
        out.append((t,pose[0],pose[1],pose[2],h10,h20))
    return np.array(out),G,z
if __name__=='__main__':
    for run in sys.argv[1:]:
        A,G,z=track(run); np.save(f'x/{run}_track.npy',A)
        s=z['slam_pose']; T=A[:,0]-s[0,0]
        sx=np.interp(A[:,0],s[:,0],s[:,1]);sy=np.interp(A[:,0],s[:,0],s[:,2]); off=np.hypot(A[:,1]-sx,A[:,2]-sy)
        step=np.hypot(*np.diff(A[:,1:3],axis=0).T); o=z['odom']
        print(run,f'scans {len(A)} inlier<10cm mean {A[:,4].mean():.3f} p5 {np.percentile(A[:,4],5):.3f} min {A[:,4].min():.3f} | <20cm mean {A[:,5].mean():.3f}; biggest step {step.max():.2f}m; path {step.sum():.1f}m')
        print('    t0  h10   h20  live-pose offset')
        for e in np.arange(10,T[-1],10):
            m=(T>=e)&(T<e+10); print(f'   {e:4.0f} {A[m,4].mean():.3f} {A[m,5].mean():.3f}  {off[m].mean():.2f}')
