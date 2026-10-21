#!/usr/bin/python3
"""Independent revisit checks. Reference selects regions only after mapping has finished."""
import csv
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from evaluate_long_corridor import summary


def read_pcd(path):
    with open(path,'rb') as f:
        header={}
        while True:
            line=f.readline().decode().strip()
            if not line: continue
            if line.startswith('#'): continue
            key,*values=line.split();header[key]=values
            if key=='DATA': break
        if int(header['POINTS'][0])==0: return np.empty((0,3))
        if header['DATA']!=['binary'] or header['FIELDS']!=['x','y','z','intensity']:
            raise ValueError('Expected uncompressed binary XYZI submap')
        return np.frombuffer(f.read(),dtype='<f4').reshape(-1,4)[:,:3].astype(float)


def world_points(points,pose):
    return Rotation.from_quat(pose[4:8]).apply(points)+pose[1:4]


def distances(a,b):
    if not len(a) or not len(b): return None
    da=cKDTree(b).query(a)[0];db=cKDTree(a).query(b)[0]
    # Restrict the residual statistic to a fixed 0.5 m overlap neighbourhood;
    # always report coverage to expose incompatible scans rather than hide them.
    all_dist=np.r_[da,db];inside=all_dist<=.5
    return dict(overlap_at_0_5m=float(inside.mean()),nearest_distance_m=summary(all_dist[inside]))


def evaluate_revisits(directory,reference):
    reference=np.atleast_2d(np.loadtxt(reference,delimiter=',',skiprows=1));reference[:,0]/=1e9
    nodes=list(csv.DictReader(open(directory/'submaps.csv')))
    times=np.array([float(n['timestamp']) for n in nodes])
    locations=np.column_stack([np.interp(times,reference[:,0],reference[:,k]) for k in range(1,4)])
    raw=np.atleast_2d(np.loadtxt(directory/'raw_trajectory.tum'))
    optimized=np.atleast_2d(np.loadtxt(directory/'optimized_trajectory.tum'))
    pairs=[];last_selected=-100
    for i in range(len(nodes)):
        candidates=np.where((times[i]-times>=30)&(np.linalg.norm(locations-locations[i],axis=1)<=2))[0]
        if not len(candidates) or i-last_selected<15: continue
        j=int(candidates[np.argmin(np.linalg.norm(locations[candidates]-locations[i],axis=1))])
        last_selected=i;pairs.append((j,i))
    results=[]
    for j,i in pairs:
        a,b=read_pcd(directory/'submaps'/('%d.pcd'%j)),read_pcd(directory/'submaps'/('%d.pcd'%i))
        # Identical fixed subsampling before/after; no registration refinement.
        a=a[::max(1,len(a)//3000)];b=b[::max(1,len(b)//3000)]
        index=lambda n:int(np.argmin(abs(raw[:,0]-times[n])))
        ia,ib=index(j),index(i)
        before=distances(world_points(a,raw[ia]),world_points(b,raw[ib]))
        after=distances(world_points(a,optimized[ia]),world_points(b,optimized[ib]))
        results.append(dict(a=j,b=i,time_a=float(times[j]),time_b=float(times[i]),
            reference_separation_m=float(np.linalg.norm(locations[j]-locations[i])),raw=before,optimized=after))
    return dict(selection='Reference positions within 2 m, at least 30 s apart; sample every 15 submaps. Reference used only in this post-run evaluator.',
        note='Nearest-neighbour consistency is not absolute map accuracy; repeated corridor geometry can be ambiguous. Wall thickness requires independently annotated wall patches.',
        pairs=results)
