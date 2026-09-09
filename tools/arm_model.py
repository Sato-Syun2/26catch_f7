"""ZIPの剛体パラメータから2軸慣性行列を計算。手先回転・上下は0固定。"""
import argparse
import json
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def inertia_matrix(archive, elbow_deg):
    with zipfile.ZipFile(archive) as z:
        r=ET.fromstring(z.read('arm_for_sim_v2_description/urdf/arm_for_sim_v2.xacro'))
    def origin(j):
        return np.fromstring(r.find(f"joint[@name='{j}']/origin").get('xyz'),sep=' ')
    elbow=origin('Revolute 2')
    slider=origin('Slider 5')
    wrist=slider+origin('Revolute 4')
    q=-np.deg2rad(elbow_deg)
    R=np.array([[np.cos(q),-np.sin(q),0],[np.sin(q),np.cos(q),0],[0,0,1]])
    M=np.zeros((2,2))
    for name,offset in [('link1_1',None),('link2_1',np.zeros(3)),('link3_1',slider),('link4_1',wrist)]:
        i=r.find(f"link[@name='{name}']/inertial")
        mass=float(i.find('mass').get('value'))
        c=np.fromstring(i.find('origin').get('xyz'),sep=' ')
        assert np.allclose(np.fromstring(i.find('origin').get('rpy'),sep=' '),0)
        izz=float(i.find('inertia').get('izz'))
        p=c if offset is None else elbow+R@(offset+c)
        J=np.zeros((3,2));J[:,0]=np.cross([0,0,-1],p)
        angular=np.array([1,0] if offset is None else [1,1])
        if offset is not None:J[:,1]=np.cross([0,0,-1],p-elbow)
        M+=mass*J.T@J+izz*np.outer(angular,angular)
    assert np.linalg.eigvalsh(M).min()>0
    return M

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--zip',type=Path,default=Path('../arm_for_sim_v2_description.zip'));a=p.parse_args()
    print(json.dumps({'axis_order':['root_ID2','elbow_ID1'],'unit':'kg*m^2',
                     'assumptions':'URDF joint angle; wrist/slider zero; no added payload; no rotor inertia correction',
                     'samples':{str(q):inertia_matrix(a.zip,q).tolist() for q in [-145,-100,-50,0,50,100,145]}},indent=2))
