import numpy as np
import math

def closest_point_on_2_lines(u1,u2,v1,v2):
    u1_arr = np.array(u1)
    u2_arr = np.array(u2)
    v1_arr = np.array(v1)
    v2_arr = np.array(v2)

    u = u2_arr - u1_arr
    v = v2_arr - v1_arr
    rho = v1_arr - u1_arr
    uv = np.dot(u,v)
    uu = np.dot(u,u)
    vv = np.dot(v,v)
    urho = np.dot(u,rho)
    vrho = np.dot(v,rho)
    try:
        vt = (vrho*uu - urho*uv) / (uv*uv - vv*uu)
        ut = (uv * vt + urho) / uu
    except: pass

    if math.isnan(vt) or math.isinf(vt): vt = 0.0
    if math.isnan(ut) or math.isinf(ut): ut = 0.0

    ut = np.max(np.array([0.0, ut]))
    ut = np.min(np.array([1.0, ut]))
    vt = np.max(np.array([0.0, vt]))
    vt = np.min(np.array([1.0, vt]))

    return ut, vt, u1_arr + ut*(u2_arr - u1_arr), v1_arr + vt*(v2_arr - v1_arr)

def dis_between_line_segments( u1,u2,v1,v2 ):
    ut,vt,p1,p2 = closest_point_on_2_lines( u1,u2,v1,v2 )
    return np.linalg.norm(p1-p2)

def pt_dis_to_line_seg(pt,a,b):
    u = np.dot(pt-a, b-a) / max(np.linalg.norm(a-b)**2, 0.000000001)
    u = np.min(np.array(   [np.max(    np.array([u,0])    )  ,1] ))
    p = a + u*(b - a)
    dis = np.linalg.norm(p - pt)
    return p, u, dis


if __name__ == '__main__':
    # print pt_dis_to_line_seg(np.array([0.,0.,0.]),np.array([0.1,0.1,0.1]),np.array([1.,1.,1.]))
    # print closest_point_on_2_lines([1.,1.,1.], [0.,0.,1.], [0.,0.,0.], [0.,1.,0.])
    print dis_between_line_segments([1.,1.,1.], [1.,0.,1.], [0.,0.,0.], [0.,1.,0.])