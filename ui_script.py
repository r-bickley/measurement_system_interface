import math as m
import numpy as np

#construct the plane equation
def plane_eqn(points):
    #where points is an array with dimensions:
    #[[xa,ya,za],[xb,yb,zb],[xc,yc,zc]]
    #check array dimensions:
    if points.shape != (3,3):
        return('Points array does not have the required dimensions.')
    else:
        p_a = points[0]
        p_b = points[1]
        p_c = points[2]
        #construct the vectors
        v_ab = np.array([[p_b[0]-p_a[0]],[p_b[1]-p_a[1]],[p_b[2]-p_a[2]]])
        v_ac = np.array([[p_c[0]-p_a[0]],[p_c[1]-p_a[1]],[p_c[2]-p_a[2]]])
        plane = np.cross(v_ab,v_ac,axis = 0)
        if np.count_nonzero(plane) == 0:
            return('Points are collinear.')
        else:
            d = -plane[0]*p_a[0]-plane[1]*p_a[1]-plane[2]*p_a[2]
            plane = np.array([plane[0],plane[1],plane[2],d])
            return plane

points_1 = np.array([[1,1,1],[-1,1,0],[2,0,3]])
testplane = plane_eqn(points_1)
print(testplane)

def local_pcb_height(plane,point):
    #where plane is the 1x4 vector and point is a x, y coordinate pair
    if plane.shape != (4,1):
        return('Plane vector does not have the required dimensions')
    if len(point) != (2):
        return('Point is not a coordinate pair')
    else:
        z_loc = (-plane[0]*point[0]-plane[1]*point[1]-plane[3])/plane[2]
        return(z_loc)

testpoint_1 = np.array([1,1])
testpoint_2 = np.array([4,5])
test_z_1 = local_pcb_height(testplane,testpoint_1)
test_z_2 = local_pcb_height(testplane,testpoint_2)
print(test_z_1)
print(test_z_2)
