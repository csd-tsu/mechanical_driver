# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 20:29:59 2015

@author: olga
"""
import numpy as np
from scipy.optimize import fsolve
from scipy.integrate import odeint
from scipy import argmin
sqrt=np.sqrt
g=9.81

def trajectory(initial_vect,x,vx,y,vy,t):
    """
    Calculates position and velocity of objects at every 
    predetermined moment of time 
    by solving the set of movement equations 
    and adds them to the end of the lists.
    """
    initial_vect=np.array(initial_vect)
    x=list(x)
    vx=list(vx)
    y=list(y)
    vy=list(vy)
    
    def movement_equations(vect, t):
       q, vq, p, vp = vect
       f0=vq
       f1=0
       f2=vp
       f3=-g
       return [f0, f1, f2, f3]

    solution=odeint(movement_equations, initial_vect, t)
    return x+list(solution.T[0]),vx+list(solution.T[1]),y+list(solution.T[2]),vy+list(solution.T[3])
       
        
    
def search_collision(x1,x2,y1,y2,sum_of_radiuses):
    """
    Calculates distances between 2 circles, 
    compares them at all moments of time with the sum of radiuses 
    and returns the moment of time in which 
    the first of possible collisions occurs 
    or 0 in the case of collision wasn't found
    """
    distance=map(lambda q1,p1,q2,p2: sqrt((q1-q2)**2+(p1-p2)**2), x1,y1,x2,y2)
    for moment in range (len(distance)):
       if distance[moment]<=sum_of_radiuses:
          break
    if moment<len(list(x1)):
       return moment
    else:
       return 0

def type_of_collision(x1,y1,x2,y2):
    """Defines whether the collision is central or not"""
    eps = 0.001
    central = 0
    if (abs(x2-x1) <= eps) or (abs(y2-y1) <= eps):
       central = 1
    return central

def collision(vx1,vy1,vx2,vy2,x1,y1,x2,y2,m1,m2,r1,r2,dt,elasticity):
    """Calculates velocities of two circles after 
    elastic or inelastic central collision"""
    	
    """find the normal vector (n)"""
    nx = x2-x1
    ny = y2-y1
   
    """find the unit vector of normal vector (un)"""
    unx = nx/sqrt(nx**2 + ny**2)
    uny = ny/sqrt(nx**2 + ny**2)
	
    """find the unit tangent vector ut"""
    utx = -uny
    uty = unx

    """resolve the velocity vectors into normal (vn) 
    and tangential (vt) components"""
    vx1n = unx * vx1
    vy1n = uny * vy1
    vx2n = unx * vx2
    vy2n = uny * vy2
    vx1t = utx * vx1
    vy1t = uty * vy1
    vx2t = utx * vx2
    vy2t = uty * vy2
	
    """tangential velocities do not change after collision"""
    vx1t_new = vx1t
    vy1t_new = vy1t
    vx2t_new = vx2t
    vy2t_new = vy2t
	
    """new normal velocities"""
    vx1n_new = (m1*vx1n+m2*vx2n-(vx1n-vx2n)*m2*elasticity)/(m1+m2)
    vy1n_new = (m1*vy1n+m2*vy2n-(vy1n-vy2n)*m2*elasticity)/(m1+m2)
    vx2n_new = (m1*vx1n+m2*vx2n+(vx1n-vx2n)*m1*elasticity)/(m1+m2)
    vy2n_new = (m1*vy1n+m2*vy2n+(vy1n-vy2n)*m1*elasticity)/(m1+m2)
	
    """final velocity vectors (v_new)"""
    vx1_new = vx1n_new*unx + vx1t_new*utx
    vy1_new = vy1n_new*uny + vy1t_new*uty
    vx2_new = vx2n_new*unx + vx2t_new*utx
    vy2_new = vy2n_new*uny + vy2t_new*uty    
   
    """angular velocities after collision"""
    v1t_new = sqrt(vx1t_new**2 + vy1t_new**2)
    v2t_new = sqrt(vx2t_new**2 + vy2t_new**2)
    w1_new = v1t_new/r1
    w2_new = v2t_new/r2

    """rotation angles after collision"""
    phi1 = w1_new*dt
    phi2 = w2_new*dt   

    return vx1_new, vy1_new, vx2_new, vy2_new, phi1, phi2 
