import numpy as np
from scipy.integrate import odeint
sqrt = np.sqrt
g = 9.81


def trajectory(initial_vect, x, vx, y, vy, t):
    """
    Calculate positions and velocities of objects at every 
    predetermined moment of time 
    by solving the set of movement equations 
    and add them to the end of the lists.
    """
    initial_vect = np.array(initial_vect)
    x = list(x)
    vx = list(vx)
    y = list(y)
    vy = list(vy)
    
    def movement_equations(vect, t):
        q, vq, p, vp = vect
        f0 = vq
        f1 = 0
        f2 = vp
        f3 = -g
        return [f0, f1, f2, f3]

    solution = odeint(movement_equations, initial_vect, t)
    return (x+list(solution.T[0]), vx+list(solution.T[1]),
            y+list(solution.T[2]), vy+list(solution.T[3]))


def search_collision(entities):
    """
    x1, x2, y1, y2, sum_of_radiuses
    Calculate distances between 2 circles, 
    compare them at all moments of time with the sum of radius,
    return the moment of time in which 
    the first of possible collisions occurs 
    or 0 in the case of collision wasn't found.
    """
    distance = map(
        lambda q1, p1, q2, p2: sqrt((q1-q2)**2+(p1-p2)**2),
        entities[0].x, entities[0].y, entities[1].x, entities[1].y
    )

    if entities[0].shape and entities[1].shape == "circle":
        moment = 0
        for moment in range(len(distance)):
            if distance[moment] <= entities[0].r + entities[1].r:
                break
        if moment >= len(list(entities[0].x)):
            moment = 0
        return moment


def collision(x1, vx1, y1, vy1, x2, vx2, y2, vy2, m1, m2, dt, elasticity, mu, k):
    """Calculate velocities of two circles after 
    elastic or inelastic central collision."""
    
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

    """new normal velocities"""
    vx1n_new = (m1*vx1n+m2*vx2n-(vx1n-vx2n)*m2*elasticity)/(m1+m2)
    vy1n_new = (m1*vy1n+m2*vy2n-(vy1n-vy2n)*m2*elasticity)/(m1+m2)
    vx2n_new = (m1*vx1n+m2*vx2n+(vx1n-vx2n)*m1*elasticity)/(m1+m2)
    vy2n_new = (m1*vy1n+m2*vy2n+(vy1n-vy2n)*m1*elasticity)/(m1+m2)

    """new tangential velocities"""
    vx1t_new = mu * elasticity * k * sqrt(k/(m1*vx1t*vx1t)) * dt + vx1t
    vy1t_new = mu * elasticity * k * sqrt(k/(m1*vy1t*vy1t)) * dt + vy1t
    vx2t_new = mu * elasticity * k * sqrt(k/(m1*vx2t*vx2t)) * dt + vx2t
    vy2t_new = mu * elasticity * k * sqrt(k/(m1*vy2t*vy2t)) * dt + vy2t
    # vx1t_new = vx1t
    # vy1t_new = vy1t
    # vx2t_new = vx2t
    # vy2t_new = vy2t

    """final velocity vectors (v_new)"""
    vx1_new = vx1n_new*unx + vx1t_new*utx
    vy1_new = vy1n_new*uny + vy1t_new*uty
    vx2_new = vx2n_new*unx + vx2t_new*utx
    vy2_new = vy2n_new*uny + vy2t_new*uty    
    
    # """angular velocities after collision"""
    # v1t_new = sqrt(vx1t_new**2 + vy1t_new**2)
    # v2t_new = sqrt(vx2t_new**2 + vy2t_new**2)
    # w1_new = v1t_new/r1
    # w2_new = v2t_new/r2
    #
    # """rotation angles after collision"""
    # phi1 = w1_new*dt
    # phi2 = w2_new*dt

    return vx1_new, vy1_new, vx2_new, vy2_new  # , phi1, phi2