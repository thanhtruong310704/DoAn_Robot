import math
import config as cfg

class Cubic:
    def __init__(self): self.a0=0; self.a1=0; self.a2=0; self.a3=0; self.T=1
    def generate(self, qi, qf, vi, vf, T):
        self.a0=qi; self.a1=vi
        self.a2=(3*(qf-qi)/(T*T))-(2*vi+vf)/T
        self.a3=(2*(qi-qf)/(T*T*T))+(vi+vf)/(T*T)
        self.T=T
    def get(self, t):
        if t>self.T: t=self.T
        return self.a0+self.a1*t+self.a2*t*t+self.a3*t*t*t

def IK(x, y, z):
    try:
        l1, l2, l3, l4 = cfg.l1, cfg.l2, cfg.l3, cfg.l4
        q1 = math.degrees(math.atan2(y, x))
        r = math.sqrt(x*x + y*y)
        r_1 = r - l4
        z_1 = l1 - z 
        A = r_1*r_1 + z_1*z_1
        max_reach = l2 + l3
        if math.sqrt(A) > max_reach: return None, None, None
        c3 = (A - l2*l2 - l3*l3) / (2 * l2 * l3)
        c3 = max(min(c3, 1), -1)
        s3 = math.sqrt(max(0.0, 1 - c3*c3))
        q3 = math.degrees(math.atan2(s3, c3))
        c2 = r_1*(l3*c3+l2)+(l3*s3)*z_1
        s2 = z_1*(l3*c3+l2)-(l3*s3)*r_1
        q2 = math.degrees(math.atan2(s2, c2))
        return q1, q2, q3
    except Exception:
        return None, None, None

def FK(q1, q2, q3):
    l1, l2, l3, l4 = cfg.l1, cfg.l2, cfg.l3, cfg.l4
    q1_rad = math.radians(q1)
    q2_rad = math.radians(q2)
    q3_rad = math.radians(q3)
    
    r_wrist = l2 * math.cos(q2_rad) + l3 * math.cos(q2_rad + q3_rad)
    z_wrist = l1 - (l2 * math.sin(q2_rad) + l3 * math.sin(q2_rad + q3_rad))
    
    x = (r_wrist + l4) * math.cos(q1_rad)
    y = (r_wrist + l4) * math.sin(q1_rad)
    z = z_wrist 
    return (x, y, z)

def cubic_xyz(start, finish, total_time=2.0, dt=0.05):
    cx, cy, cz = Cubic(), Cubic(), Cubic()
    cx.generate(start[0], finish[0], 0, 0, total_time)
    cy.generate(start[1], finish[1], 0, 0, total_time)
    cz.generate(start[2], finish[2], 0, 0, total_time)
    traj = []
    t = 0.0
    while t <= total_time + 1e-6: 
        traj.append((cx.get(t), cy.get(t), cz.get(t)))
        t += dt
    return traj