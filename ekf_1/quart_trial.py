import numpy as np 
def quat_exp(q):

        v = np.asarray(q[:3], dtype=float)
        s = float(q[3])
        theta = np.linalg.norm(v)
        
        exp_s = np.exp(s)

        if theta < 1e-12: #small angle approximation 
            return np.array([
            exp_s * v[0],
            exp_s * v[1],
            exp_s * v[2],
            exp_s
            ])

        scale = exp_s * np.sin(theta) / theta

        return np.array([
            scale * v[0],
            scale * v[1],
            scale * v[2],
            exp_s * np.cos(theta)
        ])


def quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])


        