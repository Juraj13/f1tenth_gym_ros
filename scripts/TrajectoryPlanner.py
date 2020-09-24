from matplotlib import pyplot as plt
import numpy as np
from math import sin, cos, tan, atan2, sqrt, pi


class TrajectoryPlanner:
    def __init__(self, trajectory_points, theta_0, N):
        self.trajectory_points = trajectory_points
        self.trajectory = np.empty((0,2))
        self.theta0 = theta_0
        self.thetas = np.array([theta_0])
        self.N = N          # number of interpolated points


    def add_trajectory_point(self, pos):
        lastPoint = self.get_last_trajectory_point()
        lastTheta = self.get_last_orientation()
        print(pos)
        self.trajectory_points = np.append(self.trajectory_points, [pos], axis=0)
        if len(self.trajectory_points) == 1:
            return
        
        traj_x, traj_y, theta = self.generate_trajectory(lastPoint, lastTheta, pos)
        
        self.thetas = np.append(self.thetas, theta)
        traj = np.append([traj_x], [traj_y], axis=0)
        traj = traj.transpose()
        self.trajectory = np.append(self.trajectory, traj, axis=0)
        print(theta*180/pi)
        return self.trajectory

    def update_trajectory_point(self, pointID, pos):
        self.trajectory_points[pointID, :] = pos
        self.trajectory = self.trajectory[0:(pointID-1)*self.N, :]
        self.thetas = self.thetas[0:(pointID-1)*self.N]
        if len(self.thetas) == 0:
            self.thetas = np.array([self.theta0])
        if pointID == 0:
            pointID = 1
        for i in range(pointID-1, len(self.trajectory_points)-1):
            traj_x, traj_y, theta = self.generate_trajectory( \
                self.trajectory_points[i], self.thetas[i], self.trajectory_points[i+1])

            traj = np.append([traj_x], [traj_y], axis=0)
            traj = traj.transpose()

            self.thetas = np.append(self.thetas, theta)
            self.trajectory = np.append(self.trajectory, traj, axis=0)

        return self.trajectory


    def generate_trajectory(self, pos_0, orientation_0, pos_f):
        # if method == 'mincurv':
        #     return self.minimum_curvature(pos_0, orientation_0, pos_f)
        # elif method == 'constcurv':
        #     return self.constant_curvature(pos_0, orientation_0, pos_f)
        print(self.relative_curvature(pos_0, orientation_0, pos_f))
        return self.constant_curvature(pos_0, orientation_0, pos_f)
        if self.relative_curvature(pos_0, orientation_0, pos_f) < 0.7:
            print("minimum curvature")
            return self.minimum_curvature(pos_0, orientation_0, pos_f)
        else:
            print("constant curvature")
            return self.constant_curvature(pos_0, orientation_0, pos_f)

    
    def minimum_curvature(self, pos_0, orientation_0, pos_f):
        "uses Newton-Raphson method for optimization"
        x0 = pos_0[0]; y0 = pos_0[1]; theta0 = orientation_0
        xf = pos_f[0]; yf = pos_f[1]

        ka = xf - x0
        kb = yf - y0

        tg0 = tan(theta0)
        
        if abs(tg0) > 1:
            ctg0 = 1 / tg0
            A = np.array( [[1, 3, 0, 0, 0],
                           [0, 0, 0, 1, 3],
                           [ctg0, 0, 0, 1, 0],
                           [1, 1, ctg0, 0, 0],
                           [0, 0, 1, 1, 1]] )
        else:
            A = np.array( [[0, 1, 3, 0, 0],
                           [0, 0, 0, 1, 3],
                           [0, 1, 0, tg0, 0],
                           [1, 1, 1, 0, 0],
                           [tg0, 0, 0, 1, 1]] )

        inv_A = np.linalg.inv(A)
        b = np.array([0, 0, 0, ka, kb]).reshape((5,1))
        spline_coeff = np.matmul(inv_A, b)

        if abs(tg0) > 1:
            a0 = x0; a2 = spline_coeff[0]; a3 = spline_coeff[1]
            b0 = y0; b1 = spline_coeff[2]; b2 = spline_coeff[3]; b3 = spline_coeff[4]
            a1 = b1*ctg0
        else:
            a0 = x0; a1 = spline_coeff[0]; a2 = spline_coeff[1]; a3 = spline_coeff[2]
            b0 = y0; b1 = a1*tg0; b2 = spline_coeff[3]; b3 = spline_coeff[4]

        # wrong angle (tangens -> 2 possible solutions)
        if abs(atan2(b1, a1) - theta0) > 1e-3:
            print('fulo skroz kut')
            A = [ [1, 3, 0, 0],
                  [0, 0, 1, 3],
                  [1, 1, 0, 0],
                  [0, 0, 1, 1] ]
            inv_A = np.linalg.inv(A)
            b = np.array([0, 0, ka+a1, kb+b1]).reshape((4,1))
            spline_coeff = np.matmul(inv_A, b)

            if abs(tg0) > 1:
                a0 = x0; a2 = spline_coeff[0]; a3 = spline_coeff[1]
                b0 = y0; b1 = -b1; b2 = spline_coeff[2]; b3 = spline_coeff[3]
                a1 = b1*ctg0
            else:
                a0 = x0; a1 = -a1; a2 = spline_coeff[0]; a3 = spline_coeff[1]
                b0 = y0; b1 = a1*tg0; b2 = spline_coeff[2]; b3 = spline_coeff[3]
        

        u = np.linspace(0, 1, self.N)
        x = a3*u**3 + a2*u**2 + a1*u + a0
        y = b3*u**3 + b2*u**2 + b1*u + b0

        thetaf = atan2(3*b3 + 2*b2 + b1, 3*a3 + 2*a2 + a1)
        # print(pos_0, theta0)
        return x, y, thetaf


    def constant_curvature(self, pos_0, orientation_0, pos_f):
        'pure geometry'
        x0 = pos_0[0]; y0 = pos_0[1]; theta0 = orientation_0
        xf = pos_f[0]; yf = pos_f[1]
        
        if x0 != xf:
            k1 = (x0**2 + y0**2 - xf**2 - yf**2) / (2*(x0-xf))
            k2 = (y0-yf) / (x0-xf)

        tg0 = tan(theta0)

        # special cases
        if abs(tg0) > 100 and x0 == xf:
            print("okomiti gradijent i x0 == xf")
        if abs(tg0) > 100:
            print("okomiti gradijent")
            cy = y0
            cx = k1 - k2*cy
        elif x0 == xf:
            print("x0 == xf")
            cy = (y0+yf) / 2
            cx = x0 - tg0*(cy-y0)
        else:
            cy = (y0*tg0 + k1 - x0) / (k2 + tg0)
            cx = k1 - k2*cy
        
        # wrong angle
        if (cx-x0) / (cy-y0) != tg0 and x0 != xf:
            cy = (-y0*tg0 + k1 - x0) / (k2 - tg0)
            cx = k1 - k2*cy
        elif x0 == xf:
            print('mozda greska')
            cy = (y0+yf) / 2
            cx = x0 + tg0*(cy-y0)
        
        R = sqrt((x0-cx)**2 + (y0-cy)**2)

        # curve (circle) parametrization
        alpha = atan2( y0-cy, x0-cx )
        betta = atan2( yf-cy, xf-cx )


        D = np.array([[x0-cx, y0-cy], [cos(theta0), sin(theta0)]])
        deter = np.linalg.det(D)
        # print(deter, D)
        if deter > 0:
            if alpha == pi:
                alpha = -pi
            if betta < alpha:
                betta += 2*pi
            thetaf = theta0 + (betta-alpha)
            # print(alpha, betta, theta0, thetaf)
            u = np.linspace(alpha, betta, self.N)
        else:
            if alpha == -pi:
                alpha = pi
            if betta > alpha:
                betta -= 2*pi
            thetaf = theta0 - (alpha-betta)
            u = np.linspace(alpha, betta, self.N)

        x = cx + R*np.cos(u)
        y = cy + R*np.sin(u)
        # print(pos_0, theta0)
        return x, y, thetaf

    def relative_curvature(self, pos, theta, newPos):
        v1 = [cos(theta), sin(theta)]
        v2 = [newPos[0]-pos[0], newPos[1]-pos[1]]
        cos_fi = ( v1[0]*v2[0] + v1[1]*v2[1] ) / sqrt((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))
        sin_fi = sqrt(1-cos_fi**2)
        if cos_fi == 0:
            return 1e9
        return sin_fi/cos_fi


    def get_last_trajectory_point(self):
        if len(self.trajectory_points) == 0:
            return None
        return self.trajectory_points[len(self.trajectory_points)-1]
    
    def get_last_orientation(self):
        if len(self.thetas) == 0:
            return None
        return self.thetas[len(self.thetas)-1]

    def get_trajectory(self):
        return self.trajectory