from matplotlib import pyplot as plt
import numpy as np
from math import sin, cos, tan, atan2, sqrt, pi


class TrajectoryPlanner:
    def __init__(self, trajectory_points, theta_0, N, epsilon=1e-5):
        self.trajectory_points = trajectory_points
        self.trajectory = np.empty((0,2))
        self.theta0 = theta_0
        self.thetas = np.array([theta_0])
        self.N = N              # number of interpolated points
        self.epsilon = epsilon  # when are 2 values equal
        self.Inf = 1e9          # infinity


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
        if pointID == 0:
            pointID = 1
        self.trajectory = self.trajectory[0:(pointID-1)*self.N, :]
        self.thetas = self.thetas[0:pointID]
        if len(self.thetas) == 0:
            self.thetas = np.array([self.theta0])

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
        return self.minimum_curvature(pos_0, orientation_0, pos_f)
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
        
        A = np.array( [[ 2, 6, 12, 20, 0, 0, 0, 0, 1, 0],
                       [ 6, 18, 24, 60, 0, 0, 0, 0, 1, 0],
                       [ 12, 24, 48, 120, 0, 0, 0, 0, 1, 0],
                       [ 20, 60, 120, 200, 0, 0, 0, 0, 1, 0],
                       [ 0, 0, 0, 0, 2, 6, 12, 20, 0, 1],
                       [ 0, 0, 0, 0, 6, 18, 24, 60, 0, 1],
                       [ 0, 0, 0, 0, 12, 24, 48, 120, 0, 1],
                       [ 0, 0, 0, 0, 20, 60, 120, 200, 0, 1],
                       [ 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                       [ 0, 0, 0, 0, 1, 1, 1, 1, 0, 0]] )

        inv_A = np.linalg.inv(A)

        [a, b, K] = self.bestK(x0, y0, xf, yf, theta0, inv_A, 100)
        # print(K)

        u = np.linspace(0, 1, self.N)
        x = a[5]*u**5 + a[4]*u**4 + a[3]*u**3 + a[2]*u**2 + a[1]*u + a[0]
        y = b[5]*u**5 + b[4]*u**4 + b[3]*u**3 + b[2]*u**2 + b[1]*u + b[0]

        thetaf = atan2(5*b[5] + 4*b[4] + 3*b[3] + 2*b[2] + b[1],  \
                       5*a[5] + 4*a[4] + 3*a[3] + 2*a[2] + a[1])
        # print(pos_0, theta0)
        return x, y, thetaf


    def constant_curvature(self, pos_0, orientation_0, pos_f):
        'pure geometry'
        x0 = pos_0[0]; y0 = pos_0[1]; theta0 = orientation_0
        xf = pos_f[0]; yf = pos_f[1]

        tg0 = tan(theta0)

        # special cases
        if abs(atan2(yf-y0, xf-x0) - theta0) < self.epsilon:
            print('colinear points')
            x = np.linspace(x0, xf, self.N)
            y = np.linspace(y0, yf, self.N)
            return x, y, theta0
        elif abs(atan2(yf-y0, xf-x0) + theta0) < self.epsilon:
            print("sjebo si nes")
            return
        
        elif abs(theta0) > 1e9:
            print("vertical gradient")
            k1 = (x0**2 + y0**2 - xf**2 - yf**2) / (2*(x0-xf))
            k2 = (y0-yf) / (x0-xf)
            cy = y0
            cx = k1 - k2*cy
        elif x0 == xf:
            print("x0 == xf")
            cy = (y0+yf) / 2
            cx = x0 - tg0*(cy-y0)
            if abs( (cx-x0) / (cy-y0) + tg0) > self.epsilon: # maybe unneccessary
                print("wrong angle")
        else:
            k1 = (x0**2 + y0**2 - xf**2 - yf**2) / (2*(x0-xf))
            k2 = (y0-yf) / (x0-xf)
            cy = (-y0*tg0 + k1 - x0) / (k2 - tg0)
            cx = k1 - k2*cy
        
            # wrong angle
            if abs( (cx-x0) / (cy-y0) + tg0) > self.epsilon:
                print("wrong angle %f %f" %( (cx-x0) / (cy-y0), tg0))
                cy = (y0*tg0 + k1 - x0) / (k2 + tg0)
                cx = k1 - k2*cy
        
        R = sqrt((x0-cx)**2 + (y0-cy)**2)

        # curve (circle) parametrization
        alpha = atan2( y0-cy, x0-cx )
        betta = atan2( yf-cy, xf-cx )


        D = np.array([[x0-cx, y0-cy], [cos(theta0), sin(theta0)]])
        deter = np.linalg.det(D)
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
        
        return x, y, self.angle_domain(thetaf)


    def bestK(self, x0, y0, xf, yf, theta0, inv_A, N):
        u = np.linspace(0, 1, N)
        delta_theta = np.zeros(N-1)
        ks = np.array([i for i in range(30, 200)])
        curveInts = np.zeros(len(ks))

        a0 = x0; b0 = y0
        for j in range(len(ks)):
            k = ks[j]
            a1 = k*cos(theta0)
            b1 = k*sin(theta0)
            ka = xf - a0 - a1; kb = yf - b0 - b1
            b = np.array([0, 0, 0, 0, 0, 0, 0, 0, ka, kb]).transpose()
            spline_coeff = np.matmul(inv_A, b)
            a = np.array([a0, a1, spline_coeff[0], spline_coeff[1], spline_coeff[2], spline_coeff[3]])
            b = np.array([b0, b1, spline_coeff[4], spline_coeff[5], spline_coeff[6], spline_coeff[7]])
            x = a[5]*u**5 + a[4]*u**4 + a[3]*u**3 + a[2]*u**2 + a[1]*u + a[0]
            y = b[5]*u**5 + b[4]*u**4 + b[3]*u**3 + b[2]*u**2 + b[1]*u + b[0]

            delta_theta[0] = self.angdiff( atan2(y[1]-y[0], x[1]-x[0]), theta0)**2
            if delta_theta[0] > 1.0:
                delta_theta[0] = self.Inf
            for i in range(1, len(delta_theta)-1):
                delta_theta[i] = self.angdiff( atan2(y[i]-y[i-1], x[i]-x[i-1]), atan2(y[i+1]-y[i], x[i+1]-x[i]) )**2
                if delta_theta[i] > 1.0:
                    delta_theta[i] = self.Inf
            curveInts[j] = np.trapz(delta_theta)
        
        M = np.min(curveInts)
        I = np.argmin(curveInts)
        K = ks[I]
        a1 = K*cos(theta0); b1 = K*sin(theta0)
        ka = xf - a0 - a1; kb = yf - b0 - b1
        b = np.array([0, 0, 0, 0, 0, 0, 0, 0, ka, kb]).transpose()
        spline_coeff = np.matmul(inv_A, b)
        a = np.array([a0, a1, spline_coeff[0], spline_coeff[1], spline_coeff[2], spline_coeff[3]])
        b = np.array([b0, b1, spline_coeff[4], spline_coeff[5], spline_coeff[6], spline_coeff[7]])
        return [a, b, K]


    def angdiff(self, th1, th2):
        d = th1 - th2
        d = np.mod(d+np.pi, 2*np.pi) - np.pi
        return d

    def relative_curvature(self, pos, theta, newPos):
        v1 = [cos(theta), sin(theta)]
        v2 = [newPos[0]-pos[0], newPos[1]-pos[1]]
        cos_fi = ( v1[0]*v2[0] + v1[1]*v2[1] ) / sqrt((v1[0]**2+v1[1]**2)*(v2[0]**2+v2[1]**2))
        sin_fi = sqrt(1-cos_fi**2)
        if cos_fi == 0:
            return 1e9
        return sin_fi/cos_fi

    def angle_domain(self, angle):
        if angle > pi:
            return angle - 2*pi
        elif angle < -pi:
            return angle + 2*pi
        else:
            return angle

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