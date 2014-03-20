from pymavlink.rotmat import Vector3
from math import sqrt
from numpy import *
import pylab as p
import mpl_toolkits.mplot3d.axes3d as p3

class Calibrator:
    def __init__(self):
        self.samples = []
    
    def add_sample(self,sample):
        self.samples.append([sample.x,sample.y,sample.z])
    
    def _cal_reset(self):
        self.num_iterations = 0
        self.change=100.0
        self.beta = [0,0,0,1,1,1]
        self._reset_matrices
    
    def _reset_matrices(self):
        self.ds = [0 for i in range(6)]
        self.JS = [[0,0,0,0,0,0] for i in range(6)]
    
    def calibrate(self, eps=0.000000001, max_iterations=10000):
        self._cal_reset()
        samps = list(self.samples)
        avg_sample_length = 0
        for i in samps:
            avg_sample_length += sqrt(i[0]**2+i[1]**2+i[2]**2)
        avg_sample_length /= len(samps)
        while self.num_iterations < max_iterations and self.change > eps:
            self.num_iterations+=1
            self._reset_matrices()
            for i in samps:
                self._update_matrices([i[0]/avg_sample_length,i[1]/avg_sample_length,i[2]/avg_sample_length])
            delta = self._find_delta()
            self.change = delta[0]*delta[0] + \
                          delta[0]*delta[0] + \
                          delta[1]*delta[1] + \
                          delta[2]*delta[2] + \
                          delta[3]*delta[3] / (self.beta[3]*self.beta[3]) + \
                          delta[4]*delta[4] / (self.beta[4]*self.beta[4]) + \
                          delta[5]*delta[5] / (self.beta[5]*self.beta[5])
            for i in range(6):
                self.beta[i] -= delta[i]
        
        return (Vector3(-self.beta[0],-self.beta[1],-self.beta[2])*avg_sample_length, Vector3(self.beta[3],self.beta[4],self.beta[5]))
    
    def _update_matrices(self, data):
        residual = 1.0
        jacobian = [0.0 for i in range(6)]
        for j in range(3):
            b = self.beta[3+j]
            dx = data[j] - self.beta[j]
            residual -= b**2*dx**2
            jacobian[j] = 2.0*b**2*dx
            jacobian[3+j] = -2.0*b*dx**2
        
        for j in range(6):
            self.ds[j] += jacobian[j]*residual
            for k in range(6):
                self.JS[j][k] += jacobian[j]*jacobian[k]
    
    def _find_delta(self):
        for i in range(6):
            for j in range(i+1,6):
                mu = self.JS[i][j]/self.JS[i][i]
                if mu != 0.0:
                    self.ds[j] -= mu*self.ds[i]
                    for k in range(6):
                        self.JS[k][j] -= mu*self.JS[k][i]
        
        for i in range(5,-1,-1):
            self.ds[i] /= self.JS[i][i]
            self.JS[i][i] = 1.0
            for j in range(i):
                mu = self.JS[i][j]
                self.ds[j] -= mu*self.ds[i]
                self.JS[i][j] = 0.0
        
        return list(self.ds)
