

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=...
        self.Q=...
        self.R=...
        self.x=...
        self.dt = ...
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = ...
        self.C = ...
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            ...,# v
            ...,# w
            ..., # ax
            ..., # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + ... * np.cos(th) * dt,
            y + ... * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              ..., 0,          ...,  0],
            [0, 1,              ..., 0,          ...,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , ..., ..., 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return ...
