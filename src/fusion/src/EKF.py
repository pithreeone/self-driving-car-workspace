import numpy as np
from math import cos, sin, atan2

class ExtendedKalmanFilter:
    def __init__(self):
        # define state with pose
        self.pose = np.zeros(3)
        
        # Transition matrix
        self.A = np.identity(3)
        self.B = np.identity(3)
        
        # State covariance matrix
        self.S = np.identity(3) * 1
        
        # Observation matrix
        self.C = np.identity(3)
        
        # State transition error
        self.R = np.identity(3) * 0.0001
        
        # Measurement error
        self.Q = np.identity(3)
        self.Q[0,0] = 1
        self.Q[1,1] = 1
        self.Q[2,2] = 10
        print("Initialize Extended Kalman Filter")
    
    def set_initial_pose(self, x, y, yaw):
        self.pose[0] = x
        self.pose[1] = y
        self.pose[2] = yaw
        

    def predict(self, u):
        # # TA-HINT # # 
        # Base on the Kalman Filter design in Assignment 3
        # Implement a linear or nonlinear motion model for the control input
        # Calculate Jacobian matrix of the model
        yaw = self.pose[2]

        # calculate the nonlinear transition matrix
        self.B[0,0] = cos(yaw); self.B[0,1] = -sin(yaw); self.B[0,2] = 0
        self.B[1,0] = sin(yaw); self.B[1,1] = cos(yaw); self.B[1,2] = 0
        self.B[2,0] = 0; self.B[2,1] = 0; self.B[2,2] = 1

        # calculate the Jacobian in EKF line-2
        G = np.identity(3)
        G[0,0] = 1; G[0,1] = 0; G[0,2] = -u[0]*sin(yaw) - u[1]*cos(yaw)
        G[1,0] = 0; G[1,1] = 1; G[1,2] = u[0]*cos(yaw) - u[1]*sin(yaw)
        G[2,0] = 0; G[2,1] = 0; G[2,2] = 1

        # formula of EKF in prediction stage
        self.pose = self.A.dot(self.pose) + np.matmul(self.B, u)
        self.S = G @ self.S @ np.transpose(G) + self.R
    
        
    def update(self, z):
        # # TA-HINT # # 
        # Base on the Kalman Filter design in Assignment 3
        # Implement a linear or nonlinear observation matrix for the measurement input
        # Calculate Jacobian matrix of the matrix as self.C
        
        den = self.C @ self.S @ np.transpose(self.C) + self.Q
        den = np.linalg.inv(den)
        K = self.S @ np.transpose(self.C) @ den

        self.pose = self.pose + np.matmul(K, z - np.matmul(self.C, self.pose))
        self.S = np.matmul((np.identity(len(self.pose))-np.matmul(K, self.C)), self.S)

        return self.pose, self.S
    
    
    
        