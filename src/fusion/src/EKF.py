import numpy as np
from math import cos, sin, atan2

class ExtendedKalmanFilter:
    def __init__(self):
        # Define what state to be estimate
        # Ex.
        #   only pose -> np.array([x, y, yaw])
        #   with velocity -> np.array([x, y, yaw, vx, vy, vyaw])
        #   etc...
        # define state with pose and velocity
        self.pose = np.zeros(6)
        
        # Transition matrix
        self.A = np.identity(6)
        self.B = [[1,0,0],[0,1,0],[0,0,1],[1,0,0],[0,1,0],[0,0,1]]
        
        
        # State covariance matrix
        self.S = np.identity(6) * 1
        
        # Observation matrix
        self.C = np.zeros((3,6))
        
        # State transition error
        self.R = np.identity(6) * 1
        
        # Measurement error
        self.Q = np.identity(3) * 0.00000001
        print("Initialize Extended Kalman Filter")
    
    def set_initial_pose(self, x, y, yaw):
        self.pose[0] = x
        self.pose[1] = y
        self.pose[2] = yaw
        

    def predict(self, u):
        # Base on the Kalman Filter design in Assignment 3
        # Implement a linear or nonlinear motion model for the control input
        # Calculate Jacobian matrix of the model as self.A
        
        self.A[0,0] = self.A[1,1] = self.A[2,2] = 1
        self.A[3,3] = self.A[4,4] = self.A[5,5] = 0
        self.pose = self.A.dot(self.pose) + np.matmul(self.B, u)
        self.S = self.A @ self.S @ np.transpose(self.A) + self.R
    
        
    def update(self, z):
        # Base on the Kalman Filter design in Assignment 3
        # Implement a linear or nonlinear observation matrix for the measurement input
        # Calculate Jacobian matrix of the matrix as self.C
        
        self.C = [[1,0,0,0,0,0],
                  [0,1,0,0,0,0],
                  [0,0,1,0,0,0]]
        
        den = self.C @ self.S @ np.transpose(self.C) + self.Q
        den = np.linalg.inv(den)
        K = self.S @ np.transpose(self.C) @ den

        self.pose = self.pose + np.matmul(K, z - np.matmul(self.C, self.pose))
        self.S = np.matmul((np.identity(len(self.pose))-np.matmul(K, self.C)), self.S)

        return self.pose, self.S
    
    
    
        