import numpy as np

class KalmanFilter:
    def __init__(self, x=0, y=0, yaw=0):
        # State [x, y, yaw]
        self.state = np.array([x, y, yaw])
        
        # Transition matrix
        self.A = np.identity(3)
        self.B = np.identity(3)
        
        # State covariance matrix
        self.S = np.identity(3) * 1
        
        # Observation matrix
        self.C = np.array([[1, 0, 0],[0, 1, 0]])
        
        # State transition error
        self.R = np.identity(3)
        
        # Measurement error
        self.Q = np.identity(2) * 100

    def predict(self, u):

        self.state = self.A.dot(self.state) + np.matmul(self.B, u)

        self.S = self.A @ self.S @ np.transpose(self.A) + self.R
        

    def update(self, z):
        den = self.C @ self.S @ np.transpose(self.C) + self.Q
        den = np.linalg.inv(den)
        K = self.S @ np.transpose(self.C) @ den

        self.state = self.state + np.matmul(K, z - np.matmul(self.C, self.state))
        self.S = np.matmul((np.identity(3)-np.matmul(K, self.C)), self.S)
        return self.state, self.S
