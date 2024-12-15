import numpy as np

class IMUSensor:
    def __init__(self):
        self.acceleration = np.random.rand(3)  # Simulate acceleration (x, y, z)
        self.gyroscope = np.random.rand(3)  # Simulate angular velocity (pitch, roll, yaw)
    
    def read_data(self):
        # Simulate reading sensor data
        return np.concatenate([self.acceleration, self.gyroscope])

# Example usage
imu = IMUSensor()
imu_data = imu.read_data()  # Get IMU sensor data
