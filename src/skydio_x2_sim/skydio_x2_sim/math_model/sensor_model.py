import numpy as np

class SensorModel:
    def __init__(self, operating_frequency, sigma, bias):
        self.sigma = np.array(sigma)
        self.bias = np.array(bias)
        self.dt_sensor = 1/operating_frequency
        self.t_output_last = -self.dt_sensor # Đảm bảo update ngay lần đầu
        self.measured_state = np.zeros_like(self.sigma)
        self.update_flag = False
        
    def output(self, true_state, t):
        self.update_flag = (t - self.t_output_last) >= self.dt_sensor
        
        if self.update_flag: 
            noise = self.sigma * np.random.randn(*self.sigma.shape)
            self.measured_state = true_state + noise + self.bias
            self.t_output_last = t
            
        return self.measured_state, self.update_flag