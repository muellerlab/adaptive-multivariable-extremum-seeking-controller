import numpy as np
from first_order_filters import Low_pass_filter_first_order, High_pass_filter_first_order

class Extremum_seeking_control:
    #cutoff_frequencies in rad/s
    def __init__(self, sampling_period, disturbance_frequency, 
                 disturbance_magnitude, integrator_gain, initial_search_value, 
                 high_pass_cutoff_frequency, low_pass_cutoff_frequency, use_adapter):
        self.dt = sampling_period #in [s]
        self.disturbance_frequency = disturbance_frequency #in rad/s
        self.disturbance_magnitude = disturbance_magnitude
        self.integrator_gain = integrator_gain #negative for gradient descent
        self.initial_search_value = initial_search_value
        self.use_adapter = use_adapter #boolean, true or false (use the adapter or not)
        
        self.high_pass_filter = High_pass_filter_first_order(sampling_period, high_pass_cutoff_frequency, 0, 0)
        self.low_pass_filter = Low_pass_filter_first_order(sampling_period, low_pass_cutoff_frequency, 0)
        
        self.wt = 0 #current phase of perturbation
        self.integral = 0 #integrator ouput
        
        self.m2 = 0 #estimated second moment
        self.b2 = 0.9
        self.epsilon = 1e-8 #to prevent from dividing by zero
        
        return
    
    def update(self, cost):
        high_pass_output = self.high_pass_filter.apply(cost)
        low_pass_input = high_pass_output * np.sin(self.wt)
        low_pass_output = self.low_pass_filter.apply(low_pass_input)
        gradient = 0
        
        if self.use_adapter:
            self.m2 = self.b2 * self.m2 + (1 - self.b2) * np.power(low_pass_output, 2)
            if np.sqrt(self.m2) > 1:
                gradient = low_pass_output / (np.sqrt(self.m2) + self.epsilon)
            else:
                gradient = low_pass_output * np.sqrt(self.m2)

        else:
            gradient = low_pass_output
            
        self.integral +=  self.integrator_gain * gradient * self.dt
        setpoint = self.initial_search_value + self.integral
        output = self.disturbance_magnitude * np.sin(self.wt) + setpoint
        perturbation = self.disturbance_magnitude * np.sin(self.wt)

        #update wt
        self.wt += self.disturbance_frequency * self.dt
        if self.wt > 2 * np.pi:
            self.wt -= 2 * np.pi
            
        return output, high_pass_output, low_pass_output, (np.sqrt(self.m2) + self.epsilon), gradient, setpoint
    