import numpy as np


class Low_pass_filter_first_order:
    def __init__(self, sampling_period, cutoff_frequency_rad_per_s, initial_value):
        self.previous_value = initial_value
        self.coefficient = np.exp(-sampling_period * cutoff_frequency_rad_per_s)
        return

    def apply(self, input):
        if self.coefficient <= 0.0:  # no filtering
            self.previous_value = input
            return input
        output = self.coefficient * self.previous_value + (1 - self.coefficient) * input
        self.previous_value = output
        return output


class High_pass_filter_first_order:
    def __init__(
        self, sampling_period, cutoff_frequency_rad_per_s, initial_input, initial_output
    ):
        self.dt = sampling_period
        self.wc = cutoff_frequency_rad_per_s
        # output coefficient
        self.a1 = self.dt * self.wc + 2
        # input coefficient
        self.b1 = self.dt * self.wc - 2
        # prior input
        self.u_prev = initial_input
        # prior output
        self.y_prev = initial_output
        return

    def apply(self, input):
        output = 1.0 / self.a1 * (-self.b1 * self.y_prev + 2 * (input - self.u_prev))
        self.u_prev = input
        self.y_prev = output
        return output
