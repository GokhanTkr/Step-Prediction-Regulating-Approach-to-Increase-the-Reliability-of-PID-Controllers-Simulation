"""
This part's job is to simulate the PID controller without the anomalies.
That way Step Prediction Regulating PID controller can have a reference point.
"""





import random
import Constants
import GenerateAnomaly

class PID_Control:

    # Defining some constant values for future.
    integral_value = 0
    feedback_value = 0.0
    current_time = 0.0

    def __init__(self,kp, ki, kd, setpoint):
        
        # Defining the values to use in other functions
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.anomaly_list_pid = []
        self.sample_time = 0.00

        self.last_error = 0.0
        self.last_time = 0.0
        self.output = 0.0
        self.delta_time = 0.2


    def proportional_part(self):

        # Generating P value for PID
        proportional_value = self.error * self.kp
        return proportional_value

    def integral_part(self):

        # Generating I value for PID
        self.integral_value += self.ki * self.error * self.delta_time
        return self.integral_value

    def derivative_part(self):

        # Generating D value for PID
        derivative_value = self.kd * self.delta_error / self.delta_time
        return derivative_value

    def calculate_pid(self):

        # Defining an error value to seperate the inefficient and efficient errors
        self.error = self.setpoint - self.feedback_value

        # Delta error for dy.
        self.delta_error = self.error - self.last_error

        # Defining output value.
        self.output = self.proportional_part() + self.integral_part() + self.derivative_part()

        # Updating last error for dy
        self.last_error = self.error

        # Feedback value is the position of the inefficient system which is output - external error
        self.feedback_value += self.output

        # Updating current time
        self.current_time += 0.2

        return self.feedback_value
