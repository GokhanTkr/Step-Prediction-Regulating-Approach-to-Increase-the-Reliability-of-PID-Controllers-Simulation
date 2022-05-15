"""
This part's job is to get the anomalies from GenerateAnomaly class 
and simulate the real world behaviour of the Step Prediction Regulating PID controllers
"""






import Pid
import Constants
import random
import GenerateAnomaly
import PredictedPid

class SPR:

    # Defining some constant values for future.
    integral_value = 0
    feedback_value_spr = 0.0
    current_time = 0.0
    feedback_value_list = []

    def __init__(self,kp, ki, kd, kdelta_d, setpoint,output_store,kloopControl):
        
        # Defining the values to use in other functions
        self.kloopControl = kloopControl
        self.output_store = output_store
        self.anomaly_list_spr = []
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kdelta_d = kdelta_d

        self.sample_time = 0.00

        self.last_error = 0.0
        self.last_time = 0.0
        self.output = 0.0
        self.delta_time = 0.2

        self.predicted_pid = PredictedPid.PID_Control(kp,ki,kd,setpoint)

    def get_anomaly(self,index): 
        # Generating random anomalies in every step and divide by 10 to decrease the aggresiveness
        anomaly = GenerateAnomaly.list_anomaly[index]
        return anomaly

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

    def calculate_spr(self,index_):

        # Defining an external error value to seperate the inefficient and efficient errors
        self.external_error = self.get_anomaly(index_)
        
        # Defining an error value to seperate the inefficient and efficient errors
        self.error = self.setpoint - self.feedback_value_spr

        # Delta error for dy.
        self.delta_error = self.error - self.last_error

        # Defining output value.
        self.output = self.proportional_part() + self.integral_part() + self.derivative_part()

        # Updating last error for dy
        self.last_error = self.error

        # Feedback value is the position of the inefficient system which is output - external error
        self.feedback_value_spr += (self.output - self.external_error)

        # Updating current time
        self.current_time += 0.2

        return self.feedback_value_spr


    def step_prediction_regulating(self,index_):
        self.pid_calculated_spr = self.calculate_spr(index_) 
        
        self.pid_pred_list = []
        # Creating a loop for Step Prediction
        count = 0
        for i in range(self.kloopControl):
            
            # Defining the efficient output
            self.pid_calculated_predicted = self.predicted_pid.calculate_pid()
            self.pid_pred_list.append(self.pid_calculated_predicted)
            count += 1

        delta_e = ((self.pid_pred_list[count - 1]-self.pid_calculated_spr) / (self.delta_time * count)) * self.kdelta_d
        print(delta_e)
        self.pid_calculated_spr += delta_e
        return self.pid_calculated_spr       