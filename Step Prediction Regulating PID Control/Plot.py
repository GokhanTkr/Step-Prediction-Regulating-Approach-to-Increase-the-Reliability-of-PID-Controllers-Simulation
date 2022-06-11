"""
This part's job is to plot the datas from three of the controllers
"""






import Pid
import PredictedPid
import random
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib
import Constants
import StepPredictionRegulating as spr
import math

figure(figsize=(8, 6), dpi=40)
# Defining some values and lists for future usage
x_list = [0]

output_store_pid = []
setpoint_store = []
error_list_pid = []
y_list_pid = [0]

output_store_spr = []
feed_list_spr = []

k_delta_1 = [0]
k_delta_2 = [0]
k_delta_3 = [0]
k_delta_4 = [0]

error_list_pred_pid = []
y_list_pred_pid = [0]




# Defining and tuning the PID
pid_predicted = PredictedPid.PID_Control(Constants.KP,Constants.KI,Constants.KD,Constants.SETPOINT)

pid = Pid.PID_Control(Constants.KP,Constants.KI,Constants.KD,Constants.SETPOINT)

spr_delta_1 = spr.SPR(Constants.KP,Constants.KI,Constants.KD,0.5,Constants.SETPOINT,output_store_spr,Constants.PREDICTION_CONSTANT)
spr_delta_2 = spr.SPR(Constants.KP,Constants.KI,Constants.KD,0.75,Constants.SETPOINT,output_store_spr,Constants.PREDICTION_CONSTANT)
spr_delta_3 = spr.SPR(Constants.KP,Constants.KI,Constants.KD,1,Constants.SETPOINT,output_store_spr,Constants.PREDICTION_CONSTANT)
spr_delta_4 = spr.SPR(Constants.KP,Constants.KI,Constants.KD,1.2,Constants.SETPOINT,output_store_spr,Constants.PREDICTION_CONSTANT)

# Plotting function
def begin_plot():

    # Calculating the PID Values in a loop
    for i in range(Constants.LOOP_CONSTANT):

        # Storing the outputs in a variable
        pid_calc = pid.calculate_pid(i)
        
        # Adding the results to a list
        x_list.append(i * 0.02)

        # Adding the results to a list
        y_list_pid.append(pid_calc)

        # Adding the results to a list
        error_list_pid.append(pid.error)

    # Calculating the PID Values in a loop
    for i in range(int(Constants.LOOP_CONSTANT)):

        # Storing the outputs in a variable
        pid_calc_pred = pid_predicted.calculate_pid()

        # Adding the results to a list
        y_list_pred_pid.append(pid_calc_pred)

        # Adding the results to a list
        error_list_pred_pid.append(pid.error)

    # Creating a loop for using the SPR-PID with equal steps with regular PID
    for j in range(int(Constants.LOOP_CONSTANT)):

        delta_spr_1 = spr_delta_1.step_prediction_regulating(j)
        delta_spr_2 = spr_delta_2.step_prediction_regulating(j)
        delta_spr_3 = spr_delta_3.step_prediction_regulating(j)
        delta_spr_4 = spr_delta_4.step_prediction_regulating(j)

        # Adding the results to a list
        k_delta_1.append(delta_spr_1)
        k_delta_2.append(delta_spr_2)
        k_delta_3.append(delta_spr_3)
        k_delta_4.append(delta_spr_4)

    # Summation for rmse
    e_sqrt_1 = 0
    e_sqrt_2 = 0
    e_sqrt_3 = 0
    e_sqrt_4 = 0
    for k in range(int(Constants.LOOP_CONSTANT)):
        
        e_sqrt_1 += math.pow((y_list_pred_pid[k] - k_delta_1[k]),2)
        e_sqrt_2 += math.pow((y_list_pred_pid[k] - k_delta_1[k]),2)
        e_sqrt_3 += math.pow((y_list_pred_pid[k] - k_delta_1[k]),2)
        e_sqrt_4 += math.pow((y_list_pred_pid[k] - k_delta_1[k]),2)

    
    rmse_1 = math.sqrt( e_sqrt_1/Constants.LOOP_CONSTANT)
    rmse_2 = math.sqrt( e_sqrt_2/Constants.LOOP_CONSTANT)
    rmse_3 = math.sqrt( e_sqrt_3/Constants.LOOP_CONSTANT)
    rmse_4 = math.sqrt( e_sqrt_4/Constants.LOOP_CONSTANT)

    print("rmse0.5 =",rmse_1,"rmse0.75 =",rmse_2,"rmse1 =",rmse_3,"rmse1.2 =",rmse_4)

    # Some plot automations
    plt.axhline(y = Constants.SETPOINT,linewidth = 1, color = 'b', label = "Setpoint")
    plt.plot(x_list,y_list_pid, color = 'y')
    plt.plot(x_list,k_delta_1, color = matplotlib.colors.to_hex('#FFA200'))
    plt.plot(x_list,k_delta_2, color = matplotlib.colors.to_hex('#00F7FF'))
    plt.plot(x_list,k_delta_3, color = matplotlib.colors.to_hex('#C500FF'))
    plt.plot(x_list,k_delta_4, color = matplotlib.colors.to_hex('#FF0074'))
    plt.plot(x_list,y_list_pred_pid, color = 'g')
    plt.legend(['Setpoint','PID','K_rho = 0.50','K_rho = 0.75','K_rho = 1.00','K_rho = 1.20','Predicted-PID'])
    plt.title(f""" PID Control Plot (Kp = {Constants.KP}, Ki = {Constants.KI}, Kd = {Constants.KD})""")
    plt.ylabel("Vehicle Position (Output)")
    plt.xlabel("Time (s)")
    plt.grid(True)

# A function for visualising the plots
def show_plot():
    plt.show()

