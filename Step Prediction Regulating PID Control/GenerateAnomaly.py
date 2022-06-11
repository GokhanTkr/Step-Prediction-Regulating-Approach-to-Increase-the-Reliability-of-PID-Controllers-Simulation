"""
This part's job is to create the anomalies for the controllers to simulate the real world behaviour of the controllers
"""






import Constants
import random

def generate_anomaly():

    anomaly_list = []
    
    # Generating random anomalies in every step and divide by 10 to decrease the aggresiveness
    for a in range (Constants.LOOP_CONSTANT):
        anomaly = Constants.ANOMALY_CONSTANT * random.randint(Constants.ANOMALY_START_POINT,Constants.ANOMALY_END_POINT)
        anomaly_list.append(anomaly)
    return anomaly_list


list_anomaly = generate_anomaly()