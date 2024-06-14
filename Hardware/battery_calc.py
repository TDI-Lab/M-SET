import os
original_cwd = os.getcwd()
import math
import matplotlib.pyplot as plt
import numpy as np
try:
    from Hardware_constants import *
    from cdca_epos_executor import get_coords, read_cdca_output, read_default_output
except:
    from Hardware.Hardware_constants import *
    from Hardware.cdca_epos_executor import get_coords, read_cdca_output, read_default_output
import pandas as pd
    

def calc_travel_time(start,end,speed):
    x_dist = (get_coords(end, USE_CELL_COORDS)[0]) - (get_coords(start, USE_CELL_COORDS)[0])
    y_dist = (get_coords(end, USE_CELL_COORDS)[1]) - (get_coords(start, USE_CELL_COORDS)[1])

    dist = math.sqrt((x_dist**2 + y_dist**2))

    time = dist / speed

    print(x_dist, y_dist, dist, speed, time)

    return time

def parse_input(input_path, input_mode, speed):
    positions = []
    hover_times = []
    flight_times = []

    #parse the input
    for drone in input_path:
        previous_position = None
        for position in drone:
            if input_mode == "cdca":
                print(position[0], previous_position)
                positions.append(position[0])
                hover_times.append(float(position[1])) # waiting time
                if previous_position != None:
                    print(previous_position,position[0])
                    flight_times.append(calc_travel_time(previous_position,position[0],speed)) # travel time
                previous_position = position[0]
            elif input_mode == "default":
                positions.append(position)
                hover_times.append(SENSING_TIME) # sensing time
                if previous_position != None:
                    flight_times.append(calc_travel_time(previous_position,position,speed)) # travel time
                previous_position = position

    return positions, hover_times, flight_times

def calc_total_times(path, input_mode,speed):
    positions, hover_times, flight_times = parse_input(path, input_mode, speed)
    hover_time = sum([float(ht) for ht in hover_times])
    flight_time = sum([float(ft) for ft in flight_times])

    return hover_time, flight_time, hover_time+flight_time

def calc_total_energy(hover_power, flight_power, hover_time, flight_time):
    flight_energy = flight_power * flight_time
    hover_energy = hover_power * hover_time

    return hover_energy, flight_energy, flight_energy + hover_energy


"""
FROM GOOGLE COLAB NOTEBOOK
"""
def calc_poly(x, coefficients):
  deg=len(coefficients)-1

  total = 0
  for d in range(0,deg+1):
    total += coefficients[d]*(x**(deg-d))

  return total

def calc_loss_v(flight_length, method, voltage_poly_coeff):
  if method == "average":
    return avg_lost[min(len(avg_lost),flight_length)]*-1
  elif method == "polynomial":
    return calc_poly(flight_length, voltage_poly_coeff)*-1
  else:
    return 0
  
"""
def calc_loss_level(flight_length):
  return max(0,(flight_length/count_per_level))
"""

def calc_power_consumption(flight_length, method, voltage_poly_coeff,avg_current):
  power = avg_current*calc_loss_v(flight_length,method,voltage_poly_coeff)
  return power

def calc_energy_consumption(flight_time,method,voltage_poly_coeff,avg_current):
    # E = Pt = (IV)t
    total_energy = 0
    for t in range(1,math.ceil(flight_time)+1):
        if t != math.ceil(flight_time):
            total_energy += calc_power_consumption(t,method,voltage_poly_coeff,avg_current) * 1 # P*t
        else:
            total_energy += calc_power_consumption(t,method,voltage_poly_coeff,avg_current) * (math.ceil(flight_time)-flight_time)
    return total_energy



if __name__ == '__main__':
    os.chdir(original_cwd)
    hover_power = 1.7583430197900498
    flight_power = hover_power
    voltage_poly_coeff = [-3.93539944e-11,  1.49469540e-08, 1.17748602e-06, -2.04974506e-03, -3.75124204e-02] # hovering (from after take-off), voltage loss

    os.chdir(original_cwd)
    path = read_default_output("Hardware/epospaths/Final Demo/no cdca/4(3)cfnice.txt")
    input_mode = "default"
    speed = 0.1
    avg_current = 3.75

    hover_time, flight_time, total_time = calc_total_times(path, input_mode, speed)
    hover_energy, flight_energy, total_energy = calc_total_energy(hover_power,flight_power,hover_time,flight_time)

    max_t = 420 # This is just the max x of the graph
    best_fit_x=120
    expected_max_flight_duration = 420 # 7 mins
    battery_capacity = 3330
    X = [i for i in range(0,max_t)]
    epos_E = [calc_total_energy(hover_power,flight_power,t,0)[2] for t in range(0,max_t)]
    model_E = [calc_energy_consumption(t,"polynomial",voltage_poly_coeff,avg_current) for t in range(0,max_t)]
    weights = np.ones_like(X[:best_fit_x])
    weights[0] = 1000
    a, b = np.polyfit(X[:best_fit_x],model_E[:best_fit_x],1,w=weights)
    poly = [calc_poly(x, voltage_poly_coeff) for x in X]
    trapz = [-np.trapz(poly[:x])*avg_current for x in X]
    plt.plot(X,epos_E,label='Predicted by EPOS-based model', color='m')
    plt.plot(X,model_E,label='Predicted by Hardware-based model',color='y')
    plt.xlabel("Flight Duration (s)")
    plt.ylabel("Energy Consumption (J)")
    plt.title("Predicted Values of Energy Consumption by Flight Duration")
    plt.xlim(0)
    plt.ylim(0)
    plt.legend()
    plt.show()

    voltage_poly_roots = np.roots(voltage_poly_coeff)

    print(os.getcwd())
    df = pd.DataFrame(model_E)
    df.to_csv('energy2.csv', index=False)