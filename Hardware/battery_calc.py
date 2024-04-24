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

def calc_power_consumption(flight_length, method, voltage_poly_coeff):
  power = avg_current*calc_loss_v(flight_length,method,voltage_poly_coeff)
  return power

def calc_energy_consumption(flight_time,method,voltage_poly_coeff):
    # E = Pt = (IV)t
    total_energy = 0
    for t in range(1,math.ceil(flight_time)+1):
        if t != math.ceil(flight_time):
            total_energy += calc_power_consumption(t,method,voltage_poly_coeff) * 1 # P*t
        else:
            total_energy += calc_power_consumption(t,method,voltage_poly_coeff) * (math.ceil(flight_time)-flight_time)
    return total_energy



if __name__ == '__main__':
    hover_power = 1.6152318136132513#1.5308683747745433# 1.1297552885300068 # W
    flight_power = hover_power # 1.128288294545468 # W
    voltage_poly_coeff=[-4.60990873e-10, 1.76874340e-07, -1.85072541e-05, -1.00546782e-03, -9.19204536e-03]

    os.chdir(original_cwd)
    #path = [[[[0.0, 0.0], 1], [[1.0, 1.0], 4], [[1.0, 2.0], 4], [[2.0, 1.0], 8], [[0.0, 0.0], 1]], [[[4.0, 0.0], 2]], [[[4.0, 3.0], 1], [[3.0, 2.0], 9], [[2.0, 2.0], 4], [[3.0, 1.0], 4], [[4.0, 3.0], 1]], [[[0.0, 3.0], 1], [[1.0, 2.0], 1], [[2.0, 2.0], 0], [[2.0, 1.0], 0], [[1.0, 1.0], 1], [[0.0, 3.0], 1]]]
    path = read_default_output("Hardware/epospaths/Final Demo/no cdca/4(3)cfnice.txt")
    input_mode = "default"
    speed = 0.1
    avg_current = 3.75

    hover_time, flight_time, total_time = calc_total_times(path, input_mode, speed)
    hover_energy, flight_energy, total_energy = calc_total_energy(hover_power,flight_power,hover_time,flight_time)
    
    print("")
    print("Total air time is %s s" % total_time)
    print("EPOS:  Total energy is %s J" % total_energy)
    
    model_energy = calc_energy_consumption(total_time,"polynomial",voltage_poly_coeff)
    print("Model: Total energy is %s J" % model_energy)


    max_air_time = 420 # 7 minutes
    battery_capacity = 925
    X = [i for i in range(0,max_air_time)]
    epos_E = [calc_total_energy(hover_power,flight_power,t,0)[2] for t in range(0,max_air_time)]
    model_E = [calc_energy_consumption(t,"polynomial",voltage_poly_coeff) for t in range(0,max_air_time)]
    weights = np.ones_like(X)
    weights[0] = 1000
    a, b = np.polyfit(X,model_E,1,w=weights)
    poly = [calc_poly(x, voltage_poly_coeff) for x in X]
    trapz = [-np.trapz(poly[:x])*avg_current for x in X]
    plt.plot(X,epos_E,label='epos battery model')
    plt.plot(X,model_E,label='hardware battery model',color='green')
    #plt.plot(X,trapz, label='trapz')
    plt.plot(X,[a*x+b for x in X], label='line of best fit (weighted to pass through (0,0))', linestyle='dashdot') # Line of best fit for model_E. Issues as it doesn't intersect (0,0)
    plt.plot(X,[battery_capacity for x in X], label="battery capacity (925J)", color='red', linestyle='dashed')
    plt.axvline(388.5, label='Expected max. flight duration (388s)', color='red', linestyle='dotted')
    plt.xlabel("Mission duration (s)")
    plt.ylabel("Energy Expended (J)")
    plt.title("Comparison of battery consumption models")
    plt.xlim(0)
    plt.ylim(0)
    plt.legend()
    plt.show()

    voltage_poly_roots = np.roots(voltage_poly_coeff)
    #print(max_expected_air_time)