from os import listdir, system
import os
import fnmatch
import matplotlib.pyplot as plt
import numpy as np
import csv
from battery_calc import read_default_output, read_cdca_output, calc_total_times, calc_total_energy, calc_energy_consumption

def format_data(in_filename, out_filename):
    with open(in_filename, 'r') as f:
        lines = f.readlines()
    
    # reformat
    lines = [line.replace(' ', '') for line in lines]
    lines = [line.replace('(', '') for line in lines]
    lines = [line.replace(')', '') for line in lines]
    lines = [line.replace('/?/', ',') for line in lines]
    # write lines in the file
    with open(out_filename, 'w') as f:
        f.writelines(lines)

    return f

def read_formatted_data(out_filename):
    vals = []
    colours = []
    yerrs = []
    include = True
    with open(out_filename) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
        for row in reader: # each row is a list`
            try:

                if ("hover" in row[5].lower()) or (fnmatch.fnmatchcase(row[5].lower(),"position?")):
                    colour="red"
                    yerr=25
                    include=False
                elif "waiting" in row[5].lower():
                    include=True
                elif "moving" in row[5].lower():
                    colour="green"
                    yerr=25
                    include=True
                else:
                    colour="blue"
                    yerr=None
                    include=False

                if include == True:
                    vals.append([row[0], row[1], row[2], row[3], row[4], row[5]])
                colours.append(colour)
                yerrs.append(yerr)
            except:
                pass

    return np.array(vals)

def retrieve_fields(vals,avg_current):
    time = vals[:, 0].astype(float)
    val_count = vals[:, 1].astype(float)
    id = vals[:, 2]
    batteryP = vals[:, 3].astype(float)
    voltage = vals[:, 3].astype(float)
    power = [avg_current*val for val in vals[:,3].astype(float)]

    return time, val_count, id, batteryP, voltage, power

def get_vals_by_id(vals):
        vals_by_id = [[],[],[],[]]
        for row in vals:
            id = int(float(row[2]))
            vals_by_id[id-1].append([float(element) for element in row[:4]]) # CHANGE TO row[5] if file includes battery level
    
        return vals_by_id

def get_raw_flight_times(vals_by_id,drones):
    raw_flight_times = []
    raw_start_times = []
    raw_end_times = []
    for drone in drones:
        start=vals_by_id[drone-1][0]
        Vs=[float(row[3]) for row in vals_by_id[drone-1]] # Should be row[3] for files without batteryLevel
        end=vals_by_id[drone-1][Vs.index(min(Vs))] # Occassionally this can cause the end time to be earlier than it should be, if there is an outlier low voltage before the end

        raw_start = start[0]
        raw_end = end[0]

        raw_flight_time = raw_end - raw_start

        print(drone, raw_flight_time)

        raw_start_times.append(raw_start)
        raw_end_times.append(raw_end)
        raw_flight_times.append(raw_flight_time)

    return raw_start_times, raw_end_times, raw_flight_times

def get_count_flight_times(drones, vals_by_id):
    # Finding the start and end times
    count_start_times = []
    count_end_times = []
    count_flight_times = []
    weighting_factor = 0.125
    for drone in [1,2,3,4]:
        if drone in drones:
            start=vals_by_id[drone-1][0]
            Vs=[float(row[3]) for row in vals_by_id[drone-1]]
            weighted_minimums=[v/(Vs.index(v)+1) for v in Vs]

            end=vals_by_id[drone-1][Vs.index(min(Vs))] # NOTE THAT THIS CAN SOMETIMES BE THROWN OFF BY OUTLIERS (e.g. if there is an outlier of low voltage early in the flight)
            print(vals_by_id[drone-1][int(end[1])-5:])

            count_start = start[1]
            count_end = end[1]
            count_flight_time = count_end - count_start
        else:
            count_start = -1
            count_end = -1
            count_flight_time = -1

        print(drone, count_flight_time)

        count_start_times.append(count_start)
        count_end_times.append(count_end)
        count_flight_times.append(count_flight_time)

    return count_start_times, count_end_times, count_flight_times

def replace_val(x, val, desired_val):
  if x == val:
    return val
  else:
    return x

def make_counts_contiguous(drones, counts):
    # Make the counts contiguous (regardless of removed values)
    for drone in drones:
        for c in range(1,len(counts[drone-1])-1):
            if counts[drone-1][c-1] != counts[drone-1][c]:
                counts[drone-1]=[replace_val(x, counts[drone-1][c], counts[drone-1][c-1]+1) for x in counts[drone-1]]

    return counts

def get_voltages_lost(drones,vals_by_id):
    voltages_lost = [[],[],[],[]]
    counts = [[],[],[],[]]
    for drone in drones:
    # Subtract the initial voltage from the voltage at each count

        # CHANGE to row[4] for Voltage if batteryLevel included in logging data
        voltage_lost = [float(row[3]) - vals_by_id[drone-1][0][3] for row in vals_by_id[drone-1]] # Note that the length of voltage_lost will be greater than count_end, as it will include the voltages from after the drone landed
        counts[drone-1] = [float(row[1]) for row in vals_by_id[drone-1]]
        voltages_lost[drone-1] = voltage_lost

    return voltages_lost, counts

def initialise_voltage_metrics(min_count,max_count):
    total_voltages_lost = []
    avg_lost = []
    divisor = []
    for count in range(0, max_count-min_count):
        total_voltages_lost.append(0)
        avg_lost.append(0)
        divisor.append(0)

    V=np.full((4,max_count-min_count),np.nan)
    
    return V, total_voltages_lost, avg_lost, divisor

def calc_average_vloss(min_count, max_count, avg_lost, total_voltages_lost, divisor):
    for flight_time in range(0, max_count-min_count):
        try:
            avg_lost[flight_time] = total_voltages_lost[flight_time]/min(len(drones),divisor[flight_time])
        except ZeroDivisionError:
            avg_lost[flight_time] = np.nan

    return avg_lost

def calculate_voltage_metrics(drones, V, total_voltages_lost, avg_lost, divisor, counts, voltages_lost, count_end_times, min_count, max_count):
    for count in range(min_count, max_count):
        flight_time = count-min_count

        for drone in drones:
            try:
                print(count_end_times[drone-1])
                if count <= count_end_times[drone-1]:
                    index = counts[drone-1].index(count)
                    total_voltages_lost[flight_time] += voltages_lost[drone-1][index]
                    V[drone-1][flight_time] = voltages_lost[drone-1][index]
                    divisor[flight_time] += 1
            except:
                pass
                print(drone,count)

    avg_lost = calc_average_vloss(min_count, max_count, avg_lost, total_voltages_lost, divisor)

    return V, total_voltages_lost, avg_lost, divisor

def plot_flighttime_voltageloss(drones,min_count,max_count,avg_lost,V):
    print(drones)
    for drone in drones:
        flight_time = [i for i in range(0,max_count-min_count)]
        print(flight_time)
        voltage_loss = V[drone-1]
        print(len(flight_time), len(voltage_loss))
        plt.scatter(flight_time, voltage_loss, label=drone, marker='x', s=15)

    flight_time = [i for i in range(0,max_count-min_count)]
    avg_voltage_loss = avg_lost
    plt.scatter(flight_time, avg_voltage_loss,label='average',marker='x', s=15)
    plt.xlabel("Flight Time (count)")
    plt.ylabel("Voltage loss (avg)")
    plt.title('Flight Time (count) vs Voltage loss (avg)')
    plt.legend()
    plt.show()

def get_total_power(avg_current, total_voltages_lost):
    Ps = [v*avg_current for v in total_voltages_lost]
    return Ps

def get_total_energy(avg_current,total_voltages_lost):
    Ps = get_total_power(avg_current, total_voltages_lost)
    print(Ps)
    print(sum(Ps))
    Energy = -sum(Ps)
    return Energy

def batch_recorded_total_energy(filenames, drones_list):
    total_energy = []

    for i in range(0,len(filenames)):
        in_filename=filenames[i][0]
        out_filename=filenames[i][1]
        drones = drones_list[i]

        format_data(in_filename, out_filename)
        vals = read_formatted_data(out_filename)

        # Regular vals
        avg_current = 3.75
        time, val_count, id, batteryP, voltage, power = retrieve_fields(vals,avg_current)

        # Vals split by id
        vals_by_id = get_vals_by_id(vals)

        voltages_lost, counts = get_voltages_lost(drones,vals_by_id)
        counts = make_counts_contiguous(drones,counts)

        count_start_times, count_end_times, count_flight_times = get_count_flight_times(drones, vals_by_id)
        min_count = int(max(count_start_times)) # latest flight end
        max_count = int(max(count_end_times)) # latest flight start

        V, total_voltages_lost, avg_lost, divisor = initialise_voltage_metrics(min_count,max_count)
        V, total_voltages_lost, avg_lost, divisor = calculate_voltage_metrics(drones, V, total_voltages_lost, avg_lost, divisor, counts, voltages_lost, count_end_times, min_count, max_count)

        total_path_energy = get_total_energy(avg_current, total_voltages_lost)

        if total_path_energy == 0:
            total_path_energy = np.nan

        total_energy.append(total_path_energy)

    return total_energy

def batch_predicted_total_energy(ndrones, path_filenames, input_modes, speeds, avg_current):
    # Each file has a different number of drones in it

    voltage_poly_coeff = [-3.93539944e-11,  1.49469540e-08, 1.17748602e-06, -2.04974506e-03, -3.75124204e-02]
    model_total_energy = []
    epos_total_energy = []

    for i in range(0,len(path_filenames)):
        print(path_filenames[i], input_modes[i])
        if input_modes[i] == "default":
            path = read_default_output(path_filenames[i])
        elif input_modes[i] == "cdca":
            path = read_cdca_output(path_filenames[i]) 

        path = path[:ndrones[i]]
        hover_time, flight_time, total_time = calc_total_times(path, input_modes[i], speeds[i])
        print(total_time)

        # Hardare model
        model_total_path_energy = calc_energy_consumption(total_time,"polynomial",voltage_poly_coeff,avg_current)

        # Path generation model
        epos_total_path_energy = epos_calc_predicted_energy(path,input_modes[i])[2]

        model_total_energy.append(model_total_path_energy)
        epos_total_energy.append(epos_total_path_energy)

    return epos_total_energy, model_total_energy

def epos_calc_predicted_energy(path, input_mode):
    hover_power = 1.7583430197900498
    flight_power = hover_power
    
    speed = 0.1
    avg_current = 3.75

    hover_time, flight_time, total_time = calc_total_times(path, input_mode, speed)
    hover_energy, flight_energy, total_energy = calc_total_energy(hover_power,flight_power,hover_time,flight_time)

    return hover_energy, flight_energy, total_energy

def plot_all_results():
    # All on one
    plt.plot(x,recorded_basic_energy_consumptions, label="Recorded: Basic cdca")
    plt.plot(x,recorded_pf_energy_consumptions, label="Recorded: Potenial Fields cdca")
    plt.plot(x,epos_predicted_basic_energy_consumptions, label="Predicted by EPOS: Basic cdca")
    plt.plot(x,model_predicted_basic_energy_consumptions, label="Predicted by model: Basic cdca")
    plt.plot(x,epos_predicted_pf_energy_consumptions, label="Predicted by EPOS: Potential Fields cdca")
    plt.plot(x,model_predicted_pf_energy_consumptions, label="Predicted by model: Potential Fields cdca")
    plt.plot(x,epos_predicted_nocdca_energy_consumptions, label="Predicted by EPOS: No cdca")
    plt.plot(x,model_predicted_nocdca_energy_consumptions, label="Predicted by model: No cdca")
    plt.xlabel("Number of drones in path")
    plt.ylabel("Energy Consumption (J)")
    plt.xticks([1,2,3,4])
    plt.legend()
    plt.show()

def plot_by_cdca_type(cdca_type):
    # Compare (recording, path gen model, hardware model), for each cdca type
    if cdca_type == "basic":
        plt.plot(x,recorded_basic_energy_consumptions, label="Recorded from hardware", color='c', marker='o')
        plt.plot(x,epos_predicted_basic_energy_consumptions, label="Predicted by EPOS-based model", color='m', marker='o')
        plt.plot(x,model_predicted_basic_energy_consumptions, label="Predicted by Hardware-based model", color='y', marker='o')
        plt.title("Recorded vs. Predicted Values of Energy Consumption With Scheduliing CA")
    else:
        plt.plot(x,recorded_pf_energy_consumptions, label="Recorded from hardware", color='c', marker='o')
        plt.plot(x,epos_predicted_pf_energy_consumptions, label="Predicted by EPOS-based model", color='m', marker='o')
        plt.plot(x,model_predicted_pf_energy_consumptions, label="Predicted by Hardware-based model", color='y', marker='o')
        plt.title("Recorded vs. Predicted Values of Energy Consumption With Potential Fields CA")

    plt.xlabel("Number of drones in path")
    plt.ylabel("Energy Consumption (J)")
    plt.xticks([1,2,3,4])
    plt.ylim(0,1100)
    plt.legend()
    plt.show()

def plot_by_value_origin():
    # Compare (no cdca, scheduling ca, pf ca), for each (recording, paht gen model, hardware model)

    fig, axs = plt.subplots(1,2)
    
    axs[0].plot(x,epos_predicted_nocdca_energy_consumptions, label="Predicted by EPOS: No cdca")
    axs[0].plot(x,epos_predicted_basic_energy_consumptions, label="Predicted by EPOS: Basic cdca")
    axs[0].plot(x,epos_predicted_pf_energy_consumptions, label="Predicted by EPOS: Potential Fields cdca")

    axs[1].plot(x,model_predicted_nocdca_energy_consumptions, label="Predicted by model: No cdca")
    axs[1].plot(x,model_predicted_basic_energy_consumptions, label="Predicted by model: Basic cdca")
    axs[1].plot(x,model_predicted_pf_energy_consumptions, label="Predicted by model: Potential Fields cdca")

    titles = ["Predicted by EPOS","Predicted by Model"]
    for c, ax in enumerate(axs):
        ax.set_xlabel("Number of drones in path")
        ax.set_ylabel("Energy Consumption (J)")
        ax.set_xticks([1,2,3,4])
        ax.set_ylim(0,1000)
        ax.legend()
        ax.set_title(titles[c])
    plt.suptitle("Predicted values for each cdca method, for each prediction method")
    plt.show()

def plot_nocdca_epos_vs_model():
    plt.plot(x,epos_predicted_nocdca_energy_consumptions, label="Predicted by EPOS-based model", color='m', marker='o')
    plt.plot(x,model_predicted_nocdca_energy_consumptions, label="Predicted by Hardware-based model", color='y', marker='o')
    plt.xlabel("Number of drones in path")
    plt.ylabel("Energy Consumption (J)")
    plt.xticks([1,2,3,4])
    plt.title("Predicted Values of Energy Consumption with No CDCA Algorithms Applied")
    plt.legend()
    plt.show()

if __name__=="__main__":
    drones = [[1],[1,2],[1,2,3],[1,2,3,4]]
    basic_cdca_filenames = [["Hardware/Hardware Results/eval_1dronescdca_manual","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/eval_2dronescdca_manual","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/eval_3dronescdca_manual","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/eval_4dronescdca_manual2","Hardware/Hardware Results/dataReformatted.txt"]]
    pf_cdca_filenames = [["Hardware/Hardware Results/eval_1dronescdca_manual","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/pf_2drones_cdca","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/pf_3drones_cdca","Hardware/Hardware Results/dataReformatted.txt"],["Hardware/Hardware Results/pf_4drones_cdca2","Hardware/Hardware Results/dataReformatted.txt"]]    
    basic_cdca_path_filenames = []
    pf_cdca_path_filenames = []
    
    recorded_basic_energy_consumptions = batch_recorded_total_energy(basic_cdca_filenames, drones)
    recorded_pf_energy_consumptions = batch_recorded_total_energy(pf_cdca_filenames, [[1],[1,2],[1,2,3],[1,2,3,4]])

    basic_path_filenames = ["Hardware/Experiments/Working/manual/eval_1dronecdca.txt", "Hardware/Experiments/Working/manual/eval_2dronecdca.txt", "Hardware/Experiments/Working/manual/eval_3dronecdca.txt", "Hardware/Experiments/Working/manual/eval_4dronecdca.txt"]
    pf_path_filenames = ["Hardware/Experiments/Working/manual/eval_1dronecdca.txt","Hardware/Experiments/Potential fields/pf_cdca_2drones.txt","Hardware/Experiments/Potential fields/pf_cdca_3drones.txt","Hardware/Experiments/Potential fields/pf_cdca_4drones2.txt"]
    input_modes = ["cdca","cdca","cdca","cdca"]
    speeds = [0.1,0.1,0.1,0.1]
    avg_current = 3.75
    epos_predicted_basic_energy_consumptions, model_predicted_basic_energy_consumptions = batch_predicted_total_energy([4,4,4,4], basic_path_filenames, input_modes, speeds, avg_current)
    epos_predicted_pf_energy_consumptions, model_predicted_pf_energy_consumptions = batch_predicted_total_energy([4,4,4,4], pf_path_filenames, input_modes, speeds, avg_current)

    nocdca_path_filenames = ["Hardware/Experiments/Working/manual/eval_4droneparsed.txt","Hardware/Experiments/Working/manual/eval_4droneparsed.txt","Hardware/Experiments/Working/manual/eval_4droneparsed.txt","Hardware/Experiments/Working/manual/eval_4droneparsed.txt"]
    epos_predicted_nocdca_energy_consumptions, model_predicted_nocdca_energy_consumptions = batch_predicted_total_energy([1,2,3,4], nocdca_path_filenames, input_modes, speeds, avg_current)

    x = [len(d) for d in drones]
    print("")
    print(x, recorded_basic_energy_consumptions)
    print(x, recorded_pf_energy_consumptions)
    print(x, epos_predicted_basic_energy_consumptions)
    print(x, model_predicted_basic_energy_consumptions)

    plot_by_cdca_type("basic")
    plot_by_cdca_type("pf")
    plot_nocdca_epos_vs_model()