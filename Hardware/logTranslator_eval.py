from os import listdir, system
import os
import fnmatch
import matplotlib.pyplot as plt
import numpy as np
import csv
#working_path = "/content/drive/My Drive/bio-inspired"

def format_data(in_filename, out_filename):
    with open(in_filename, 'r') as f: # I think it is eval_4dronescdca_manual2 that you need
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

                # There is a record with status == hovering.Startingtofollowplans when follow_plans() is first called

                if include == False:
                    print(row[5]) # Rows that have been removed

                if include == True:
                    vals.append([row[0], row[1], row[2], row[3], row[4], row[5]])
                colours.append(colour)
                yerrs.append(yerr)
            except:
                pass

    return np.array(vals)

def retrieve_fields(vals):
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
        Vs=[float(row[3]) for row in vals_by_id[drone-1]] # SHOULD BE row[3] for files without batteryLevel
        #print(Vs)
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
    count_flight_times = [] # may need to remove this
    weighting_factor = 0.125
    for drone in [1,2,3,4]:
        if drone in drones:
            start=vals_by_id[drone-1][0]
            Vs=[float(row[3]) for row in vals_by_id[drone-1]]
            end=vals_by_id[drone-1][Vs.index(min(Vs))] # NOTE THAT THIS CAN SOMETIMES BE THROWN OFF BY OUTLIERS (e.g. if there is an outlier of low voltage early in the flight)
            # print("")
            # print([v for v in Vs])
            # print([(Vs.index(v)+1) for v in Vs])
            weighted_minimums=[v/(Vs.index(v)+1) for v in Vs]
            #print(weighted_minimums)
            # print(min(weighted_minimums))
            end=vals_by_id[drone-1][weighted_minimums.index(min(weighted_minimums))] # NOTE THAT THIS OVERRIDES THE EARLIER VALUE OF end IF THAT WAS UNCOMMENTED
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
    # Assumes all drones start at same count?

        # CHANGE to row[4] for Voltage if batteryLevel included in logging data
        voltage_lost = [float(row[3]) - vals_by_id[drone-1][0][3] for row in vals_by_id[drone-1]] # Note that the length of voltage_lost will be greater than count_end, as it will include the voltages from after the drone landed
        counts[drone-1] = [float(row[1]) for row in vals_by_id[drone-1]]
        voltages_lost[drone-1] = voltage_lost

    return voltages_lost, counts

def initialise_voltage_metrics():
    total_voltages_lost = []
    avg_lost = []
    divisor = []
    for count in range(0, max_count-min_count):
        total_voltages_lost.append(0)
        avg_lost.append(0)
        divisor.append(0)

    V=np.full((4,max_count-min_count),np.nan)
    
    return V, total_voltages_lost, avg_lost, divisor

def calc_average_vloss():
    for flight_time in range(0, max_count-min_count):
        try:
            avg_lost[flight_time] = total_voltages_lost[flight_time]/min(len(drones),divisor[flight_time])
        except ZeroDivisionError:
            avg_lost[flight_time] = np.nan

    return avg_lost

def calculate_voltage_metrics():
    for count in range(min_count, max_count):
        flight_time = count-min_count

        for drone in drones:
            try:
                #print(count_end_times[drone-1])
                if count <= count_end_times[drone-1]:
                    index = counts[drone-1].index(count)
                    total_voltages_lost[flight_time] += voltages_lost[drone-1][index]
                    V[drone-1][flight_time] = voltages_lost[drone-1][index]
                    divisor[flight_time] += 1
            except:
                pass
                print(drone,count)
                #print(counts[drone-1].index(count)

    avg_lost = calc_average_vloss()

    return V, total_voltages_lost, avg_lost, divisor

def plot_flighttime_voltageloss():
    print(drones)
    for drone in drones:
        #voltages_len = len(voltages_lost[drone-1][min_count:max_count]) # check
        flight_time = [i for i in range(0,max_count-min_count)]
        print(flight_time)
        voltage_loss = V[drone-1]
        #print(voltage_loss)
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

if __name__=="__main__":
    in_filename = "Hardware/Hardware Results/eval_4dronescdca_manual2"
    out_filename = "Hardware/Hardware Results/dataReformatted.txt"
    drones = [1,2,3,4]

    format_data(in_filename, out_filename)
    vals = read_formatted_data(out_filename)

    # Regular vals
    avg_current = 3.75
    time, val_count, id, batteryP, voltage, power = retrieve_fields(vals)

    # Vals split by id
    vals_by_id = get_vals_by_id(vals)

    voltages_lost, counts = get_voltages_lost(drones,vals_by_id)
    counts = make_counts_contiguous(drones,counts) # REMOVE?

    count_start_times, count_end_times, count_flight_times = get_count_flight_times(drones, vals_by_id)
    min_count = int(max(count_start_times)) # latest flight end
    max_count = int(max(count_end_times)) # latest flight start

    V, total_voltages_lost, avg_lost, divisor = initialise_voltage_metrics()
    V, total_voltages_lost, avg_lost, divisor = calculate_voltage_metrics()

    plot_flighttime_voltageloss()
