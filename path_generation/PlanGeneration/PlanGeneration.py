import configparser
import csv

import numpy as np
from pathlib import Path

from MapSetting import MapSetting
from RouteGeneration import RouteGeneration


# The target without information of time, value is second
TARGET = [[0, 100, 100],
          [100, 100, 0],
          [100, 100, 0]]


def main(is_timeslots=False):
    # 1. Initialize the output directory
    dataset_path = '../datasets/' + dataset_name
    if not Path(dataset_path).exists():
        Path(dataset_path).mkdir()

    # 2. Set the target for drones, i,e., the required sensing value of each cell
    target_arr = []
    for i in range(len(TARGET)):
        for j in range(len(TARGET[0])):
            target_arr.append(TARGET[i][j])
    cells_num = len(target_arr)
    # output the target to .target file
    target_path = dataset_path + '/monitoring.target'
    with open(target_path, 'w', newline='', encoding='utf-8-sig') as targetFile:
        writer = csv.writer(targetFile)
        writer.writerow(target_arr)
    targetFile.close()

    # 3. Set the map of the sensing environment
    map_setting = MapSetting()
    map_setting.setting_default(cells_num, stations_num, height, total_hover_time, map_length)

    # 4. Generate a number of plans for each drone/agent
    for agent_id in range(agents_num):

        # 4a. Generate the plans for one drone/agent
        agent_plans = []
        for plan_id in range(plans_num):

            # (1) initialize and calculate the shortest route for the drone
            # initialization
            drone_route = RouteGeneration()
            # the departure and destination (charging station)
            station_idx = agent_id % stations_num
            # the number of cells that the drone visits
            visited_cells_num = plan_id % max_visited_cells_num + 1
            # to find a route
            drone_route.find_route(station_idx, visited_cells_num, map_setting)

            # (2) output the list of visited cells and normalized energy consumption for the drone
            visited_cells_list = drone_route.visited_cells
            plan_cost = float(drone_route.energy_consumption / battery_capacity)

            # (3) transform into a plan format.
            if not is_timeslots:
                # plans do not have time information
                plan = np.array(drone_route.hover_time_arr)
            else:
                # plans have time information, prevent over-sensing and under-sensing
                plan_dim_len = cells_num * total_hover_time
                plan = np.zeros(plan_dim_len)
                visited_cells_num = len(visited_cells_list)
                # the average time that a drone spends on each visited cell
                length_of_timeslot = int(total_hover_time / visited_cells_num)
                # the order that a drone visits a cell
                visited_order = 0
                for v_cell in visited_cells_list:
                    # the starting index in a dimension of a plan
                    plan_dim_id_start = int(v_cell) * total_hover_time
                    """
                    For each visited cell, find the starting index of the plan (every # of timeslots), then find the
                    order of the visited cell (average time spent on each cell, as the visited cell with former order
                    will occupy the former time), and the average time spent on the current visited cell.
                    """
                    for slot in range(length_of_timeslot):
                        plan[plan_dim_id_start + visited_order * length_of_timeslot + slot] += 1
                    visited_order += 1

            plan_dict = {'plan': plan, 'cost': plan_cost}
            agent_plans.append(plan_dict)

        # 4b. Output the plans of the drone into the datasets dir
        agent_plans_path = dataset_path + '/agent_' + str(agent_id) + '.plans'
        with open(agent_plans_path, 'w', newline='', encoding='utf-8-sig') as planFile:
            for index, agent_plan in enumerate(agent_plans):
                # write the cost of plan and write each dimension of the plan
                line = str(agent_plan['cost']) + ':' + ','.join(map(str, agent_plan['plan']))
                # write a line to the file
                if index == len(agent_plans) - 1:
                    planFile.write(line)
                else:
                    planFile.write(line + '\n')
        planFile.close()


if __name__ == '__main__':

    config = configparser.ConfigParser()
    config.read('../conf/generation.properties')

    dataset_name = config.get('plan', 'dataset')
    plans_num = int(config.get('plan', 'planNum'))
    agents_num = int(config.get('plan', 'agentsNum'))
    total_hover_time = int(config.get('plan', 'timeslots'))
    max_visited_cells_num = int(config.get('plan', 'maxVisitedCells'))

    stations_num = int(config.get('map', 'stationsNum'))
    height = int(config.get('map', 'height'))
    map_length = int(config.get('map', 'mapLength'))

    battery_capacity = int(config.get('power', 'batteryCapacity'))

    main()

