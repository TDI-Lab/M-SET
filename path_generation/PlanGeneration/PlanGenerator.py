import configparser
import csv
from os import listdir, mkdir
from os.path import isdir
from shutil import rmtree, copy2

import numpy as np
from pathlib import Path

from path_generation.PlanGeneration.MapSetting import MapSetting
from path_generation.PlanGeneration.RouteGeneration import RouteGeneration
from path_generation.PlanGeneration.PlanGenerationProperties import PlanGenerationProperties


class PlanGenerator:

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()

        config = configparser.ConfigParser()
        config.read(f'{self.parent_path}/conf/generation.properties')

        self.properties = PlanGenerationProperties(
            dataset_name=config.get('plan', 'dataset'),
            plans_num=int(config.get('plan', 'planNum')),
            agents_num=int(config.get('plan', 'agentsNum')),
            total_hover_time=int(config.get('plan', 'timeslots')),
            path_mode=config.get("plan", "pathMode"),
            stations_num=int(config.get('map', 'stationsNum')),
            height=int(config.get('map', 'height')),
            map_length=int(config.get('map', 'mapLength')),
            battery_capacity=float(config.get('power', 'batteryCapacity'))
        )

    def clean_datasets(self):
        target = f"{self.parent_path}/datasets"
        if not isdir(target):
            mkdir(target)
        datasets = listdir(target)
        for dataset in datasets:
            if isdir(dataset):
                rmtree(dataset)

    def generate_plans(self, mission_file=None, is_timeslots=False):
        # 1. Initialize the output directory
        dataset_path = f'{self.parent_path}/datasets/'
        if not Path(dataset_path).exists():
            Path(dataset_path).mkdir()
        dataset_path += f"{self.properties.dataset_name}"
        if not Path(dataset_path).exists():
            Path(dataset_path).mkdir()

        #  copy if mission file defined
        if mission_file is not None:
            new_mission_file = f"{dataset_path}/{self.properties.dataset_name}.csv"
            copy2(mission_file, new_mission_file)

        # 2. Set the map of the sensing environment
        map_setting = MapSetting()
        map_setting.setting_read(
            f"{self.parent_path}/datasets/{self.properties.dataset_name}/{self.properties.dataset_name}.csv"
        )

        # 3. Set the target for drones, i,e., the required sensing value of each cell
        cells = map_setting.cells
        target_arr = [cell["value"] for cell in cells]
        # output the target to .target file
        target_path = (f"{self.parent_path}/datasets/{self.properties.dataset_name}/"
                       f"{self.properties.dataset_name}.target")
        with open(target_path, 'w', newline='', encoding='utf-8-sig') as targetFile:
            writer = csv.writer(targetFile)
            writer.writerow(target_arr)
        targetFile.close()

        # 4. Generate a number of plans for each drone/agent
        for agent_id in range(self.properties.agents_num):

            # 4a. Generate the plans for one drone/agent
            agent_plans = []
            for plan_id in range(self.properties.plans_num):
                # (1) initialize and calculate the shortest route for the drone
                # initialization
                drone_route = RouteGeneration()
                # the departure and destination (charging station)
                station_idx = agent_id % self.properties.stations_num
                # to find a route
                drone_route.find_route(station_idx, self.properties.path_mode, map_setting, plan_id)

                # (2) output the list of visited cells and normalized energy consumption for the drone
                plan_cost = float(drone_route.energy_consumption / self.properties.battery_capacity)

                path_taken = drone_route.path_taken

                # (3) transform into a plan format.
                plan = np.array(drone_route.hover_time_arr)

                plan_dict = {'plan': plan, 'cost': plan_cost, "path": path_taken}
                agent_plans.append(plan_dict)

            #  sort agent plans
            agent_plans.sort(key=lambda x: x["cost"], reverse=True)
            # 4b. Output the plans of the drone into the datasets dir
            agent_plans_path = f"{dataset_path}/agent_{agent_id}.plans"
            agent_paths = f"{dataset_path}/agent_{agent_id}.paths"
            with open(agent_plans_path, 'w', newline='', encoding='utf-8') as planFile, \
                    open(agent_paths, 'w', newline='', encoding='utf-8') as pathFile:
                for index, agent_plan in enumerate(agent_plans):
                    # write the cost of plan and write each dimension of the plan
                    plan_line = str(agent_plan['cost']) + ':' + ','.join(map(str, agent_plan['plan']))
                    path_line = ",".join(agent_plan["path"])
                    # write a line to the file
                    if index == len(agent_plans) - 1:
                        planFile.write(plan_line)
                        pathFile.write(path_line)
                    else:
                        planFile.write(plan_line + '\n')
                        pathFile.write(path_line + "\n")
            planFile.close()
        return 0
