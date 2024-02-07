# Read the "global config file" and create a config file for different system configurations

import os

# These lists are used to create the config file for the plan generation and EPOS
# Everything in the lists are just for the example, the actual parameters should be defined in the global config file
param_for_pathgen = [
    "dataset",
    "planNum",
    "agentsNum",
    "timeslots",
    "maxVisitedCells"
]

param_for_epos = [
    "airSpeed"
]

class configManager:
    # initialize the class
    def __init__(self, global_config_path):
        self.global_config_path = global_config_path
        self.configs = self.read_global_config(self.global_config_path)

    # read the global config file and store the configurations in a dictionary
    def read_global_config(self, global_config_file):
        try:
            with open(global_config_file, "r") as f:
                lines = f.readlines()
                configs = {}
                desc = []
                for line in lines:
                    stripped_line = line.strip()

                    if stripped_line.startswith("# "):
                        desc.append(line)
                    elif "=" in stripped_line:
                        key, value = stripped_line.split("=", 1)
                        key = key.strip()
                        value = value.strip()
                        configs[key] = {"value": value, "description": desc}
                        desc = []

        except IOError as err:
            print(f"An error occurred while reading the file: {err}")
            return None

        return configs
        
    # create the config file for the plan generation
    def write_pathgen_config_file(self, file_name = "pathGen_config.properties", storing_directory = "./"):
        file_path = os.path.join(storing_directory, file_name)  # the path of the config file

        try:
            with open(file_path, "w") as f:
                f.write("###### Configuration file for the path generation ######\n\n")
                for key in param_for_pathgen:
                    for line in self.configs[key]["description"]:
                        f.write(line)
                    f.write(f'{key} = {self.configs[key]["value"]}\n\n')
        except (IOError, KeyError) as err:
            print(f"An error occurred while writing the file: {err}")
            return None

    # create the config file for the plan generation, same as the previous function
    def write_epos_config_file(self, file_name = "epos_config.properties", storing_directory = "./"):
        file_path = os.path.join(storing_directory, file_name)  # the path of the config file

        try:
            with open(file_path, "w") as f:
                f.write("###### Configuration file for EPOS ######\n\n")
                for key in param_for_epos:
                    for line in self.configs[key]["description"]:
                        f.write(line)
                    f.write(f'{key} = {self.configs[key]["value"]}\n\n')
        except (IOError, KeyError) as err:
            print(f"An error occurred while writing the file: {err}")
            return None


# Example
#config = configManager("generation.properties")
#config.write_pathgen_config_file()
#config.write_epos_config_file("hi.properties")