# Read the "global config file" and create a config file for different system configurations

import os
import configparser
from pathlib import Path

# These lists are used to create the config file for the plan generation and EPOS
# Everything in the lists are just for the example, the actual parameters will be defined in the global config file
sections_for_pathgen = [
    "plan",
    "map",
    "power",
]

sections_for_epos = [
    "epos"
]

class configManager:
    # initialize the class
    def __init__(self, config_path):
        parent_path = Path(__file__).parent.resolve()
        config_file = parent_path / config_path
        self.config_path = config_file
        self.configs = configparser.ConfigParser()
        self.read_global_config()

    # read the global config file and store the configurations in a dictionary
    def read_global_config(self):
        if os.path.exists(self.config_file) == False:
            raise FileNotFoundError(f"The config file {self.config_file} does not exist.")
        self.config.read(self.config_file)

    # get the value of a parameter from the global config file
    # section: the section of the parameter, e.g. [plan], [map], [power], [epos]
    # option: the name of the parameter, e.g. num_paths, map_file, power_file, etc.
    def get_property(self, section, option):
        return self.config.get(section, option)
    
    def set_property(self, section, option, value):
        if not self.config.has_section(section):
            self.config.add_section(section)
        self.config.set(section, option, value)
        
    # create the config file for the plan generation
    def write_pathgen_config_file(self):
        parent_path = Path(__file__).parent.resolve()
        save_path = parent_path / 'PlanGeneration/conf/generation.properties'

        try:
            # Filter the sections that are used for the path generation
            save_content = configparser.ConfigParser()
            for section in sections_for_pathgen:
                if self.config.has_section(section):
                    save_content.add_section(section)
                    for key, value in self.config[section].items():
                        save_content.set(section, key, value)

            with open(save_path, "w") as f:
                save_content.write(f)
            f.close()
        except Exception as e:
            print(f"An error occurred while writing the config file: {str(e)}")

    # create the config file for the plan generation, but without the section headers
    # In the global config file, the parameter is stored in the [epos] section, just to make it easier to read and recognize the parameters
    def write_epos_config_file(self):
        parent_path = Path(__file__).parent.resolve()
        save_path = parent_path / 'EPOS/conf/epos.properties'

        try:
            # Filter the sections that are used for the EPOS
            with open(save_path, "w") as f:
                for section in sections_for_epos:
                    if self.config.has_section(section):
                        for key, value in self.config[section]:
                            f.write(f'{key}={value}\n\n')       # no space is allowed between the key and the value
            
            f.close()
        except Exception as e:
            print(f"An error occurred while writing the config file: {str(e)}")


# Example
#config = configManager("generation.properties")
#config.write_pathgen_config_file()
#config.write_epos_config_file()