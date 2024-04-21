# Read the "global config file" and create a config file for different system configurations

import os
import configparser
from pathlib import Path
from typing import Dict

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


class ConfigManager:
    # initialize the class
    def __init__(self, *args):
        self.target_path = None
        if len(args) > 0:
            config_path = args[0]
            parent_path = Path(__file__).parent.resolve()
            config_file = f"{parent_path}/{config_path}"
            self.config_path = config_file
            self.config = configparser.ConfigParser()
            self.read_global_config()
        else:
            self.config_path = None

    # read the global config file and store the configurations in a dictionary
    def read_global_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"The config file {self.config_path} does not exist.")
        self.config.read(self.config_path)

    # get the value of a parameter from the global config file
    # section: the section of the parameter, e.g. [plan], [map], [power], [epos]
    # option: the name of the parameter, e.g. num_paths, map_file, power_file, etc.
    def get_property(self, section, option):
        return self.config.get(section, option)

    def set_property(self, section, option, value):
        if not self.config.has_section(section):
            self.config.add_section(section)
        self.config.set(section, option, value)

    def set_target_path(self, path):
        self.target_path = path

    def write_config_file(self, properties: Dict[str, Dict[str, str]]):
        new_config = Path(self.target_path)
        new_config.parent.mkdir(parents=True, exist_ok=True)
        with open(self.target_path, "w") as file:
            for section, settings in properties.items():
                #  Write property without a section
                if not isinstance(settings, dict):
                    file.writelines(f"{section}={settings}\n")
                    continue
                #  Write section
                file.writelines(f"[{section}]\n")
                for setting, value in settings.items():
                    file.writelines(f"{setting}={value}\n")
                file.writelines("\n")

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
                            f.write(f'{key}={value}\n\n')  # no space is allowed between the key and the value

            f.close()
        except Exception as e:
            print(f"An error occurred while writing the config file: {str(e)}")
