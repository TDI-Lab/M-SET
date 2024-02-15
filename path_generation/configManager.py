# Read the "global config file" and create a config file for different system configurations

import os
import configparser

# These lists are used to create the config file for the plan generation and EPOS
# Everything in the lists are just for the example, the actual parameters should be defined in the global config file
sections_for_pathgen = [
    "plan",
    "map",
    "power",
]

sections_for_epos = [
    "airSpeed"
]

class configManager:
    # initialize the class
    def __init__(self, config_path):
        self.config_path = config_path
        self.configs = configparser.ConfigParser()
        self.read_global_config()

    # read the global config file and store the configurations in a dictionary
    def read_global_config(self):
        if os.path.exists(self.config_file) == False:
            raise FileNotFoundError(f"The config file {self.config_file} does not exist.")
        self.config.read(self.config_file)

        """
        # This is the old way to read the global config file
        # Since there is .properties file with different format, I will comment it out for now
        try:
            with open(config_file, "r") as f:
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
        """

    # get the value of a parameter from the global config file
    # section: the section of the parameter
    # option: the name of the parameter
    def get_property(self, section, option):
        return self.config.get(section, option)
    
    def set_property(self, section, option, value):
        if not self.config.has_section(section):
            self.config.add_section(section)
        self.config.set(section, option, value)
        
    # create the config file for the plan generation
    def write_pathgen_config_file(self, file_name = "pathGen_config.properties", storing_directory = "./"):
        save_path = os.path.join(storing_directory, file_name)

        # Filter the sections that are used for the path generation
        save_content = configparser.ConfigParser()
        for section in sections_for_pathgen:
            if self.config.has_section(section):
                save_content.add_section(section)
                for key, value in self.config[section]:
                    save_content.set(section, key, value)

        with open(save_path, "w") as f:
            save_content.write(f)



        """
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
        """

    # create the config file for the plan generation, same as the previous function
    def write_epos_config_file(self, file_name = "epos_config.properties", storing_directory = "./"):
        save_path = os.path.join(storing_directory, file_name)  # the path of the config file

        # Filter the sections that are used for the EPOS
        save_content = configparser.ConfigParser()
        for section in sections_for_epos:
            if self.config.has_section(section):
                save_content.add_section(section)
                for key, value in self.config[section]:
                    save_content.set(section, key, value)

        with open(save_path, "w") as f:
            save_content.write(f)

        """
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
        """


# Example
#config = configManager("generation.properties")
#config.write_pathgen_config_file()
#config.write_epos_config_file("hi.properties")