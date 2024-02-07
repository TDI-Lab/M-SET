# A class with functions that produce a config file for each parameter set

import os

class CreateConfigFile:
    def __init__(self, file_name, storing_directory = "./", configs = None):
        self.file_name = file_name
        self.storing_directory = storing_directory
        if configs is None:
            self.configs = {
                "param1": {"value": 1111, "description": "Description of param1"},
                "param2": {"value": 2222, "description": "Description of param2"},
                "param3": {"value": 3333, "description": "Description of param3"},
            }
        else:
            self.configs = configs

    def create_config_file(self):
        file_path = os.path.join(self.storing_directory, self.file_name)  # the path of the config file

        # check if the directory already exists
        if not os.path.exists(self.storing_directory):
            # if not, create the directory
            print(f"{self.storing_directory} does not exist. Creating it now...")
            os.makedirs(self.storing_directory)

        try:
            with open(self.file_name, "w") as f:
                f.write("# Configuration file\n\n")
                for param, details in self.configs.items():
                    f.write(f"# {details['description']}\n")
                    f.write(f'{param} = {details["value"]}\n\n')
        except IOError as err:
            print(f"An error occurred while writing the file: {err}")
            return False

        return file_path
    
# Example usage
#file_name = "config1.conf"
#config = CreateConfigFile(file_name)
#config.create_config_file()
#print(f"Config file created at {config.create_config_file()}")