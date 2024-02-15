import subprocess
from os import getcwd, mkdir
from os.path import exists, join
from pathlib import Path
from shutil import rmtree


class EPOSWrapper:

    def __init__(self):
        pass

    # Start the EPOS workflow
    def run(self):

        # Get the parent path of the current file
        parent_path = Path(__file__).parent.resolve()
        # Combine the parent path with the "output" directory
        path = parent_path / "output"

        self.cleanOutput(path)
        return self.runEPOS()

    # Run the EPOS jar file
    # Make sure the PATH to the java executable is correct
    def runEPOS(self):
        parent_path = Path(__file__).parent.resolve()
        #cmd = "/usr/bin/java -jar IEPOS.jar"       # for Ubuntu
        cmd = "java -jar IEPOS.jar"                 # for Windows
        proc = subprocess.Popen(
            cmd,
            cwd=parent_path,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        proc.wait()
        out, err = proc.communicate()
        return proc.returncode

    # Clean the output directory
    def cleanOutput(self, output_dir):
        if exists(output_dir):
            rmtree(output_dir)
        #print(output_dir)
        mkdir(output_dir)