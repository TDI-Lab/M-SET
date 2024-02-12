import subprocess
from os import mkdir
from os.path import exists
from pathlib import Path
from shutil import rmtree


class EPOSWrapper:

    def __init__(self):
        pass

    def run(self):
        self.cleanOutput("path_generation/EPOS/output")
        return self.runEPOS()

    def runEPOS(self):
        parent_path = Path(__file__).parent.resolve()
        cmd = "/usr/bin/java -jar IEPOS.jar"
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

    def cleanOutput(self, output_dir):
        if exists(output_dir):
            rmtree(output_dir)
        mkdir(output_dir)
