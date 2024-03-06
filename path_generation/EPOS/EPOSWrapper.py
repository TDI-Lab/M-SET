import subprocess
from os import mkdir
from os.path import exists
from pathlib import Path
from shutil import rmtree


class EPOSWrapper:

    def __init__(self):
        pass

    # Start the EPOS workflow
    def run(self, out=False, err=False):

        # Get the parent path of the current file
        parent_path = Path(__file__).parent.resolve()
        # Combine the parent path with the "output" directory
        path = parent_path / "output"

        self.cleanOutput(path)
        return self.runEPOS(show_out=out, show_err=err)

    # Run the EPOS jar file
    # Make sure the PATH to the java executable is correct
    def runEPOS(self, show_out=False, show_err=False):
        parent_path = Path(__file__).parent.resolve()
        cmd = "java -jar IEPOS.jar"
        proc = subprocess.Popen(
            cmd,
            cwd=parent_path,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        proc.wait()
        out, err = proc.communicate()
        if show_err:
            print(err.decode("utf-8"))
        if show_out:
            print(out.decode("utf-8"))
        return proc.returncode

    # Clean the output directory
    def cleanOutput(self, output_dir):
        if exists(output_dir):
            rmtree(output_dir)
        #print(output_dir)
        mkdir(output_dir)