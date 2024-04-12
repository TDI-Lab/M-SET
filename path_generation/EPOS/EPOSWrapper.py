import subprocess
from os import mkdir
from os.path import exists
from pathlib import Path
from shutil import rmtree


class EPOSWrapper:

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()

    # Start the EPOS workflow
    def run(self, out=False, err=False):

        # Combine the parent path with the "output" directory
        path = f"{self.parent_path}/output"

        self.clean_output(path)
        return self.runEPOS(show_out=out, show_err=err)

    # Run the EPOS jar file
    # Make sure the PATH to the java executable is correct
    def runEPOS(self, show_out=False, show_err=False):
        cmd = "java -jar IEPOS.jar"
        proc = subprocess.Popen(
            cmd,
            cwd=self.parent_path,
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
    def clean_output(self, output_dir):
        if exists(output_dir):
            rmtree(output_dir)
        mkdir(output_dir)
