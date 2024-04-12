# Drones-Testbed

##  Setup
In order to run the DSM (Drone Sensing Mission) workflow, the config file ```drone_sense.properties``` 
must be created.  To do this, run either
```python3 setup.py```<br>
or <br>
```python setup.py``` <br>
After, the self documented config file should appear in the root of the project.

## Key Things to Note
There is a difference between `MissionName` and `MissionFile`.  `MissionName` refers to the
name of the directory generated during path generation and EPOS, it will store all generated paths and plans.
`MissionFile` is the file that all the original sensing requirements come from, and it must be an absolute path.
Hence, the sensing requirements file and the mission name don't need to be the same.