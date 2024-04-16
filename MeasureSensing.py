
import csv

class cell:
    def __init__(self,type, id, pos, value):
        self.id = id
        self.type = type
        self.position = pos
        self.x = pos[0]
        self.y = pos[1]
        self.value = int(value)
        self.measured_value = 0
    def __str__(self):
        return f"Cell {self.id}: x={self.x}, y={self.y}, z={self.position[2]}, value={self.value}"

class MeasureSensing:
    def __init__(self, sensing_mission_path):
        self.sensing_mission_path = sensing_mission_path
        self.cells = []
        self.read_mission_file()

    # read csv file with header type, id, x, y, z, value
    def read_mission_file(self):
        self.sensing_mission = []
        with open(self.sensing_mission_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # skip header
            for row in reader:
                self.sensing_mission.append(row)
                type, id, x, y, z, value = row
                self.cells.append(cell(type, id, [x, y, z], value))

    def get_total_sensing_value(self):
        total_value = 0
        for cell in self.cells:
            total_value += cell.value
        return total_value
    
    def add_sensed_value(self, pos, duration):
        for cell in self.cells:
            if [float(pos[0]), float(pos[1])] == [float(cell.x), float(cell.y)]: # Convert coordinates to floats before comparing
                cell.measured_value += duration

    def reset_measured_values(self):
        for cell in self.cells:
            cell.measured_value = 0

    def get_sensing_percentage(self):
        total_value = self.get_total_sensing_value()
        total_measured_value = 0
        for cell in self.cells:
            total_measured_value += abs(cell.measured_value - cell.value)
        sensing_percentage = round((abs(total_value - total_measured_value) / total_value * 100), 2)

        print(f"Total value: {total_value}")
        print(f"Total measured value: {total_measured_value}")
        print(f"Sensing percentage: {sensing_percentage}%")
        return sensing_percentage

    def measure_sensing(self, paths):
        self.reset_measured_values()
        for path in paths:
            for p in path: # p -> [[x, y], duration]
                if p[1] > 0:
                    self.add_sensed_value(p[0], p[1])


        return self.get_sensing_percentage()


    
