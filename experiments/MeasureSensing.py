
import csv

class cell:
    def __init__(self,type, id, pos, value):
        self.id = id
        self.type = type
        self.position = pos
        self.x = pos[0]
        self.y = pos[1]
        self.value = float(value)
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
                if (type == 'BASE'):
                    continue
                self.cells.append(cell(type, id, [x, y, z], value))

    def get_total_sensing_value(self):
        total_value = 0
        for cell in self.cells:
            total_value += cell.value
        return total_value
    
    def add_sensed_value(self, pos, duration):
        for cell in self.cells:
            if abs(float(pos[0]) - float(cell.x)) < 0.01 and abs(float(pos[1]) - float(cell.y)) < 0.01: # Check if positions are within a small distance
                cell.measured_value += duration

    def reset_measured_values(self):
        for cell in self.cells:
            cell.measured_value = 0

    def get_sensing_percentages(self):
        total_value = self.get_total_sensing_value()
        total_mismatch = 0
        total_undersensing = 0
        total_oversensing = 0
        for cell in self.cells:
            total_mismatch += abs(cell.measured_value - cell.value)
            # if undersensed, add the difference
            if cell.measured_value < cell.value:
                total_undersensing += cell.value - cell.measured_value
            if cell.measured_value > cell.value:
                total_oversensing += cell.measured_value - cell.value
        sensing_mismatch = round(total_mismatch / total_value * 100, 2)
        undersensing_percentage = round(total_undersensing / total_value * 100, 2)
        oversensing_percentage = round(total_oversensing / total_value * 100, 2)
        sensing_data = {
            "Total Sensing mismatch percentage": sensing_mismatch,
            "Undersensing percentage": undersensing_percentage,
            "Oversensing percentage": oversensing_percentage
        }
        print("\nSensing Data: \n")
        for key, value in sensing_data.items():
            print(f"{key:50}= {value}%")
        return sensing_mismatch, undersensing_percentage,  oversensing_percentage
    
        


    def measure_sensing(self, paths):
        self.reset_measured_values()
        for path in paths:
            for p in path: # p -> [[x, y], duration]
                if p[1] > 0:
                    self.add_sensed_value(p[0], p[1])


        return self.get_sensing_percentages()


    
