from random import randint


def create_regular_sensing_missions(self):
    sizes = [i for i in range(2, 13)]
    for size in sizes:
        mission_name = f"{size}x{size}_regular.csv"
        rows = ["type,id,x,y,z,value\n"]
        #  Create sensing cells
        cell_id = 0
        for i in range(1, size + 1):
            for j in range(1, size + 1):
                new_row = f"SENSE,{cell_id},{i},{j},1,1\n"
                rows.append(new_row)
                cell_id += 1
        rows.append(f"BASE,0,0,0,0,0\n")
        rows.append(f"BASE,1,{size + 1},0,0,0\n")
        rows.append(f"BASE,2,0,{size + 1},0,0\n")
        rows.append(f"BASE,3,{size + 1},{size + 1},0,0\n")
        with open(f"{self.parent_path}/../examples/{mission_name}", "w") as file:
            for row in rows:
                file.write(row)


def create_random_sensing_missions(self):
    sizes = [i for i in range(2, 13)]
    for size in sizes:
        mission_name = f"{size}x{size}_random.csv"
        rows = ["type,id,x,y,z,value\n"]
        #  Create sensing cells
        cell_id = 0
        for i in range(1, size + 1):
            for j in range(1, size + 1):
                sensing_value = randint(0, 10)
                new_row = f"SENSE,{cell_id},{i},{j},1,{sensing_value}\n"
                rows.append(new_row)
                cell_id += 1
        rows.append(f"BASE,0,0,0,0,0\n")
        rows.append(f"BASE,1,{size + 1},0,0,0\n")
        rows.append(f"BASE,2,0,{size + 1},0,0\n")
        rows.append(f"BASE,3,{size + 1},{size + 1},0,0\n")
        with open(f"{self.parent_path}/../examples/{mission_name}", "w") as file:
            for row in rows:
                file.write(row)

def create_gaussian_sensing_missions(self):
    sizes = [i for i in range(2, 13)]
    for size in sizes:
        kernel = self.__gaussian_filter(size)
        mission_name = f"{size}x{size}_gaussian.csv"
        rows = ["type,id,x,y,z,value\n"]
        #  Create sensing cells
        cell_id = 0
        for i in range(1, size + 1):
            for j in range(1, size + 1):
                sensing_value = kernel[i - 1, j - 1] * 10.
                new_row = f"SENSE,{cell_id},{i},{j},1,{sensing_value}\n"
                rows.append(new_row)
                cell_id += 1
        rows.append(f"BASE,0,0,0,0,0\n")
        rows.append(f"BASE,1,{size + 1},0,0,0\n")
        rows.append(f"BASE,2,0,{size + 1},0,0\n")
        rows.append(f"BASE,3,{size + 1},{size + 1},0,0\n")
        with open(f"{self.parent_path}/../examples/{mission_name}", "w") as file:
            for row in rows:
                file.write(row)
