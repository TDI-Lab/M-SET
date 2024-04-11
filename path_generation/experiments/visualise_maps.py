from matplotlib import pyplot as plt
import helper


figures = 1


def visualise_map(size, map_type="regular"):
    global figures
    with open(f"{helper.get_parent_path()}/../../examples/{size}x{size}_{map_type}.csv", "r",
              encoding="utf-8-sig") as file:
        lines = file.readlines()
    lines = lines[1:]
    x_points = []
    y_points = []
    sizes = []
    for line in lines:
        line = line.strip("\n").split(",")
        x = float(line[2])
        y = float(line[3])
        sensing_value = float(line[5]) * 10.
        x_points.append(x)
        y_points.append(y)
        sizes.append(sensing_value)
    plt.figure(figures)
    figures += 1
    plt.scatter(x_points, y_points, c=sizes, s=100., marker="s")


def visualise_testbed():
    global figures
    with open(f"{helper.get_parent_path()}/../../examples/testbed.csv", "r", encoding="utf-8-sig") as file:
        lines = file.readlines()
    lines = lines[1:]
    x_points = []
    y_points = []
    sizes = []
    for line in lines:
        line = line.strip("\n").split(",")
        x = float(line[2])
        y = float(line[3])
        sensing_value = float(line[5])
        x_points.append(x)
        y_points.append(y)
        sizes.append(sensing_value)
    plt.figure(figures)
    figures += 1
    plt.scatter(x_points, y_points, c=sizes, s=100., marker="s")


if __name__ == "__main__":
    num = 12
    visualise_map(num, "regular")
    visualise_map(num, "gaussian")
    visualise_map(num, "random")
    plt.show()
