import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import make_interp_spline


def result_changes_in_plans(self):
    result_data = self.__retrieve_file_data("selected-plans")
    results = result_data
    #  Sort results into their runs i.e. the simulation they come from
    runs = {}
    for result in results:
        result = result.strip("\n").split(",")
        result = [int(float(i)) for i in result]
        if result[0] not in runs:
            runs[result[0]] = [result[1:]]
        else:
            runs[result[0]].append(result[1:])
    #  Ensure that the iterations are correctly sorted
    for run, iterations in runs.items():
        iterations.sort(key=lambda x: x[0])
    #  Compute the average changes between each iteration
    changes_on_each_iteration = [[] for _ in range(len(runs))]
    for run, iterations in runs.items():
        run_changes = []
        for i in range(1, len(iterations) - 1):
            changes = 0.
            prev_iter = iterations[i - 1][1:]
            cur_iter = iterations[i][1:]
            for p, c in zip(prev_iter, cur_iter):
                if p != c:
                    changes += 1
            changes /= len(prev_iter)
            run_changes.append(changes)
        changes_on_each_iteration[run] = run_changes
    #  Compute the average change for each iteration
    final_changes = []
    for i in range(len(changes_on_each_iteration[0])):
        avg = 100. * (sum([arr[i] for arr in changes_on_each_iteration]) / len(changes_on_each_iteration))
        final_changes.append(avg)
    #  Compute the maximum number of iterations
    iterations = np.array([i for i in range(1, len(final_changes) + 1)])
    #  Plot!
    x = np.linspace(iterations.min(), iterations.max(), 1000)
    x_y_spline = make_interp_spline(iterations, final_changes)
    y = x_y_spline(x)

    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.grid()
    ax.set_xlabel("Iterations")
    ax.set_ylabel("% Change in Plans")
    ax.set_title("% Change in Plans Overtime")
    fig.savefig(f"{self.parent_path}/results/change_in_plans.png")