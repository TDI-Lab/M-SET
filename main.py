"""
Workflow executed from here.
"""
import numpy as np

from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths()

    total_sensing = np.zeros((1, len(plans[0][1])))
    for plan in plans:
        total_sensing += np.array(plan[1])
        print(plan)
    print(total_sensing)


if __name__ == '__main__':
    main()
