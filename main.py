"""
Workflow executed from here.
"""
import numpy as np

from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths()
    plan_total = np.array([0. for _ in range(len(plans[0][1]))])
    for plan in plans:
        print(plan)
        plan_total += np.array(plan[1])
    target_total = np.array([5., 9., 4., 5., 4., 8.])
    print("Total Plans:", plan_total)
    print("Mismatch:", plan_total - target_total)


if __name__ == '__main__':
    main()
