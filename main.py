"""
Workflow executed from here.
"""
import numpy as np

from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths()
    for plan in plans:
        print(plan)


if __name__ == '__main__':
    main()
