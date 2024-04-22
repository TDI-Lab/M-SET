"""
Workflow executed from here.
"""

from path_generation.PathGenerator import PathGenerator


def main():
    raw = True
    #  Hello :)  There are two steps to running the path generator (once the config is set up)
    #  First, instantiate the PathGenerator object
    pg = PathGenerator()
    #  Then, call PathGenerator.generate_paths.  For CD/CA purposes, you want raw=False (so nothing)
    plans = pg.generate_paths(raw=raw)
    if raw:
        for plan in plans:
            print(plan)
    else:
        for plan, path in plans.items():
            print(plan, path)
    print(pg.get_coordinate_position_results())


if __name__ == '__main__':
    main()
