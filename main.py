"""
Workflow executed from here.
"""

from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths(raw=False)
    for plan, path in plans.items():
        print(plan, path)


if __name__ == '__main__':
    main()
