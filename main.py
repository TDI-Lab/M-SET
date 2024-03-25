"""
Workflow executed from here.
"""

from path_generation.PathGenerator import PathGenerator


def main():
    raw = True
    pg = PathGenerator()
    plans = pg.generate_paths(raw=raw)
    if raw:
        for plan in plans:
            print(plan)
    else:
        for plan, path in plans.items():
            print(plan, path)


if __name__ == '__main__':
    main()
