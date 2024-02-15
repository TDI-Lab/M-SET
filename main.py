"""
Workflow executed from here.
"""
from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths()
    
    if plans is None or len(plans) == 0:
        print("No plans were generated.")
    else:
        for plan in plans:
            print(plan)


if __name__ == '__main__':
    main()
