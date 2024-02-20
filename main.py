"""
Workflow executed from here.
"""
from path_generation.PathGenerator import PathGenerator


def main():
    pg = PathGenerator()
    plans = pg.generate_paths()

    sum = []
    
    if plans is None or len(plans) == 0:
        print("No plans were generated.")
    else:
        for plan in plans:
            print(plan)
            # accumulate the time of each column (aka cell in the plan)
            for i in range(len(plan[1])):
                if i >= len(sum):
                    sum.append(plan[1][i])
                else:
                    sum[i] += plan[1][i]
    
    # print the total time of the plan 
    for i in range(len(sum)):
        print("Total time of column", i, "is", sum[i])
        


if __name__ == '__main__':
    main()
