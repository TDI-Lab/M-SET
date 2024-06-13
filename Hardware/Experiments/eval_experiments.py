#     Test 1 drone, 2 drones, 3 drones, and 4 drones using EPOS with collision avoidance (6 cells); 

# Generate original 4 drone epos path (no cdca)
def generate_epos_path():
    pg = PathGenerator()
    plans = pg.generate_paths()

    for plant, path in plans.items():
        print(f"plan: {plant}, path: {path}")

    # if n_drones == 1:
    #     plans = [plans]

    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input
    
    print("")
    for planx in parsed_plans:
        print(planx)
    print("")

    return path

# Generate the 1, 2, 3, 4 drone cdca paths
for n_drones in range(1,5):
    pass