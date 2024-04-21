def experiment_num_visited_cells_collisions(self):
    #  Set target directories
    parent_path = Path(__file__).parent.resolve()
    properties_path = f"{parent_path}/../drone_sense.properties"
    #  Set EPOS properties
    collision_probabilities = []
    config = ConfigManager()
    low = 2
    high = 15
    for num_visited_cells in range(low, high + 1):
        #  Set system properties
        config.set_target_path(properties_path)
        system_conf = deepcopy(self.STANDARD_SYSTEM_CONF)
        system_conf["global"]["MissionName"] = f"5x5"
        system_conf["global"]["MissionFile"] = f"{parent_path}/../examples/5x5.csv"
        system_conf["global"]["NumberOfAgents"] = 12
        system_conf["path_generation"]["MaximumNumberOfVisitedCells"] = num_visited_cells
        system_conf["epos"]["PlanDimension"] = 25
        system_conf["epos"]["NumberOfSimulations"] = 50
        system_conf["epos"]["IterationsPerSimulation"] = 8
        system_conf["path_generation"]["NumberOfPlans"] = 64
        system_conf["epos"]["globalCostFunction"] = "MIS"
        system_conf["drone"]["BatteryCapacity"] = 270000
        config.write_config_file(system_conf)
        collision_probabilities.append(self.__generate_collision_probabilities())

    #  Plot!
    means = np.array([i[0] for i in collision_probabilities])
    stds = np.array([i[1] for i in collision_probabilities])
    lowers = means - stds
    uppers = means + stds
    sizes = [i for i in range(low, high + 1)]
    plt.plot(sizes, means)
    plt.fill_between(sizes,
                     lowers,
                     uppers,
                     color="b", alpha=.15)
    plt.grid()
    plt.title("Affect of # Visited Cells on Collision Rates")
    plt.xlabel("# Visited Cells")
    plt.ylabel("% Chance of Drone Collision per Step")
    plt.show()