def experiment_num_agents_collisions(self):
    #  Set target directories
    parent_path = Path(__file__).parent.resolve()
    properties_path = f"{parent_path}/../drone_sense.properties"
    #  Set EPOS properties
    collision_probabilities = []
    config = ConfigManager()
    low = 2
    high = 140
    for num_agents in range(low, high + 1):
        #  Set system properties
        config.set_target_path(properties_path)
        system_conf = deepcopy(self.STANDARD_SYSTEM_CONF)
        system_conf["global"]["MissionName"] = f"6x6"
        system_conf["global"]["MissionFile"] = f"{parent_path}/../examples/12x12.csv"
        system_conf["global"]["NumberOfAgents"] = num_agents
        system_conf["path_generation"]["MaximumNumberOfVisitedCells"] = 16
        system_conf["epos"]["PlanDimension"] = 144
        system_conf["epos"]["NumberOfSimulations"] = 25
        system_conf["epos"]["IterationsPerSimulation"] = 8
        system_conf["path_generation"]["NumberOfPlans"] = 64
        system_conf["epos"]["globalCostFunction"] = "MIS"
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
    plt.title("Affect of # Agents on Collision Rates")
    plt.xlabel("# Agents")
    plt.ylabel("% Chance of Drone Collision per Step")
    plt.show()