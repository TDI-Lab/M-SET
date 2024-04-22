import os
from matplotlib import pyplot as plt
import pandas as pd

class VisualiseData:
    def __init__(self, paths_to_results, path_to_save):
        self.path_to_results = paths_to_results
        self.path_to_save = path_to_save
        self.df = None

        self.read_results(self.path_to_results)

    def read_results(self, paths):

        # If paths is a string, make it a one-item list
        if isinstance(paths, str):
            paths = [paths]
        #use pandas to read csv file
        for path in paths:
            if self.df is None:
                self.df = pd.read_csv(path)
            else:
                self.df = pd.concat([self.df, pd.read_csv(path)])
        # self.df = pd.read_csv(path)

    def plotNumAgentsVsSensingAccuracy(self, map_name):
        #plot a line graph where one axis is the number of agents and the other is the sensing accuracy
        # for each strategy

        #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Sensing Mismatch %'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['n_drones'], strategy_data['Sensing Mismatch %'], label=strategy)

        plt.xlabel('Number of Agents')
        plt.ylabel('Sensing Mismatch (%)')
        plt.title('Number of Agents vs Sensing Mismatch % for ' + map_name)
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_sensing_accuracy_'+map_name+'.png')
        plt.show()

  
    def plotTotalDurationVsAgents(self):

        #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Total Duration of Flights'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['n_drones'], strategy_data['Total Duration of Flights'], label=strategy)

        plt.xlabel('Number of Agents')
        plt.ylabel('Total Duration of Flights (s)')
        plt.title('Number of Agents vs Total Duration of Flights (s)')
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_total_duration.png')
        plt.show()

    def plotNumAgentsVsCollisions(self):
        #plot a line graph where one axis is the number of agents and the other is the number of collisions
        # for each strategy

        #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Total Collisions'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['n_drones'], strategy_data['Total Collisions'], label=strategy)


        plt.xlabel('Number of Agents')
        plt.ylabel('Number of Collisions')
        plt.title('Number of Agents vs Number of Collisions')
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_collisions.png')
        plt.show()

    def plotSensingMismatchVsCollisions(self):
         #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'Sensing Mismatch %'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Total Collisions'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['Sensing Mismatch %'], strategy_data['Total Collisions'], label=strategy)


        plt.xlabel('% Sensing Mismatch (%)')
        plt.ylabel('Number of Collisions')
        plt.title('Sensing Mismatch % vs Number of Collisions')
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_collisions.png')
        plt.show()

    def plotNumAgentsVsRiskOfCollision(self):
        #plot a line graph where one axis is the number of agents and the other is the risk of collision
        # for each strategy

        #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Risk of Collision'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['n_drones'], strategy_data['Risk of Collision'], label=strategy)

        plt.xlabel('Number of Agents')
        plt.ylabel('Risk of Collision (%)')
        plt.title('Number of Agents vs Risk of Collision')
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_risk_of_collision.png')
        plt.show()

    def plotSensingMismatchAndDistanceTradeOff():
        pass

    def plottypesOfCollisions(self):
        #plot the number of each type of collision for each strategy

        #group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy'])

        #get mean of sensing accuracy for each num of agents per strategy
        means = grouped['Cross Collisions', 'Parallel_Collisions', 'Cell Occupied Collisions'].mean().reset_index()

        #get different strategies
        strategies = means['Strategy'].unique()

        # plot the mean sensing accuracy for each num of agents, per strategy
        for strategy in strategies:
            strategy_data = means[means['Strategy'] == strategy]
            plt.plot(strategy_data['Cross Collisions'], label='Cross Collisions')
            plt.plot(strategy_data['Parallel Collisions'], label='Parallel Collisions')
            plt.plot(strategy_data['Cell Occupied Collisions'], label='Cell Occupied Collisions')

        plt.xlabel('Strategy')
        plt.ylabel('Number of Collisions')
        plt.title('Types of Collisions')
        plt.legend()

        #save the plot
        plt.savefig(self.path_to_save + 'types_of_collisions.png')
        plt.show()
        
# Path: experiments/MeasureSensing.py
    

