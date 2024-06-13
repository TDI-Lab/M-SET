import os
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import seaborn as sns

new_names = {
    'basic_ca': 'Scheduling-CA',
    'no_ca': 'EPOS',
    'pf_ca': 'PF-CA'

}
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
                self.df = pd.read_csv(path, on_bad_lines='skip')
            else:
                self.df = pd.concat([self.df, pd.read_csv(path, on_bad_lines='skip')])
        # self.df = pd.read_csv(path)

    def plotNumAgentsVsSensingAccuracy(self, map_name):
        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of sensing accuracy for each num of agents per strategy
        median = grouped['Sensing Mismatch %'].median().reset_index()
        lower_quartile = grouped['Sensing Mismatch %'].quantile(0.25).reset_index()
        upper_quartile = grouped['Sensing Mismatch %'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            
            # Swap the strategy names to the corresponding ones in the new_names dictionary
            strategy_name = new_names.get(strategy, strategy)
            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Sensing Mismatch %'], linestyle='--', marker='s', markersize=10, label=strategy_name)
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Sensing Mismatch %'], linestyle='--', marker='o', markersize=8, label=strategy_name)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Sensing Mismatch %'], strategy_upper['Sensing Mismatch %'], alpha=0.3)
        plt.xticks(strategy_median['n_drones'].unique())

        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Sensing Mismatch (%)', fontsize=14)
        plt.title('Number of Drones vs Sensing Mismatch % in ' + map_name, fontsize=14)
        plt.legend(loc='upper left')  # Set the legend position to upper left

        # save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_sensing_accuracy_'+map_name+'.png', dpi=300)
        plt.show()

  
    def plotTotalDurationVsAgents(self, map_name):
        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of total duration of flights for each num of agents per strategy
        median = grouped['Total Duration of Flights'].median().reset_index()
        lower_quartile = grouped['Total Duration of Flights'].quantile(0.25).reset_index()
        upper_quartile = grouped['Total Duration of Flights'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            strategy_name = new_names.get(strategy, strategy)
            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Total Duration of Flights'], linestyle='--', marker='s', label=strategy_name, )
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Total Duration of Flights'], linestyle='--', marker='o', label=strategy_name)

            # sns.lineplot(x=strategy_median['n_drones'], y=strategy_median['Total Duration of Flights'],linestyle='--', marker='o', label=strategy_name)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Total Duration of Flights'], strategy_upper['Total Duration of Flights'], alpha=0.3)

        plt.xticks(strategy_median['n_drones'].unique())
        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Total Duration of Flights (s)', fontsize=14)
        plt.title('Number of Drones vs Total Duration of Flights (s) for ' + map_name, fontsize=14)
        plt.legend()

        # save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_total_duration_'+map_name+'.png', dpi=300)
        plt.show()

    def plotNumAgentsVsCollisions(self, map_name):
        # plot a line graph where one axis is the number of agents and the other is the number of collisions
        # for each strategy

        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of collisions for each num of agents per strategy
        median = grouped['Total Collisions'].median().reset_index()
        lower_quartile = grouped['Total Collisions'].quantile(0.25).reset_index()
        upper_quartile = grouped['Total Collisions'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            strategy_name = new_names.get(strategy, strategy)
            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Total Collisions'], linestyle='--', marker='s', label=strategy_name)
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Total Collisions'], linestyle='--', marker='o', label=strategy_name)

            # sns.lineplot(x=strategy_median['n_drones'], y=strategy_median['Total Collisions'],linestyle='--', marker='o', label=strategy)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Total Collisions'], strategy_upper['Total Collisions'], alpha=0.3)
        plt.xticks(strategy_median['n_drones'].unique())

        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Number of Collisions', fontsize=14)
        plt.title('Number of Drones vs Number of Collisions in ' + map_name, fontsize=14)
        plt.legend()

        # save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_collisions_'+map_name+'.png', dpi=300)
        plt.show()

    def plotAgents_vs_TotalDuration(self, map_name):
        # add a new column for total duration of flights + hover duration
        self.df['Total Duration + Hover'] = self.df['Total Duration of Flights'] + self.df['Total Hover Duration']

        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of total duration + hover for each num of agents per strategy
        median = grouped['Total Duration + Hover'].median().reset_index()
        lower_quartile = grouped['Total Duration + Hover'].quantile(0.25).reset_index()
        upper_quartile = grouped['Total Duration + Hover'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            strategy_name = new_names.get(strategy, strategy)
            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Total Duration + Hover'], linestyle='--', marker='s', label=strategy_name)
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Total Duration + Hover'], linestyle='--', marker='o', label=strategy_name)

            # sns.lineplot(x=strategy_median['n_drones'], y=strategy_median['Total Duration + Hover'], linestyle='--', marker='o', label=strategy)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Total Duration + Hover'], strategy_upper['Total Duration + Hover'], alpha=0.3)
        plt.xticks(strategy_median['n_drones'].unique())

        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Total Flight + Hover Duration (s)', fontsize=14)
        plt.title('Number of Drones vs Flight + Hover Duration (s) in ' + map_name, fontsize=14)
        plt.legend()

        # save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_total_duration_'+map_name+'.png', dpi=300)
        plt.show()

   

    def plotNumAgentsVsRiskOfCollision(self, map_name):
        # plot a line graph where one axis is the number of agents and the other is the risk of collision
        # for each strategy

        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of risk of collision for each num of agents per strategy
        median = grouped['Risk of Collision'].median().reset_index()
        lower_quartile = grouped['Risk of Collision'].quantile(0.25).reset_index()
        upper_quartile = grouped['Risk of Collision'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            strategy_name = new_names.get(strategy, strategy)

            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Risk of Collision'], linestyle='--', marker='s', label=strategy_name)
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Risk of Collision'], linestyle='--', marker='o', label=strategy_name)

            # plt.plot(strategy_median['n_drones'], strategy_median['Risk of Collision'], linestyle='--', marker='o', label=strategy)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Risk of Collision'], strategy_upper['Risk of Collision'], alpha=0.3)
        plt.xticks(strategy_median['n_drones'].unique())

        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Risk of Collision (%)', fontsize=14)
        plt.title('Number of Drones vs Risk of Collision in ' + map_name, fontsize=14)
        plt.legend(loc='upper left')  # Set the legend position to upper left

        # save the plot
        plt.savefig(self.path_to_save + 'num_agents_vs_risk_of_collision_'+map_name+'.png', dpi=300)
        plt.show()


    def plottypesOfCollisions(self, map_name):
        # plot the number of each type of collision for each strategy
    
        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy'])
    
        # get mean of sensing accuracy for each num of agents per strategy
        means = grouped[['Cross Collisions', 'Parallel Collisions', 'Dest. Occupied Collisions']].mean().reset_index()
    
        # get different strategies
        strategies = means['Strategy'].unique()
    
        # define the width of the bars
        bar_width = 0.2
    
        # define the labels for the bars
        labels = ['Cross Collisions', 'Parallel Collisions', 'Dest. Occupied Collisions']
    
        # create an array for the x positions of the bars
        x = np.arange(len(labels))
    
        # plot the mean sensing accuracy for each num of agents, per strategy
        for i, strategy in enumerate(strategies):
            strategy_name = new_names.get(strategy, strategy)
           
            strategy_data = means[means['Strategy'] == strategy]
            if labels[2] in strategy_data.columns:
                plt.bar(x + i * bar_width, strategy_data[labels].values[0], width=bar_width, label=strategy_name, alpha=0.8)
    
        # set the x-ticks to be the collision types, centered in the group of bars
        plt.xticks(x + bar_width, labels)
    
        plt.xlabel('Collision Type', fontsize=14)
        plt.ylabel('Average Number of Collisions', fontsize=14)
        plt.title('Types of Collisions in ' + map_name , fontsize=14)
        plt.legend()
    
        # save the plot
        plt.savefig(self.path_to_save + 'types_of_collisions_'+map_name+'.png', dpi=300)
        plt.show()


    def plotPowerEstimation(self, map_name):
        hover_power = 1.7583430197900498
        # hover_power = 1.125339532665632
        # add a new column for total duration of flights + hover duration
        self.df['Total Duration + Hover'] = self.df['Total Duration of Flights'] + self.df['Total Hover Duration']

        # multiply the total duration + hover by the hover power constant
        self.df['Energy Estimation'] = self.df['Total Duration + Hover'] * hover_power

        # group the data by strategy and number of agents
        grouped = self.df.groupby(['Strategy', 'n_drones'])

        # get median and quartiles of power estimation for each num of agents per strategy
        median = grouped['Energy Estimation'].median().reset_index()
        lower_quartile = grouped['Energy Estimation'].quantile(0.25).reset_index()
        upper_quartile = grouped['Energy Estimation'].quantile(0.75).reset_index()

        # get different strategies
        strategies = median['Strategy'].unique()

        # plot the median with error bars for each num of agents, per strategy
        for strategy in strategies:
            strategy_median = median[median['Strategy'] == strategy]
            strategy_lower = lower_quartile[lower_quartile['Strategy'] == strategy]
            strategy_upper = upper_quartile[upper_quartile['Strategy'] == strategy]
            strategy_name = new_names.get(strategy, strategy)
            if strategy_name == "EPOS":
                plt.plot(strategy_median['n_drones'], strategy_median['Energy Estimation'], linestyle='--', marker='s', label=strategy_name)
            else:
                plt.plot(strategy_median['n_drones'], strategy_median['Energy Estimation'], linestyle='--', marker='o', label=strategy_name)

            # sns.lineplot(x=strategy_median['n_drones'], y=strategy_median['Power Estimation'], linestyle='--', marker='o', label=strategy)
            plt.fill_between(strategy_median['n_drones'], strategy_lower['Energy Estimation'], strategy_upper['Energy Estimation'], alpha=0.3)
        plt.xticks(strategy_median['n_drones'].unique())

        plt.xlabel('Number of Drones', fontsize=14)
        plt.ylabel('Energy Estimation (Joules)', fontsize=14)
        plt.title('Number of Drones vs Energy Estimation in ' + map_name, fontsize=14)
        plt.legend()

        # save the plot
        plt.savefig(self.path_to_save + '3num_agents_vs_power_estimation_'+map_name+'.png', dpi=300)
        plt.show()
