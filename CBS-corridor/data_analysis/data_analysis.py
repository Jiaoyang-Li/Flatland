#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt


with open("../cmake-build-release/PythonCBS/summary.txt", 'r') as file:
    results = json.load(file)
    instances = {}
    success = {}
    runtime = {}
    agents = {}
    algorithm = {}
    cost = {}
    makespan = {}
    deadline = {}
    for r in results:
        print(r)
        algo = r['algorithm']
        if algo not in instances:
            instances[algo] = []
            success[algo] = []
            runtime[algo] = []
            agents[algo] = []
            algorithm[algo] = []
            cost[algo] = []
            makespan[algo] = []
            deadline[algo] = []

        instances[algo].append(r['instance'])
        success[algo].append(r['finished_agents'] / r['agents'])
        runtime[algo].append(r['runtime'])
        agents[algo].append(r['agents'])
        algorithm[algo].append(r['algorithm'])
        cost[algo].append(r['solution_cost'] / r['finished_agents'])
        makespan[algo].append(r['makespan'])
        deadline[algo].append(r['max_timestep'])

    for algo in instances.keys():
        print("\n\nAlgorithm {}".format(algo))
        print("success rate = {}".format(success[algo]))
        print("runtime = {}".format(runtime[algo]))
        print("average cost = {}".format(cost[algo]))
        print("makespan = {}".format(makespan[algo]))

    plt.subplot(3, 1, 1)
    plt.xticks(rotation=45)
    plt.ylabel("success rate")
    for algo in instances.keys():
        plt.scatter(instances[algo], success[algo], label=algo)
    plt.legend()
    plt.subplot(3, 1, 2)
    plt.xticks(rotation=45)
    plt.ylabel("average cost")
    for algo in instances.keys():
        plt.scatter(instances[algo], cost[algo], label=algo)
    plt.subplot(3, 1, 3)
    plt.xticks(rotation=45)
    plt.ylabel("makespan")
    for algo in instances.keys():
        plt.scatter(instances[algo], makespan[algo], label=algo)
        plt.scatter(instances[algo], deadline[algo], label="Max timstep", marker='x', c='k')
    plt.show()