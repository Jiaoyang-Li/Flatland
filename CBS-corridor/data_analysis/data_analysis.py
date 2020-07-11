#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import numpy as np

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
conflicts = {}
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
        conflicts[algo] = {'vertex':[], 'start':[], 'corridor':[], 'chasing':[]}
    instances[algo].append(r['instance'])
    success[algo].append(r['finished_agents'] / r['agents'])
    runtime[algo].append(r['runtime'])
    agents[algo].append(r['agents'])
    algorithm[algo].append(r['algorithm'])
    cost[algo].append(r['solution_cost'] / r['finished_agents'] / (r['height'] + r['width']))
    makespan[algo].append(r['makespan'])
    deadline[algo].append(r['max_timestep'])
    conflicts[algo]['start'].append(r['num_start'])
    conflicts[algo]['corridor'].append(r['num_semi_corridor'])
    conflicts[algo]['chasing'].append(r['num_chasing'])
    conflicts[algo]['vertex'].append(r['HL_expanded'] - r['num_start'] - r['num_semi_corridor'] - r['num_chasing'])
for algo in instances.keys():
    print("\n\nAlgorithm {}".format(algo))
    print("success rate = {}".format(np.mean(success[algo])))
    print("runtime = {}".format(np.mean(runtime[algo])))
    print("normalized cost = {}".format(np.mean(cost[algo])))
    print("makespan = {}".format(np.mean(makespan[algo])))

plt.subplot(2, 2, 1)
plt.xticks(rotation=45)
plt.ylabel("success rate")
for algo in instances.keys():
    if algo == 'CBSH(1.0)_groupsize=32_priority=1' or algo == 'CBSH(1.0)_groupsize=8_priority=1':
        continue
    plt.scatter(instances[algo], success[algo], label=algo, alpha=0.3)
plt.legend()
plt.subplot(2, 2, 2)
plt.xticks(rotation=45)
plt.ylabel("runtime (s)")
for algo in instances.keys():
    if algo == 'CBSH(1.0)_groupsize=32_priority=1' or algo == 'CBSH(1.0)_groupsize=8_priority=1':
        continue
    plt.scatter(instances[algo], runtime[algo], label=algo, alpha=0.3)
    plt.hlines(240, instances[algo][0], instances[algo][-1], linestyles='dashed')
plt.subplot(2, 2, 3)
plt.xticks(rotation=45)
plt.ylabel("normalized cost")
for algo in instances.keys():
    if algo == 'CBSH(1.0)_groupsize=32_priority=1' or algo == 'CBSH(1.0)_groupsize=8_priority=1':
        continue
    plt.scatter(instances[algo], cost[algo], label=algo, alpha=0.3)
plt.subplot(2, 2, 4)
plt.xticks(rotation=45)
plt.ylabel("makespan")
for algo in instances.keys():
    if algo == 'CBSH(1.0)_groupsize=32_priority=1' or algo == 'CBSH(1.0)_groupsize=8_priority=1':
        continue
    plt.scatter(instances[algo], makespan[algo], label=algo, alpha=0.3)
    plt.scatter(instances[algo], deadline[algo], label="Max timstep", marker='x', c='k')
plt.show()


algo = 'CBSH(1.0)_groupsize=32_priority=1'
plt.subplot(1, 2, 1)
plt.xticks(rotation=45)
plt.ylabel("#resolved conflicts")
plt.title(algo)
N = len(conflicts[algo]['vertex'])
y = [conflicts[algo]['vertex'][i] + conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i] for i in range(N)]
plt.bar(instances[algo], y, label='vertex')
y = [conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i] for i in range(N)]
plt.bar(instances[algo], y, label='start')
y = [conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i] for i in range(N)]
plt.bar(instances[algo], y, label='corridor')
y = conflicts[algo]['chasing']
plt.bar(instances[algo], y, label='chasing')
plt.legend()
plt.subplot(1, 2, 2)
plt.xticks(rotation=45)
plt.ylabel("%resolved conflicts")
plt.title(algo)
N = len(conflicts[algo]['vertex'])
y = [1 for i in range(N)]
plt.bar(instances[algo], y, label='vertex')
y = [(conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i]) / (conflicts[algo]['vertex'][i] + conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i]) for i in range(N)]
plt.bar(instances[algo], y, label='start')
y = [(conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i]) / (conflicts[algo]['vertex'][i] + conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i]) for i in range(N)]
plt.bar(instances[algo], y, label='corridor')
y = [conflicts[algo]['chasing'][i] / (conflicts[algo]['vertex'][i] + conflicts[algo]['start'][i] + conflicts[algo]['corridor'][i] + conflicts[algo]['chasing'][i]) for i in range(N)]
plt.bar(instances[algo], y, label='chasing')
plt.legend()
plt.show()