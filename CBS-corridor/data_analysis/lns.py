#!/usr/bin/env python3
import os
import csv
import matplotlib.pyplot as plt


colors = ['#E74C3C', '#27AE60', '#F1C40F', '#A569BD', '#5DADE2', '#E67E22', '#95A5A6', '#1E8449', '#2980B9', 'k']
markers = ['s-', 'o-', 'd-', '>-', 'x-', '*-', 'p-', 'v-', '<-', 'h']
linestyle = ['--', '-', ':', '-.']
fontsize = 12
labelsize = 12


folder = '../cmake-build-release/PythonCBS/0801/'
i = -1
for filename in os.listdir(folder):
    file_name, extension = os.path.splitext(filename)
    if extension != '.csv':
        continue
    plt.figure(0)
    plt.title(file_name)
    plt.tick_params(labelsize=labelsize)
    fig = plt.gcf()
    fig.set_size_inches(6.4, 4)
    with open(folder + file_name + extension, newline='') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',', quotechar='|')
        n_cost = []  # normalized cost
        t_runtime = []  # total runtime
        for row in reader:
            if 'normalized cost' in row and 'total runtime' in row:
                n_cost.append(row['normalized cost'])
                t_runtime.append(row['total runtime'])
        plt.plot(t_runtime, n_cost, markers[i], markerfacecolor='w',
                 color=colors[i])  #, label=file_name[-4:-1])

    plt.xlabel('Runtime (s)', fontsize=fontsize)
    plt.ylabel('Normalized cost', fontsize=fontsize)
    plt.legend(frameon=False, fontsize=fontsize)
    plt.savefig("figures/" + file_name, bbox_inches='tight')
    plt.show()