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
i = 0

rst = {}
for filename in os.listdir(folder):
    file_name, extension = os.path.splitext(filename)
    if extension != '.csv':
        continue
    p = file_name.rfind('_')
    instance_name = file_name[:p - 1]
    algorithm = file_name[p + 1:]
    if instance_name not in rst:
        rst[instance_name] = {}
    with open(folder + file_name + extension, newline='') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',', quotechar='|')
        rst[instance_name][algorithm] = {}
        for key in reader.fieldnames:
            rst[instance_name][algorithm][key] = []
            rst[instance_name][algorithm][key] = []
        for row in reader:
            for key in reader.fieldnames:
                rst[instance_name][algorithm][key].append(float(row[key]))

for instance_name in rst:
    print(instance_name)
    plt.figure(0)
    plt.title(instance_name, fontsize=fontsize)
    plt.tick_params(labelsize=labelsize)
    fig = plt.gcf()
    fig.set_size_inches(6.4, 4)
    for i, algorithm in enumerate(rst[instance_name]):
        print(algorithm)
        plt.plot(rst[instance_name][algorithm]['total runtime'],
                 rst[instance_name][algorithm]['normalized cost'],
                 markers[i], color=colors[i], markersize=2, linewidth=1,
                 label=algorithm + "_" + str(len(rst[instance_name][algorithm]['total runtime'])) + "iters")

    plt.xlabel('Runtime (s)', fontsize=fontsize)
    plt.ylabel('Normalized cost', fontsize=fontsize)
    plt.legend(frameon=False, fontsize=fontsize)
    plt.savefig("figures/" + instance_name, bbox_inches='tight')
    plt.show()