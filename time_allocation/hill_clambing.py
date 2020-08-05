import glob, os
import pandas as pd 
import random


lns_folder = "/Users/zche0040/Codes/challenge/0805/"
param = "LNS301groupsize=5"
exe_time_file = "./execuation_time.csv"
total_time_limit = 28800
test_num = [50,50,50,40,30,30,30,30,20,20,20,10,10,10]
mcp_build_time = 2

#load data from execuation_time.csv
def get_mean_exe_time(exe_time_file):
    exe_time_csv = pd.read_csv(exe_time_file)
    exe_time_csv["test"] = [None] * len(exe_time_csv.index)
    exe_time_csv["level"] = [None] * len(exe_time_csv.index)
    for id,row in exe_time_csv.iterrows():
        instance = row["instance"]
        level = int(os.path.basename(instance).split("_")[1].split(".")[0])
        test = int(os.path.dirname(instance).split("_")[1])
        row["test"] = test
        row["level"] = level
        exe_time_csv.iloc[id] = row

    mean_exe_time = [None]*14

    test_groups = exe_time_csv.groupby("test")
    for test,test_group in test_groups:
        mean_exe_time[test] = test_group["execuation_time"].mean()
    
    return mean_exe_time

#load lns data
def get_lns_data(lns_folder):
    lns_files = glob.glob(lns_folder+"*")

    lns_data = {}

    for lns in lns_files:
        filename = os.path.basename(lns).split("_")
        test = int(filename[1])
        level = int(filename[3])
        if test not in lns_data:
            lns_data[test] = {}
        if level not in lns_data[test]:
            lns_data[test][level] = {}
        lns_pa = filename[5].split(".")[0]
        
        data = pd.read_csv(lns)
        lns_data[test][level][lns_pa] = data
    return lns_data

# load data from file
lns_data = get_lns_data(lns_folder)
mean_exe_time = get_mean_exe_time(exe_time_file)

# calculate total time based on time_setting
def get_total_time(time_setting):
    total_time = 0
    for i in range(0,len(time_setting)):
        total_time += time_setting[i] * test_num[i]
    return total_time
        

# calculate total normalized cost based on time_setting
def get_cost(time_setting,cost_list):
    for i in range(0,len(time_setting)):
        t = time_setting[i]
        level0 = lns_data[i][0][param]
        level0 = level0.loc[level0["total runtime"]<= t]
        if len(level0.index) >0:
            level0 = min(level0["normalized cost"])
        else:
            level0 = 100
        level1 = lns_data[i][1][param]
        level1 = level1.loc[level1["total runtime"]<= t]
        if len(level1.index) >0:
            level1 = min(level1["normalized cost"])
        else:
            level1 = 100
        mean = (level0+level1)/2
        cost_list[i] = mean * test_num[i]

# get new_cost for new time limit.
def get_new_cost(sel_test,new_time):
    new_cost = [0,0]
    for i in range(0,len(sel_test)) :
        t = new_time[i]
        test = sel_test[i]
        level0 = lns_data[test][0][param]
        level0 = level0.loc[level0["total runtime"]<= t]
        if len(level0.index) >0:
            level0 = min(level0["normalized cost"])
        else:
            level0 = 100
        level1 = lns_data[test][1][param]
        level1 = level1.loc[level1["total runtime"]<= t]
        if len(level1.index) >0:
            level1 = min(level1["normalized cost"])
        else:
            level1 = 100
        mean = (level0+level1)/2
        new_cost[i] = mean * test_num[test]
    return new_cost

# start hill clambing
def optimize_time():

    #get total execuation/build mcp time
    total_exe_time = 0
    for i in range(0,len(mean_exe_time)):
        total_exe_time += (mean_exe_time[i] + mcp_build_time) * test_num[i]

    
    # remaining_time for lns
    remaining_time = total_time_limit - total_exe_time

    #initial setting for each test
    initial_time = remaining_time/400
    time_setting = [initial_time]*14
    cost_list = [0]*14
    get_cost(time_setting,cost_list)
    sum_norm_cost = sum(cost_list)

    iteration = 0
    while iteration<=40000:
        sel_tests = random.sample(range(0,14),2)
        test_cost = cost_list[sel_tests[0]] + cost_list[sel_tests[1]]
        selected_time = [time_setting[sel_tests[0]], time_setting[sel_tests[1]]]

        best_time = [time_setting[sel_tests[0]], time_setting[sel_tests[1]]]
        best_cost = [cost_list[sel_tests[0]],cost_list[sel_tests[1]]]
        time_ratio = test_num[sel_tests[0]]/test_num[sel_tests[1]]

        time_change = 1
        new_time = [selected_time[0]+time_change, selected_time[1]-time_change*time_ratio]
        while new_time[0] > 5 and new_time[0]<280 and new_time[1] > 5 and new_time[1]<280:
            new_cost = get_new_cost(sel_tests,new_time)
            if sum(new_cost) < test_cost:
                best_time = new_time[:]
                best_cost = new_cost[:]
                test_cost = sum(new_cost)
            new_time[0] = selected_time[0]+time_change            
            new_time[1] = selected_time[1]-time_change*time_ratio
            time_change += 1
            iteration+=1
        time_setting[sel_tests[0]] = best_time[0]
        time_setting[sel_tests[1]] = best_time[1]
        cost_list[sel_tests[0]] = best_cost[0]
        cost_list[sel_tests[1]] = best_cost[1]
        print("Iteration: ",iteration)
        print("Update time setting: ", time_setting)
        print("Update total cost: ", sum(cost_list))
        print("Total lns time: ", get_total_time(time_setting))
    print()
    
    

optimize_time()

    
    
    